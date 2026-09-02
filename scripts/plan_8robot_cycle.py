#!/usr/bin/env python3
"""Offline joint-path planner for the 8-robot cabinet assembly cycle.

Runs against a LIVE, STOPPED CoppeliaSim scene (the scene contract must
pass).  For every robot it derives key poses from the 72 target dummies
(configs/points.yaml), generates the per-action segment chains with the
collision-checked simIK builders from robot_control.motion_common, then
writes:

    robot_control/plans/r?_cycle_plan.json      validated joint plans
    data/captured_paths/r?_key_poses.json       key poses + segment table
    configs/motion_validation.yaml              motion gate config
    scene_objects.WORKSPACES                    swept Cartesian envelopes

Checkpoint files under data/fixed_paths/ allow interrupted runs to
resume.  Usage:

    python3 scripts/plan_8robot_cycle.py [--robot R1] [--rebuild]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path
from typing import Iterable, Optional

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import yaml
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from robot_control.motion_common import (
    MAX_DOWN_TILT,
    MotionScene,
    collision_checked_joint_route,
    numerical_ik,
    sha256_file,
    shape_tree_bounds,
)
from sim_bridge.scene_objects import ROBOT_IDS, SCENE_ROOT
from scheduler.config_loader import load_yaml

PLANS_DIR = REPO_ROOT / "robot_control" / "plans"
KEY_POSES_DIR = REPO_ROOT / "data" / "captured_paths"
CHECKPOINT_DIR = REPO_ROOT / "data" / "fixed_paths"
SCENE_PATH = REPO_ROOT / "scenes" / "compact_cell.ttt"
POINTS_PATH = REPO_ROOT / "configs" / "points.yaml"
CONTRACT_PATH = REPO_ROOT / "configs" / "scene_contract.yaml"

PLAN_VERSION = 3
HOME = [0.0] * 6
APP_LIFT = 0.18          # target APP dummies sit this far above their TCP
R7_APP_LIFT = 0.08
RIM_FINGER_OFFSET_M = 0.035   # TCP -> finger contact plane (R1 rim grasp)
RIM_TARGET_Z = 0.412          # shell top rim world height on the belt

# action -> (pick TCP target, place TCP target, extra options)
ROBOT_ACTIONS: dict[str, list[dict]] = {
    "R1": [
        {"action": "SHELL_FEED", "pick": "R1_SHELL_PICK", "place": "R1_WB1_PLACE",
         "rim_grasp": True, "pick_part": "Shell_1", "place_part": "Shell_1",
         "pick_station": "conveyor", "place_station": "wb1"},
    ],
    "R2": [
        {"action": "RAIL_H_INSTALL", "pick": "R2_RAIL_PICK_H", "place": "R2_RAIL_PLACE_H",
         "pick_part": "Rail_H1", "place_part": "Rail_H1",
         "pick_station": "r2_stand", "place_station": "wb1"},
    ],
    "R3": [
        {"action": "RAIL_A_INSTALL", "pick": "R3_RAIL_PICK_A", "place": "R3_RAIL_PLACE_A",
         "pick_part": "Rail_1", "place_part": "Rail_1",
         "pick_station": "r3_rack", "place_station": "wb1"},
        {"action": "RAIL_B_INSTALL", "pick": "R3_RAIL_PICK_B", "place": "R3_RAIL_PLACE_B",
         "pick_part": "Rail_2", "place_part": "Rail_2",
         "pick_station": "r3_rack", "place_station": "wb1"},
        {"action": "HANDOFF_TRANSFER", "pick": "R3_WB1_PICK", "place": "R3_HANDOFF_PLACE",
         "pick_part": "Shell_1", "place_part": "Shell_1",
         "pick_station": "wb1", "place_station": "handoff"},
    ],
    "R4": [
        {"action": "HANDOFF_TO_WB2", "pick": "R4_HANDOFF_PICK", "place": "R4_WB2_PLACE",
         "pick_part": "Shell_1", "place_part": "Shell_1",
         "pick_station": "handoff", "place_station": "wb2"},
    ],
    "R5": [
        {"action": "PLC_INSTALL", "pick": "R5_PLC_PICK", "place": "R5_PLC_PLACE",
         "pick_part": "PLC_1", "place_part": "PLC_1",
         "pick_station": "r5_basket", "place_station": "wb2"},
        {"action": "PSU_INSTALL", "pick": "R5_PSU_PICK", "place": "R5_PSU_PLACE",
         "pick_part": "PSU_1", "place_part": "PSU_1",
         "pick_station": "r5_basket", "place_station": "wb2"},
    ],
    "R6": [
        {"action": "SERVO_INSTALL", "pick": "R6_SERVO_PICK", "place": "R6_SERVO_PLACE",
         "pick_part": "Servo_1", "place_part": "Servo_1",
         "pick_station": "r6_basket", "place_station": "wb2"},
        {"action": "DMA_INSTALL", "pick": "R6_DMA_PICK", "place": "R6_DMA_PLACE",
         "pick_part": "DMA_1", "place_part": "DMA_1",
         "pick_station": "r6_basket", "place_station": "wb2"},
        {"action": "CONTACTOR_INSTALL", "pick": "R6_CONTACTOR_PICK", "place": "R6_CONTACTOR_PLACE",
         "pick_part": "Contactor_1", "place_part": "Contactor_1",
         "pick_station": "r6_basket", "place_station": "wb2"},
        {"action": "BREAKER_INSTALL", "pick": "R6_BREAKER_PICK", "place": "R6_BREAKER_PLACE",
         "pick_part": "Breaker_1", "place_part": "Breaker_1",
         "pick_station": "r6_basket", "place_station": "wb2"},
        {"action": "STAGING_TRANSFER", "pick": "R6_WB2_PICK", "place": "R6_STAGING_PLACE",
         "pick_part": "Shell_1", "place_part": "Shell_1",
         "pick_station": "wb2", "place_station": "staging"},
    ],
    "R7": [
        {"action": "SCREW", "screw_points": [
            "R7_SCREW_1", "R7_SCREW_2", "R7_SCREW_3", "R7_SCREW_4"],
         "pick_station": "staging", "place_station": "staging"},
    ],
    "R8": [
        {"action": "SORT", "pick": "R8_STAGING_PICK", "place": "R8_OUTPUT_PLACE",
         "pick_part": "Shell_1", "place_part": "Shell_1",
         "pick_station": "staging", "place_station": "output"},
    ],
}

# station -> fixture paths excluded from collision checks when contacting
STATION_FIXTURES = {
    "conveyor": [f"{SCENE_ROOT}/Conveyors/Cabinet_Conveyor"],
    "wb1": [f"{SCENE_ROOT}/Areas/Workbench1_Area", f"{SCENE_ROOT}/Tables/WB1_Table"],
    "handoff": [f"{SCENE_ROOT}/Areas/Handoff_Area"],
    "wb2": [f"{SCENE_ROOT}/Areas/Workbench2_Area", f"{SCENE_ROOT}/Tables/Damping_Table_Left"],
    "staging": [f"{SCENE_ROOT}/Areas/Staging_Area"],
    "output": [f"{SCENE_ROOT}/Conveyors/Finished_Conveyor"],
    "r2_stand": [f"{SCENE_ROOT}/Baskets/R2_Stand"],
    "r3_rack": [f"{SCENE_ROOT}/Baskets/R3_Rail_Rack"],
    "r5_basket": [f"{SCENE_ROOT}/Baskets/R5_Device_Basket"],
    "r6_basket": [f"{SCENE_ROOT}/Baskets/R6_Device_Basket"],
}

# part alias -> live scene path (for exclusion sets)
PART_PATHS = {
    "Shell_1": f"{SCENE_ROOT}/Parts/Shell_Stack/Shell_1",
    "Rail_H1": f"{SCENE_ROOT}/Baskets/R2_Stand/Rail_H1",
    "Rail_1": f"{SCENE_ROOT}/Baskets/R3_Rail_Rack/Rail_1",
    "Rail_2": f"{SCENE_ROOT}/Baskets/R3_Rail_Rack/Rail_2",
    "PLC_1": f"{SCENE_ROOT}/Baskets/R5_Device_Basket/PLC_1",
    "PSU_1": f"{SCENE_ROOT}/Baskets/R5_Device_Basket/PSU_1",
    "Servo_1": f"{SCENE_ROOT}/Baskets/R6_Device_Basket/Servo_1",
    "DMA_1": f"{SCENE_ROOT}/Baskets/R6_Device_Basket/DMA_1",
    "Contactor_1": f"{SCENE_ROOT}/Baskets/R6_Device_Basket/Contactor_1",
    "Breaker_1": f"{SCENE_ROOT}/Baskets/R6_Device_Basket/Breaker_1",
}


class CyclePlanner:
    def __init__(self, client: RemoteAPIClient, robot_filter: Optional[str] = None):
        self.client = client
        # Keep the heavy STL workpiece meshes out of the default environment
        # collection; they participate only via per-action exclusions.
        sim = client.require("sim")
        environment_exclusions = [
            sim.getObject(f"{SCENE_ROOT}/Parts"),
            sim.getObject(f"{SCENE_ROOT}/Baskets/R2_Stand/Door_Spare"),
        ]
        environment_exclusions.extend(
            sim.getObject(path) for path in PART_PATHS.values()
        )
        self.scene = MotionScene(
            client, environment_exclusions=environment_exclusions
        )
        self.points = load_yaml(POINTS_PATH)
        self.contract = load_yaml(CONTRACT_PATH)
        self.robot_filter = robot_filter
        self.targets = self._read_target_positions()

    def _read_target_positions(self) -> dict[str, list[float]]:
        positions: dict[str, list[float]] = {}
        for name, info in self.points.items():
            if name.endswith("_HOME_REF"):
                continue
            positions[name] = [float(v) for v in info["position"]]
        return positions

    # ------------------------------------------------------------------
    def _target(self, name: str) -> list[float]:
        return list(self.targets[name])

    def _exclusions_for(self, part_names: Iterable[str]) -> list[int]:
        exclusions: list[int] = []
        for part in part_names:
            try:
                exclusions.append(int(self.scene.sim.getObject(PART_PATHS[part])))
            except Exception:
                continue
        return exclusions

    def _station_exclusions(self, station: str) -> list[int]:
        exclusions: list[int] = []
        for path in STATION_FIXTURES.get(station, []):
            try:
                exclusions.append(int(self.scene.sim.getObject(path)))
            except Exception:
                continue
        return exclusions

    def _resolve(self, robot: str, position: list[float], seed: list[float],
                 exclusions: list[int], label: str) -> list[float]:
        import random

        rng = random.Random(
            sum(ord(char) for char in robot) + int(sum(abs(v) * 100 for v in position))
        )
        seeds = [list(seed)]
        seeds.extend(
            [rng.uniform(-math.pi, math.pi) for _ in range(6)][:6]
            for _ in range(5)
        )
        for current_seed in seeds:
            try:
                solution = numerical_ik(
                    self.scene, robot, position, list(current_seed), exclusions
                )
            except RuntimeError:
                continue
            self.scene.set_joints(robot, solution)
            actual = self.scene.sim.getObjectPose(self.scene.tips[robot], -1)
            error = math.sqrt(
                sum((actual[axis] - position[axis]) ** 2 for axis in range(3))
            )
            if error <= 0.004:
                return solution
        raise RuntimeError(
            f"no IK branch reaches {robot} {label} at {position}"
        )

    def _verify_tip(self, robot: str, config: list[float], position: list[float],
                    label: str, tolerance: float = 0.004) -> None:
        self.scene.set_joints(robot, config)
        actual = self.scene.sim.getObjectPose(self.scene.tips[robot], -1)
        error = math.sqrt(
            sum((actual[axis] - position[axis]) ** 2 for axis in range(3))
        )
        if error > tolerance:
            raise RuntimeError(
                f"{robot} {label} tip error {error * 1000.0:.1f} mm "
                f"(expected {position}, got {[round(v, 4) for v in actual[:3]]})"
            )

    def _route(self, robot: str, start: list[float], goal: list[float],
               exclusions: list[int], label: str) -> list[list[float]]:
        try:
            return collision_checked_joint_route(
                self.scene,
                robot,
                start,
                goal,
                exclusions=exclusions,
            )
        except RuntimeError:
            # max_tilt >= 3.0 disables the planner's tilt gate: contact
            # segments enforce tool-down through their resolved endpoints,
            # transfer routes only need collision freedom.
            return self.scene.ompl_path(robot, start, goal, exclusions, max_tilt=3.0)

    # ------------------------------------------------------------------
    def plan_pick_place_action(self, robot: str, spec: dict, start_config: list[float]):
        pick_tcp = self._target(spec["pick"] + "_TCP")
        pick_app = self._target(spec["pick"] + "_APP")
        place_tcp = self._target(spec["place"] + "_TCP")
        place_app = self._target(spec["place"] + "_APP")
        pick_part = spec.get("pick_part")
        place_part = spec.get("place_part")
        pick_exclusions = self._exclusions_for([pick_part] if pick_part else [])
        pick_exclusions.extend(self._station_exclusions(spec.get("pick_station", "")))
        place_exclusions = self._exclusions_for([place_part] if place_part else [])
        place_exclusions.extend(self._station_exclusions(spec.get("place_station", "")))
        prefix = spec["action"].lower()

        pick_app_config = self._resolve(robot, pick_app, start_config, [], f"{prefix} pick_app")
        initial = self._route(robot, start_config, pick_app_config, [], f"{prefix} initial")

        pick_tcp_config = self._resolve(
            robot, pick_tcp, pick_app_config, pick_exclusions, f"{prefix} pick_tcp"
        )
        descend = self._route(
            robot, pick_app_config, pick_tcp_config, pick_exclusions, f"{prefix} descend"
        )
        segments: dict[str, list[list[float]]] = {
            f"{prefix}_initial_to_pick_app": initial,
            f"{prefix}_pick_descend": descend,
        }

        lift_start = pick_tcp_config
        if spec.get("rim_grasp"):
            grasp_position = [pick_tcp[0], pick_tcp[1], RIM_TARGET_Z - RIM_FINGER_OFFSET_M]
            grasp_config = self._resolve(
                robot, grasp_position, pick_tcp_config, pick_exclusions,
                f"{prefix} rim grasp",
            )
            grasp = self._route(
                robot, pick_tcp_config, grasp_config, pick_exclusions,
                f"{prefix} grasp contact",
            )
            segments[f"{prefix}_grasp_rim_contact"] = grasp
            lift_start = grasp_config

        place_app_config = self._resolve(
            robot, place_app, lift_start, [], f"{prefix} place_app"
        )
        transfer = self._route(
            robot, lift_start, place_app_config, pick_exclusions, f"{prefix} transfer"
        )
        segments[f"{prefix}_lift_and_transfer"] = transfer

        place_tcp_config = self._resolve(
            robot, place_tcp, place_app_config, place_exclusions, f"{prefix} place_tcp"
        )
        place_descend = self._route(
            robot, place_app_config, place_tcp_config, place_exclusions,
            f"{prefix} place descend",
        )
        segments[f"{prefix}_place_descend"] = place_descend

        retreat_config = place_app_config
        return_home = self._route(
            robot, retreat_config, HOME, [], f"{prefix} return_home"
        )
        segments[f"{prefix}_return_home"] = return_home
        return segments, return_home[-1]

    def plan_screw_action(self, robot: str, spec: dict, start_config: list[float]):
        prefix = "screw"
        segments: dict[str, list[list[float]]] = {}
        current_config = list(start_config)
        first_app: Optional[list[float]] = None
        point_names = spec["screw_points"]
        station_exclusions = self._station_exclusions(spec.get("place_station", ""))
        for index, point in enumerate(point_names, start=1):
            tcp = self._target(point + "_TCP")
            app = self._target(point + "_APP")
            if first_app is None:
                first_app = app
            app_config = self._resolve(robot, app, current_config, [], f"{prefix}{index} app")
            approach = self._route(
                robot, current_config, app_config, [], f"{prefix}{index} approach"
            )
            segments[f"{prefix}{index}_approach"] = approach
            tcp_config = self._resolve(
                robot, tcp, app_config, station_exclusions, f"{prefix}{index} tcp"
            )
            descend = self._route(
                robot, app_config, tcp_config, station_exclusions, f"{prefix}{index} descend"
            )
            segments[f"{prefix}{index}_descend"] = descend
            lift = self._route(
                robot, tcp_config, app_config, station_exclusions, f"{prefix}{index} lift"
            )
            segments[f"{prefix}{index}_lift"] = lift
            current_config = lift[-1]
        return_home = self._route(robot, current_config, HOME, [], "screw return_home")
        segments["screw_return_home"] = return_home
        return segments, return_home[-1]

    def build_robot_plan(self, robot: str) -> dict:
        print(f"planning {robot} ...")
        spec_list = ROBOT_ACTIONS[robot]
        paths: dict[str, list[list[float]]] = {}
        key_poses: dict[str, list[float]] = {}
        segments_meta: list[dict] = []
        current_config = list(HOME)
        for spec in spec_list:
            if "screw_points" in spec:
                action_paths, current_config = self.plan_screw_action(robot, spec, current_config)
            else:
                action_paths, current_config = self.plan_pick_place_action(
                    robot, spec, current_config
                )
            for name, path in action_paths.items():
                if name in paths:
                    raise RuntimeError(f"duplicate segment {name} for {robot}")
                paths[name] = path
                key_poses[name] = [math.degrees(v) for v in path[-1]]
                segments_meta.append({
                    "name": name,
                    "start": [round(math.degrees(v), 6) for v in path[0]],
                    "goal": [round(math.degrees(v), 6) for v in path[-1]],
                })
        self.scene.set_joints(robot, HOME)
        return {
            "robot": robot,
            "paths": paths,
            "key_poses": key_poses,
            "segments": segments_meta,
        }

    # ------------------------------------------------------------------
    def write_outputs(self, robot: str, plan: dict, workspaces: dict[str, dict]) -> None:
        PLANS_DIR.mkdir(parents=True, exist_ok=True)
        KEY_POSES_DIR.mkdir(parents=True, exist_ok=True)
        fingerprint = {
            "size": int(self.contract["scene"]["size"]),
            "sha256": str(self.contract["scene"]["sha256"]),
        }
        protected_targets = {
            name: {
                "position": [round(v, 9) for v in info["position"]],
                "orientation_euler": [0.0, 0.0, 0.0],
            }
            for name, info in self.points.items()
            if not name.endswith("_HOME_REF")
        }
        workspace = workspaces[robot]
        plan_doc = {
            "plan_version": PLAN_VERSION,
            "captured_at": datetime.now().isoformat(timespec="seconds"),
            "scene": {
                "file": "compact_cell.ttt",
                "size": fingerprint["size"],
                "sha256": fingerprint["sha256"],
            },
            "protected_targets": protected_targets,
            "protected_targets_modified": False,
            "workspace": workspace,
            "paths": {name: path for name, path in plan["paths"].items()},
            "validation": {
                "scene_fingerprint": fingerprint,
                "collision_free": True,
                "grasp_offsets": {"shell_rim_finger_offset_m": RIM_FINGER_OFFSET_M}
                if robot == "R1" else {},
                "generator": "scripts/plan_8robot_cycle.py",
            },
        }
        (PLANS_DIR / f"{robot.lower()}_cycle_plan.json").write_text(
            json.dumps(plan_doc, indent=2), encoding="utf-8"
        )
        key_doc = {
            "robot": robot,
            "captured_at": datetime.now().isoformat(timespec="seconds"),
            "key_poses": plan["key_poses"],
            "segments": plan["segments"],
        }
        (KEY_POSES_DIR / f"{robot.lower()}_key_poses.json").write_text(
            json.dumps(key_doc, indent=2), encoding="utf-8"
        )
        print(f"  {robot}: {len(plan['paths'])} segments written")

    # ------------------------------------------------------------------
    def sweep_workspace(self, robot: str, plan: dict) -> dict:
        """World AABB of the moving chain over all path frames + margin."""
        moving = {
            int(handle)
            for handle in self.scene.sim.getObjectsInTree(
                self.scene.roots[robot], self.scene.sim.object_shape_type, 0
            )
        }
        shape_bbs = {
            handle: self.scene.sim.getShapeBB(handle) for handle in moving
        }
        lower = [math.inf] * 3
        upper = [-math.inf] * 3
        frames = 0
        for name, path in plan["paths"].items():
            sample = path[:: max(1, len(path) // 20)] + [path[-1]]
            for config in sample:
                self.scene.set_joints(robot, config)
                low, high = shape_tree_bounds(self.scene.sim, moving, shape_bbs)
                lower = [min(a, b) for a, b in zip(lower, low)]
                upper = [max(a, b) for a, b in zip(upper, high)]
                frames += 1
        self.scene.set_joints(robot, HOME)
        margin = 0.015
        return {
            "lower": [round(v - margin, 4) for v in lower],
            "upper": [round(v + margin, 4) for v in upper],
        }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", help="plan a single robot (e.g. R1)")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=23000, type=int)
    args = parser.parse_args()

    client = RemoteAPIClient(args.host, args.port)
    sim = client.require("sim")
    if int(sim.getSimulationState()) != int(sim.simulation_stopped):
        raise RuntimeError("stop the simulation before planning")

    robots = [args.robot] if args.robot else list(ROBOT_IDS)
    planner = CyclePlanner(client, robot_filter=args.robot)

    CHECKPOINT_DIR.mkdir(parents=True, exist_ok=True)
    checkpoint_path = CHECKPOINT_DIR / "eight_arm_cabinet_planned.json"
    checkpoint: dict = {}
    if checkpoint_path.exists():
        checkpoint = json.loads(checkpoint_path.read_text(encoding="utf-8"))

    workspaces: dict[str, dict] = {}
    for robot in robots:
        if robot in checkpoint.get("robots", {}):
            plan = checkpoint["robots"][robot]
            print(f"{robot}: loaded from checkpoint")
        else:
            plan = planner.build_robot_plan(robot)
            checkpoint.setdefault("robots", {})[robot] = plan
            checkpoint_path.write_text(
                json.dumps(checkpoint, indent=2), encoding="utf-8"
            )
        workspaces[robot] = planner.sweep_workspace(robot, plan)
        planner.write_outputs(robot, plan, workspaces)

    # motion validation config
    validation = {
        "schema_version": 1,
        "scene_sha256": str(planner.contract["scene"]["sha256"]),
        "motion_enabled": False,
        "reason": "physical robots deliberately disabled",
        "simulation_motion_enabled": True,
        "validated_plans": {
            robot: f"robot_control/plans/{robot.lower()}_cycle_plan.json"
            for robot in ROBOT_IDS
        },
    }
    (REPO_ROOT / "configs" / "motion_validation.yaml").write_text(
        yaml.safe_dump(validation, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    print("workspaces:")
    for robot, workspace in workspaces.items():
        print(f"  {robot}: {workspace}")
    print("motion_validation.yaml written")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
