#!/usr/bin/env python3
"""Visually replay the consecutive actions saved in the partial checkpoint.

The replay stops before the first missing action, so an interrupted incremental
planning run can be inspected without pretending that the remaining process is
executable.  The scene is left paused at the final completed frame.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.run_8arm_cabinet_assembly import (  # noqa: E402
    ACTION_TARGETS,
    ASSEMBLY_OFFSETS,
    CONTACT_STATION,
    PICK_PART,
    PLACE_PART,
    PLAN_SCHEMA_VERSION,
    REFERENCE_CENTER,
    SCENE_FILE,
    STATIONS,
    AssemblyRuntime,
    RemoteAPIClient,
    Scene,
    fingerprint,
    motion_policy_matches,
    planning_product_state,
    process_stages,
)


DEFAULT_PARTIAL_PLAN = REPO_ROOT / "data/fixed_paths/eight_arm_cabinet.partial.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--scene", type=Path, default=SCENE_FILE)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PARTIAL_PLAN)
    parser.add_argument(
        "--robot",
        choices=tuple(ACTION_TARGETS),
        help="replay this robot's consecutive completed actions in task order",
    )
    parser.add_argument(
        "--through",
        metavar="ROBOT:ACTION",
        help=(
            "stop after this completed action; by default replay through the "
            "last consecutive action in process order"
        ),
    )
    return parser.parse_args()


def completed_prefix(plan: dict) -> list[tuple[str, str]]:
    prefix: list[tuple[str, str]] = []
    for stage in process_stages():
        if len(stage) != 1:
            raise RuntimeError(f"visual prefix replay expects serial stages, got {stage}")
        robot, stem = stage[0]
        if stem not in plan.get("actions", {}).get(robot, {}):
            break
        prefix.append((robot, stem))
    return prefix


def select_prefix(
    available: list[tuple[str, str]], through: str | None
) -> list[tuple[str, str]]:
    if through is None:
        return available
    try:
        robot, stem = through.upper().split(":", 1)
    except ValueError as exc:
        raise RuntimeError("--through must use ROBOT:ACTION, for example R2:RAIL_PLACE_H") from exc
    endpoint = (robot, stem)
    if endpoint not in available:
        shown = ", ".join(f"{r}:{s}" for r, s in available) or "none"
        raise RuntimeError(
            f"requested endpoint {robot}:{stem} is not in the completed prefix; "
            f"available endpoints: {shown}"
        )
    return available[: available.index(endpoint) + 1]


def completed_robot_actions(plan: dict, robot: str) -> list[tuple[str, str]]:
    completed: list[tuple[str, str]] = []
    saved = plan.get("actions", {}).get(robot, {})
    for stem in ACTION_TARGETS[robot]:
        if stem not in saved:
            break
        completed.append((robot, stem))
    return completed


def prepare_robot_context(runtime: AssemblyRuntime, robot: str, stem: str) -> None:
    """Materialize prerequisite parts at a selected robot's work station."""
    station, installed = planning_product_state(robot, stem)
    sim = runtime.sim
    sim.setObjectPosition(
        runtime.pallet,
        -1,
        [STATIONS[station][0], STATIONS[station][1], runtime.pallet_z],
    )
    runtime.pallet_station = station
    if not installed:
        return
    runtime.assembly = int(sim.createDummy(0.025))
    sim.setObjectAlias(runtime.assembly, "Assembly_In_Process")
    sim.setObjectParent(runtime.assembly, runtime.pallet, True)
    sim.setObjectPosition(runtime.assembly, -1, STATIONS[station])
    sim.setObjectQuaternion(runtime.assembly, -1, [0.0, 0.0, 0.0, 1.0])
    sim.setObjectInt32Param(
        runtime.assembly, sim.objintparam_visibility_layer, 0
    )
    for key in installed:
        matrix = list(sim.getObjectMatrix(runtime.ref_handles[key], -1))
        for index in range(3):
            matrix[3 + index * 4] += (
                STATIONS[station][index]
                + ASSEMBLY_OFFSETS.get(key, [0.0, 0.0, 0.0])[index]
                - REFERENCE_CENTER[index]
            )
        part = runtime.part_handles[key]
        sim.setObjectParent(part, runtime.assembly, True)
        sim.setObjectMatrix(part, -1, matrix)


def replay_action(runtime: AssemblyRuntime, robot: str, stem: str) -> None:
    required_station, _ = planning_product_state(robot, stem)
    if runtime.pallet_station != required_station:
        runtime.index_pallet(required_station, wait_for=(), emits=f"{required_station}_READY")

    label = f"{robot} {stem}"
    if (robot, stem) in PICK_PART:
        runtime.execute_pair(
            [runtime.pick(robot, stem, PICK_PART[(robot, stem)])], label
        )
        return
    if (robot, stem) in PLACE_PART:
        runtime.execute_pair(
            [
                runtime.place(
                    robot,
                    stem,
                    PLACE_PART[(robot, stem)],
                    CONTACT_STATION[(robot, stem)],
                )
            ],
            label,
        )
        return
    if robot == "R7" and stem.startswith("SCREW_"):
        runtime.execute_screw(
            runtime.track(robot, stem),
            int(stem.rsplit("_", 1)[1]),
            wait_for=(),
            emits=f"{stem}_DONE",
        )
        return
    runtime.execute_pair([(runtime.track(robot, stem), lambda: None)], label)


def main() -> int:
    args = parse_args()
    scene_path = args.scene.expanduser().resolve()
    plan_path = args.plan.expanduser().resolve()
    plan = json.loads(plan_path.read_text(encoding="utf-8"))

    if int(plan.get("schema_version", 0)) != PLAN_SCHEMA_VERSION:
        raise RuntimeError(
            f"partial plan schema is {plan.get('schema_version')}, expected {PLAN_SCHEMA_VERSION}"
        )
    if not motion_policy_matches(plan):
        raise RuntimeError("partial plan motion-policy fingerprint is stale")
    if {key: plan["scene"][key] for key in ("size", "sha256")} != fingerprint(scene_path):
        raise RuntimeError("partial plan is bound to a different scene")

    available = (
        completed_robot_actions(plan, args.robot)
        if args.robot
        else completed_prefix(plan)
    )
    selected = select_prefix(available, args.through)
    if not selected:
        raise RuntimeError("partial checkpoint has no consecutive completed actions")

    print(
        "[replay] completed prefix: "
        + " -> ".join(f"{robot}:{stem}" for robot, stem in selected),
        flush=True,
    )

    client = RemoteAPIClient(args.host, args.port)
    client.timeout = 900.0
    scene = Scene(client)
    live_path = Path(
        scene.sim.getStringParam(scene.sim.stringparam_scene_path_and_name)
    ).resolve()
    if live_path != scene_path:
        raise RuntimeError(f"open scene is {live_path}, expected {scene_path}")
    state = int(scene.sim.getSimulationState())
    if state == int(scene.sim.simulation_paused):
        scene.sim.stopSimulation()
        deadline = time.monotonic() + 10.0
        while (
            int(scene.sim.getSimulationState()) != int(scene.sim.simulation_stopped)
            and time.monotonic() < deadline
        ):
            time.sleep(0.05)
        state = int(scene.sim.getSimulationState())
    if state != int(scene.sim.simulation_stopped):
        raise RuntimeError("stop the CoppeliaSim simulation before starting replay")

    runtime = AssemblyRuntime(scene, plan, 1.0)
    runtime.reset_product()
    if args.robot and selected:
        prepare_robot_context(runtime, selected[0][0], selected[0][1])
    scene.set_all_home()
    runtime.move_to_stows(simulate=False)
    scene.install_batch_script()
    started = False
    succeeded = False
    try:
        client.setStepping(True)
        scene.sim.startSimulation()
        started = True
        client.step()
        for robot, stem in selected:
            replay_action(runtime, robot, stem)
        scene.sim.pauseSimulation()
        client.setStepping(False)
        succeeded = True
    finally:
        if not succeeded and started:
            try:
                if int(scene.sim.getSimulationState()) != int(scene.sim.simulation_stopped):
                    scene.sim.stopSimulation()
            finally:
                client.setStepping(False)
        scene.remove_batch_script()
        scene.remove_planner_script()

    last_robot, last_stem = selected[-1]
    print(
        f"[done] replayed {len(selected)} completed actions; paused at "
        f"{last_robot}:{last_stem}",
        flush=True,
    )
    print("[done] press Stop in CoppeliaSim before replaying again", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
