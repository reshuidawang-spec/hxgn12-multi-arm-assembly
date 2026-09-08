#!/usr/bin/env python3
"""Audit every configured task APP/TCP pair against the live stopped scene."""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import zmq
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.run_8arm_cabinet_assembly import (
    ACTION_TARGETS,
    HOME,
    MAX_DOWN_TILT,
    POINTS,
    ROBOT_IDS,
    R1_EDGE_ENDPOINTS,
    R2_RAIL_ENDPOINTS,
    R3_RAIL_ENDPOINTS,
    Scene,
    action_exclusions,
    cartesian_down_line,
    ik_candidates,
    rotate_vector,
)


def audit(host: str, port: int, selected: set[str] | None = None) -> dict:
    client = RemoteAPIClient(host=host, port=port)
    client.socket.setsockopt(zmq.RCVTIMEO, 300000)
    client.socket.setsockopt(zmq.SNDTIMEO, 300000)
    scene = Scene(client)
    sim = scene.sim
    if int(sim.getSimulationState()) != int(sim.simulation_stopped):
        raise RuntimeError("stop the simulation before auditing TCP reachability")

    rows: list[dict] = []
    scene.set_all_home()
    try:
        for robot in ROBOT_IDS:
            for stem in ACTION_TARGETS[robot]:
                action_name = f"{robot}_{stem}"
                if selected and action_name not in selected:
                    continue
                app = int(sim.getObject(POINTS[f"{robot}_{stem}_APP"]))
                tcp = int(sim.getObject(POINTS[f"{robot}_{stem}_TCP"]))
                app_position = [float(v) for v in sim.getObjectPosition(app, -1)]
                tcp_position = [float(v) for v in sim.getObjectPosition(tcp, -1)]
                app_quaternion = [
                    float(v) for v in sim.getObjectQuaternion(app, -1)
                ]
                tcp_quaternion = [
                    float(v) for v in sim.getObjectQuaternion(tcp, -1)
                ]
                dot = abs(sum(a * b for a, b in zip(app_quaternion, tcp_quaternion)))
                axis_z = rotate_vector(app_quaternion, [0.0, 0.0, 1.0])[2]
                row = {
                    "action": action_name,
                    "app": app_position,
                    "tcp": tcp_position,
                    "vertical_drop_m": app_position[2] - tcp_position[2],
                    "orientation_dot": dot,
                    "tool_axis_z": axis_z,
                    "reachable": False,
                }
                try:
                    if math.dist(app_position[:2], tcp_position[:2]) > 1e-6:
                        raise RuntimeError("APP/TCP XY mismatch")
                    if app_position[2] <= tcp_position[2]:
                        raise RuntimeError("APP is not above TCP")
                    if dot < 0.999:
                        raise RuntimeError("APP/TCP quaternion mismatch")
                    if axis_z > -math.cos(MAX_DOWN_TILT):
                        raise RuntimeError("tool axis is not vertical-down")
                    transit_exclusions, contact_exclusions = action_exclusions(
                        scene, robot, stem
                    )
                    known = {
                        "R1": R1_EDGE_ENDPOINTS,
                        "R2": R2_RAIL_ENDPOINTS,
                        "R3": R3_RAIL_ENDPOINTS,
                    }.get(robot, {}).get(stem, {}).get("app", HOME)
                    candidates = ik_candidates(
                        scene,
                        robot,
                        app_position,
                        [float(value) for value in known],
                        transit_exclusions,
                        attempts=40,
                        max_solutions=4,
                        fixed_quaternion=app_quaternion,
                        include_other_robots=False,
                    )
                    errors: list[str] = []
                    for candidate in candidates:
                        try:
                            approach = cartesian_down_line(
                                scene,
                                robot,
                                candidate,
                                app_position,
                                tcp_position,
                                contact_exclusions,
                                fixed_quaternion=app_quaternion,
                                include_other_robots=False,
                            )
                            row["reachable"] = True
                            row["app_joint_seed"] = approach[0]
                            row["tcp_joint_seed"] = approach[-1]
                            break
                        except RuntimeError as exc:
                            errors.append(str(exc))
                    if not row["reachable"]:
                        if not candidates:
                            raise RuntimeError("no collision-free APP IK branch")
                        raise RuntimeError(" | ".join(errors[-3:]))
                except RuntimeError as exc:
                    row["error"] = str(exc)
                finally:
                    scene.set_all_home()
                rows.append(row)
                state = "PASS" if row["reachable"] else "FAIL"
                print(f"[{state}] {row['action']}", flush=True)
    finally:
        scene.set_all_home()
        scene.remove_planner_script()
        # Scene helpers are runtime-only and must not remain in the editor.
        sim.removeObjects(
            [int(handle) for handle in scene.down_tips.values()]
            + [int(handle) for handle in scene.virt_tips.values()]
        )

    passed = sum(bool(row["reachable"]) for row in rows)
    return {
        "total": len(rows),
        "passed": passed,
        "failed": len(rows) - passed,
        "rows": rows,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument(
        "--actions",
        nargs="*",
        help="optional action names such as R4_PSU_PICK",
    )
    args = parser.parse_args()
    report = audit(args.host, args.port, set(args.actions or ()))
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["failed"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
