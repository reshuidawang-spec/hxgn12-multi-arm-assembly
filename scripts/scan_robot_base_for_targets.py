#!/usr/bin/env python3
"""Scan safe base positions for a robot's configured vertical APP/TCP pairs."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.run_8arm_cabinet_assembly import (
    HOME,
    POINTS,
    Scene,
    action_exclusions,
    cartesian_down_line,
    ik_candidates,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot")
    parser.add_argument("actions", nargs="+")
    parser.add_argument("--port", type=int, default=23000)
    args = parser.parse_args()
    client = RemoteAPIClient(port=args.port)
    scene = Scene(client)
    sim = scene.sim
    root = scene.roots[args.robot]
    original = [float(v) for v in sim.getObjectPosition(root, -1)]
    conveyor = int(sim.getObject("/FiveCR5A_Cell/Conveyors/Central_Indexing_Conveyor"))
    candidates = [
        (-0.50, -0.16), (-0.50, -0.12), (-0.50, -0.08),
        (-0.45, -0.16), (-0.55, -0.16), (-0.40, -0.16),
        (-0.60, -0.16),
    ]
    found = None
    try:
        for x, y in candidates:
            sim.setObjectPosition(root, -1, [x, y, original[2]])
            scene.set_all_home()
            base_collision = any(
                int(sim.checkCollision(shape, conveyor)[0]) > 0
                for shape in scene.fixed_base_shapes[args.robot]
            )
            if base_collision:
                print(f"[BASE COLLISION] {(x, y)}", flush=True)
                continue
            passed = True
            for stem in args.actions:
                app_handle = int(sim.getObject(POINTS[f"{args.robot}_{stem}_APP"]))
                tcp_handle = int(sim.getObject(POINTS[f"{args.robot}_{stem}_TCP"]))
                app = [float(v) for v in sim.getObjectPosition(app_handle, -1)]
                tcp = [float(v) for v in sim.getObjectPosition(tcp_handle, -1)]
                quaternion = [
                    float(v) for v in sim.getObjectQuaternion(app_handle, -1)
                ]
                transit, contact = action_exclusions(scene, args.robot, stem)
                branches = ik_candidates(
                    scene, args.robot, app, list(HOME), transit,
                    attempts=32, max_solutions=3,
                    fixed_quaternion=quaternion,
                    include_other_robots=False,
                )
                action_passed = False
                for branch in branches:
                    try:
                        cartesian_down_line(
                            scene, args.robot, branch, app, tcp, contact,
                            fixed_quaternion=quaternion,
                            include_other_robots=False,
                        )
                        action_passed = True
                        break
                    except RuntimeError:
                        continue
                scene.set_all_home()
                if not action_passed:
                    passed = False
                    print(f"[FAIL] {(x, y)} {stem}", flush=True)
                    break
            if passed:
                found = (x, y)
                print(f"[PASS] base={found}", flush=True)
                break
    finally:
        sim.setObjectPosition(root, -1, original)
        scene.set_all_home()
        scene.remove_planner_script()
        sim.removeObjects(
            [int(handle) for handle in scene.down_tips.values()]
            + [int(handle) for handle in scene.virt_tips.values()]
        )
    print({"robot": args.robot, "base": found}, flush=True)
    return 0 if found else 1


if __name__ == "__main__":
    raise SystemExit(main())
