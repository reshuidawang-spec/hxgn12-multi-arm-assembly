#!/usr/bin/env python3
"""Find a vertical-down yaw family that makes selected APP/TCP pairs reachable."""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.run_8arm_cabinet_assembly import (
    ACTION_TARGETS,
    HOME,
    POINTS,
    Scene,
    action_exclusions,
    cartesian_down_line,
    ik_candidates,
)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("actions", nargs="+")
    parser.add_argument("--port", type=int, default=23000)
    args = parser.parse_args()
    requested = set(args.actions)
    client = RemoteAPIClient(port=args.port)
    scene = Scene(client)
    sim = scene.sim
    results = {}
    scene.set_all_home()
    try:
        for robot, stems in ACTION_TARGETS.items():
            for stem in stems:
                name = f"{robot}_{stem}"
                if name not in requested:
                    continue
                app = int(sim.getObject(POINTS[f"{name}_APP"]))
                tcp = int(sim.getObject(POINTS[f"{name}_TCP"]))
                app_position = [float(v) for v in sim.getObjectPosition(app, -1)]
                tcp_position = [float(v) for v in sim.getObjectPosition(tcp, -1)]
                transit, contact = action_exclusions(scene, robot, stem)
                result = None
                for theta_deg in list(range(0, 180, 10)) + [51.1]:
                    theta = math.radians(theta_deg)
                    quaternion = [math.cos(theta), math.sin(theta), 0.0, 0.0]
                    candidates = ik_candidates(
                        scene, robot, app_position, list(HOME), transit,
                        attempts=28, max_solutions=3,
                        fixed_quaternion=quaternion,
                        include_other_robots=False,
                    )
                    for candidate in candidates:
                        try:
                            cartesian_down_line(
                                scene, robot, candidate, app_position,
                                tcp_position, contact,
                                fixed_quaternion=quaternion,
                                include_other_robots=False,
                            )
                            result = {
                                "theta_deg": theta_deg,
                                "physical_yaw_deg": 2.0 * theta_deg,
                                "quaternion": quaternion,
                            }
                            break
                        except RuntimeError:
                            continue
                    scene.set_all_home()
                    if result is not None:
                        break
                results[name] = result
                print(f"{name}: {result}", flush=True)
    finally:
        scene.set_all_home()
        scene.remove_planner_script()
        sim.removeObjects(
            [int(handle) for handle in scene.down_tips.values()]
            + [int(handle) for handle in scene.virt_tips.values()]
        )
    return 0 if all(value is not None for value in results.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
