#!/usr/bin/env python3
"""Offline down-facing reachability grid scan for the 8-robot line.

For every robot, scan a coarse XY grid over the production area at the
corridor and contact heights, testing whether a collision-free tool-down
IK solution exists.  Results are checkpointed to
``test/reachability_map.json`` and consumed by ``validate_chain.py``.

Requires a live, stopped CoppeliaSim scene (port 23000).  Each robot
takes a few minutes; the scan resumes from the checkpoint.

Usage:
    python3 test/reachability_map.py [--robot R1] [--step 0.15]
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from coppeliasim_zmqremoteapi_client import RemoteAPIClient  # noqa: E402

from run_8arm_cabinet_assembly import Scene, ik_candidates  # noqa: E402
from sim_bridge.scene_objects import ROBOT_IDS  # noqa: E402

OUT_PATH = REPO_ROOT / "test" / "reachability_map.json"

# scan domain (scene metres) and heights
X_MIN, X_MAX = -4.2, 0.6
Y_MIN, Y_MAX = -1.6, 1.6
HEIGHTS = (0.46,)  # corridor height is the binding constraint
RING_MIN, RING_MAX = 0.35, 0.88  # plausible reach ring around each base
ATTEMPTS = 1  # single stow-seeded solve, matching the planner's usage


def grid_cells(step: float) -> list[tuple[float, float]]:
    cells = []
    x = X_MIN
    while x <= X_MAX + 1e-9:
        y = Y_MIN
        while y <= Y_MAX + 1e-9:
            cells.append((round(x, 3), round(y, 3)))
            y += step
        x += step
    return cells


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot", default=None)
    parser.add_argument("--step", type=float, default=0.15)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=23000, type=int)
    args = parser.parse_args()

    client = RemoteAPIClient(args.host, args.port)
    scene = Scene(client)
    sim = scene.sim
    if int(sim.getSimulationState()) != int(sim.simulation_stopped):
        raise RuntimeError("stop the simulation before the scan")

    checkpoint: dict = {}
    if OUT_PATH.is_file():
        checkpoint = json.loads(OUT_PATH.read_text(encoding="utf-8"))
    checkpoint.setdefault("robots", {})
    checkpoint.setdefault("params", {"step": args.step, "heights": list(HEIGHTS)})

    # warm-start every solve from the robot's validated stow (the same
    # seed the motion planner uses), falling back to the zero pose.
    stows: dict[str, list[float]] = {}
    partial_path = (
        REPO_ROOT / "data" / "fixed_paths" / "eight_arm_cabinet.partial.json"
    )
    if partial_path.is_file():
        partial = json.loads(partial_path.read_text(encoding="utf-8"))
        stows = {key: value for key, value in partial.get("stow", {}).items()}

    cells = grid_cells(args.step)
    robots = [args.robot] if args.robot else list(ROBOT_IDS)
    for robot in robots:
        if robot in checkpoint["robots"]:
            print(f"{robot}: cached, skipping")
            continue
        # fresh connection per robot: the ZMQ REQ socket degrades under
        # sustained load, and object handles stay valid on the same server.
        client = RemoteAPIClient(args.host, args.port)
        scene = Scene(client)
        sim = scene.sim
        base = [float(v) for v in sim.getObjectPosition(scene.roots[robot], -1)]
        stow = stows.get(robot, [0.0] * 6)
        start = time.time()
        ok_cells: dict[str, list[list[float]]] = {
            f"{height}": [] for height in HEIGHTS
        }
        tested = 0
        for x, y in cells:
            distance = math.hypot(x - base[0], y - base[1])
            if distance < RING_MIN or distance > RING_MAX:
                continue
            for height in HEIGHTS:
                tested += 1
                try:
                    candidates = ik_candidates(
                        scene, robot, [x, y, height], stow, [],
                        attempts=ATTEMPTS, max_solutions=1,
                    )
                except Exception:
                    candidates = []
                if candidates:
                    ok_cells[f"{height}"].append([x, y])
        checkpoint["robots"][robot] = {
            "base": [round(v, 3) for v in base],
            "ok_cells": ok_cells,
            "tested": tested,
        }
        OUT_PATH.write_text(
            json.dumps(checkpoint, ensure_ascii=False, indent=1)
        )
        total = sum(len(v) for v in ok_cells.values())
        print(
            f"{robot}: {total} reachable cells ({tested} tested, "
            f"{time.time() - start:.0f}s)"
        )
    print(f"reachability map -> {OUT_PATH}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
