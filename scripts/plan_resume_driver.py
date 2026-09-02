#!/usr/bin/env python3
"""Resilient driver for the 8-arm cycle planner.

The planner's ZMQ REQ socket degrades after roughly ten minutes of
continuous use (LINGER=0 drops a reply and corrupts the REQ state
machine).  The planner checkpoints after every action, so this driver
cycles: run the planner for a bounded window, and when it crashes (or
times out), restart CoppeliaSim and resume from the checkpoint until all
32 actions are planned.

Usage:  python3 scripts/plan_resume_driver.py [--window 720] [--max-cycles 20]
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
PARTIAL = REPO_ROOT / "data" / "fixed_paths" / "eight_arm_cabinet.partial.json"
SCENE = REPO_ROOT / "scenes" / "compact_cell.ttt"
COPPELIA = (
    "/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/coppeliaSim.sh"
)
TOTAL_ACTIONS = 32


def planned_actions() -> int:
    if not PARTIAL.is_file():
        return 0
    plan = json.loads(PARTIAL.read_text(encoding="utf-8"))
    return sum(len(value) for value in plan.get("actions", {}).values())


def port_open(port: int = 23000) -> bool:
    import socket

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(1.0)
    try:
        return sock.connect_ex(("127.0.0.1", port)) == 0
    finally:
        sock.close()


def kill_coppelia() -> None:
    subprocess.run(
        ["pkill", "-9", "-f", "/opt/CoppeliaSim"], check=False
    )
    time.sleep(3)


def launch_coppelia() -> None:
    subprocess.Popen(
        [COPPELIA, str(SCENE)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )
    for _ in range(40):
        if port_open():
            return
        time.sleep(3)
    raise RuntimeError("CoppeliaSim did not open the ZMQ port in time")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", type=int, default=720)
    parser.add_argument("--max-cycles", type=int, default=30)
    args = parser.parse_args()

    for cycle in range(1, args.max_cycles + 1):
        done = planned_actions()
        print(f"[driver] cycle {cycle}: {done}/{TOTAL_ACTIONS} actions planned")
        if done >= TOTAL_ACTIONS:
            print("[driver] plan complete")
            return 0
        launch_coppelia()
        try:
            subprocess.run(
                [
                    sys.executable,
                    str(REPO_ROOT / "scripts" / "run_8arm_cabinet_assembly.py"),
                    "--plan-only",
                ],
                cwd=str(REPO_ROOT),
                timeout=args.window,
            )
        except subprocess.TimeoutExpired:
            print("[driver] planner window elapsed; restarting instance")
        except Exception as exc:
            print(f"[driver] planner exited: {exc}")
        kill_coppelia()
        time.sleep(2)
    print("[driver] cycle budget exhausted")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
