#!/usr/bin/env python3
"""Remove one robot's actions from an incremental fixed-path checkpoint."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("robot", choices=[f"R{i}" for i in range(1, 9)])
    parser.add_argument(
        "--plan",
        type=Path,
        default=Path("data/fixed_paths/eight_arm_cabinet.partial.json"),
    )
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    backup = args.plan.with_suffix(args.plan.suffix + f".before_{args.robot.lower()}_replan")
    shutil.copy2(args.plan, backup)
    removed = sorted(plan.setdefault("actions", {}).pop(args.robot, {}).keys())
    plan.get("stow", {}).pop(args.robot, None)
    plan.get("stow_positions", {}).pop(args.robot, None)
    args.plan.write_text(
        json.dumps(plan, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    total = sum(len(stems) for stems in plan["actions"].values())
    print(f"removed {args.robot}: {removed}")
    print(f"retained actions: {total}/30")
    print(f"backup: {backup}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
