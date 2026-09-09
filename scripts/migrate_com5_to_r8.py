#!/usr/bin/env python3
"""Remove obsolete R6 COM5 checkpoint actions after assigning COM5 to R8."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--plan",
        type=Path,
        default=Path("data/fixed_paths/eight_arm_cabinet.partial.json"),
    )
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    actions = plan.setdefault("actions", {})
    r6 = actions.setdefault("R6", {})
    removed = [stem for stem in ("COM5_PICK", "COM5_PLACE") if r6.pop(stem, None) is not None]
    r8 = actions.setdefault("R8", {})
    for stem in ("COM5_PICK", "COM5_PLACE"):
        r8.pop(stem, None)
    if not r8:
        actions.pop("R8", None)

    backup = args.plan.with_suffix(args.plan.suffix + ".before_com5_r8")
    shutil.copy2(args.plan, backup)
    args.plan.write_text(
        json.dumps(plan, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    total = sum(len(stems) for stems in actions.values())
    print(f"removed R6 actions: {removed}")
    print(f"retained actions: {total}/30")
    print(f"backup: {backup}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
