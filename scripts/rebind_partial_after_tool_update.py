#!/usr/bin/env python3
"""Rebind a checkpoint after a collision-reducing tool-only scene update."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def fingerprint(path: Path) -> dict[str, object]:
    return {
        "size": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=Path("scenes/compact_cell.ttt"))
    parser.add_argument(
        "--plan", type=Path,
        default=Path("data/fixed_paths/eight_arm_cabinet.partial.json"),
    )
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    old = dict(plan["scene"])
    plan["scene"] = fingerprint(args.scene)
    args.plan.write_text(
        json.dumps(plan, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(f"[rebind] {old} -> {plan['scene']}")
    print(f"[rebind] retained {sum(len(v) for v in plan['actions'].values())}/30 actions")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
