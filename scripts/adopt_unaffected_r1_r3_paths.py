#!/usr/bin/env python3
"""Carry the unchanged R1-R3 prefix into the current scene checkpoint.

R4 tool-only geometry revisions change the monolithic scene fingerprint but
do not change the R1-R3 robot trees, task TCPs, workpieces, or workspaces.
This helper copies only that isolated prefix; the current-scene R4 actions and
all other checkpoint metadata remain authoritative.  A subsequent WB1
preflight is still required before full-cycle execution.
"""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path


ROBOTS = ("R1", "R2", "R3")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--source", type=Path,
        default=Path("data/fixed_paths/eight_arm_cabinet.json"),
    )
    parser.add_argument(
        "--checkpoint", type=Path,
        default=Path("data/fixed_paths/eight_arm_cabinet.partial.json"),
    )
    args = parser.parse_args()
    source = json.loads(args.source.read_text(encoding="utf-8"))
    checkpoint = json.loads(args.checkpoint.read_text(encoding="utf-8"))
    source_policy = source.get("motion_policy", {}).get("fingerprint", {}).get("sha256")
    current_policy = checkpoint.get("motion_policy", {}).get("fingerprint", {}).get("sha256")
    if source_policy != current_policy:
        raise RuntimeError("motion-policy fingerprints differ; refusing adoption")
    backup = args.checkpoint.with_suffix(args.checkpoint.suffix + ".before_r1_r3_adoption")
    shutil.copy2(args.checkpoint, backup)
    for robot in ROBOTS:
        if not source.get("actions", {}).get(robot):
            raise RuntimeError(f"source has no completed {robot} actions")
        checkpoint.setdefault("actions", {})[robot] = source["actions"][robot]
        checkpoint["stow"][robot] = source["stow"][robot]
        checkpoint["stow_positions"][robot] = source["stow_positions"][robot]
    args.checkpoint.write_text(
        json.dumps(checkpoint, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    completed = sum(len(value) for value in checkpoint["actions"].values())
    print(f"[adopt] restored unchanged R1-R3 prefix; checkpoint={completed}/30")
    print(f"[adopt] backup={backup}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
