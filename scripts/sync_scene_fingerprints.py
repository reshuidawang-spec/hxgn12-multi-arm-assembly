#!/usr/bin/env python3
"""把场景字节指纹同步到流水线计划和场景契约。

CoppeliaSim GUI 每次保存(哪怕只改颜色/可见性)都会改变 ttt 字节,导致
流水线控制器以 "pipeline plan is bound to a different scene" 拒绝运行。
本脚本只重绑已经验收的流水线计划;完整单臂规划文件可能包含尚未验收
的点位,绝不在这里自动改写。仅当场景只发生颜色/可见性等持久化变化时
使用;几何或点位发生变化后必须重新做完整回放审计。

用法:
    python3 scripts/sync_scene_fingerprints.py [--scene scenes/compact_cell.ttt]
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

SCENE_FILE = REPO_ROOT / "scenes" / "compact_cell.ttt"
PLAN_FILES = [
    REPO_ROOT / "data" / "fixed_paths" / "eight_arm_cabinet.partial.json",
]
CONTRACT_FILE = REPO_ROOT / "configs" / "scene_contract.yaml"


def fingerprint(path: Path) -> dict[str, object]:
    data = path.read_bytes()
    return {"size": len(data), "sha256": hashlib.sha256(data).hexdigest()}


def sync_scene_fingerprints(scene_path: Path = SCENE_FILE) -> list[str]:
    """重绑已验收流水线资产,返回更新过的文件清单(无变化则为空)。"""
    live = fingerprint(scene_path)
    updated: list[str] = []

    for plan_file in PLAN_FILES:
        if not plan_file.is_file():
            continue
        plan = json.loads(plan_file.read_text(encoding="utf-8"))
        if plan.get("scene") != live:
            plan["scene"] = dict(live)
            plan_file.write_text(
                json.dumps(plan, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            updated.append(str(plan_file.relative_to(REPO_ROOT)))

    text = CONTRACT_FILE.read_text(encoding="utf-8")
    new_text = re.sub(
        r"^(  sha256: )[^\n]*\n(  size: )[^\n]*\n",
        lambda m: f"{m.group(1)}{live['sha256']}\n{m.group(2)}{live['size']}\n",
        text,
        count=1,
        flags=re.M,
    )
    if new_text != text:
        CONTRACT_FILE.write_text(new_text, encoding="utf-8")
        updated.append(str(CONTRACT_FILE.relative_to(REPO_ROOT)))

    return updated


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scene", type=Path, default=SCENE_FILE)
    args = parser.parse_args()
    live = fingerprint(args.scene)
    print(f"[sync] 场景指纹 {args.scene}: {live}")
    updated = sync_scene_fingerprints(args.scene)
    if not updated:
        print("[sync] 所有绑定文件已一致,无需更新")
    else:
        for name in updated:
            print(f"[sync] 已重绑: {name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
