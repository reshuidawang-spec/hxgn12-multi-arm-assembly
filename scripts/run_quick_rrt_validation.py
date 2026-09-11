#!/usr/bin/env python3
"""Minimal offline RRT-Connect search validation on six accepted path tubes.

The validity oracle is a bounded tube around an already accepted R1--R8 joint
path.  This proves the search implementation and repeatability without a live
CoppeliaSim process; it does not re-prove geometric collision clearance.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import statistics
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_PLAN = ROOT / "data" / "fixed_paths" / "eight_arm_cabinet.json"
DEFAULT_OUTPUT = ROOT / "data" / "quick_eight_arm_validation"
CASES = (
    ("R1", "SHELL_PICK"),
    ("R2", "RAIL_PLACE_H"),
    ("R3", "RAIL_PLACE_B"),
    ("R4", "PSU_PLACE"),
    ("R6", "BREAKER_PLACE"),
    ("R8", "FILTER_PLACE"),
)


def distance(left: list[float], right: list[float]) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def edge_valid(left: list[float], right: list[float], valid, resolution: float = 0.06) -> bool:
    steps = max(1, math.ceil(distance(left, right) / resolution))
    return all(
        valid([a + (b - a) * index / steps for a, b in zip(left, right)])
        for index in range(1, steps + 1)
    )


def chain_length(tree: list[tuple[list[float], int]], index: int) -> float:
    total = 0.0
    while tree[index][1] >= 0:
        parent = tree[index][1]
        total += distance(tree[index][0], tree[parent][0])
        index = parent
    return total


def rrt_connect(reference: list[list[float]], seed: int,
                tube_radius: float = 0.32, step: float = 0.16,
                max_iterations: int = 2500) -> tuple[bool, float, int]:
    rng = random.Random(seed)
    start, goal = reference[0], reference[-1]
    # Stored trajectories deliberately unwrap wrist joints across +/-pi to
    # avoid a full-turn discontinuity.  Preserve that accepted representation
    # while bounding search to the observed branch plus a small margin.
    limits = [
        (min(item[joint] for item in reference) - 0.35,
         max(item[joint] for item in reference) + 0.35)
        for joint in range(6)
    ]
    limits[2] = (max(limits[2][0], -2.79), min(limits[2][1], 2.79))

    def valid(state: list[float]) -> bool:
        if any(not low <= value <= high for value, (low, high) in zip(state, limits)):
            return False
        return min(distance(state, item) for item in reference) <= tube_radius

    trees = [[(list(start), -1)], [(list(goal), -1)]]

    def nearest(tree, target):
        return min(range(len(tree)), key=lambda index: distance(tree[index][0], target))

    def extend(tree, target):
        parent = nearest(tree, target)
        source = tree[parent][0]
        gap = distance(source, target)
        if gap <= step:
            candidate = list(target)
        else:
            candidate = [a + (b - a) * step / gap for a, b in zip(source, target)]
        if not edge_valid(source, candidate, valid):
            return None, False
        tree.append((candidate, parent))
        return len(tree) - 1, gap <= step

    for iteration in range(1, max_iterations + 1):
        if rng.random() < 0.15:
            sample = list(trees[1][0][0])
        else:
            anchor = rng.choice(reference)
            sample = [value + rng.gauss(0.0, tube_radius / 3.0) for value in anchor]
        first_index, _ = extend(trees[0], sample)
        if first_index is None:
            trees.reverse()
            continue
        target = trees[0][first_index][0]
        second_index = None
        reached = False
        while True:
            second_index, reached = extend(trees[1], target)
            if second_index is None or reached:
                break
        if reached and second_index is not None:
            length = chain_length(trees[0], first_index) + chain_length(trees[1], second_index)
            return True, length, iteration
        trees.reverse()
    return False, float("nan"), max_iterations


def reference_prefix(action: dict) -> list[list[float]]:
    frames = [[float(value) for value in frame] for frame in action["frames"]]
    app = [float(value) for value in action["endpoint_seeds"]["app"]]
    tcp = int(action["tcp_frame"])
    app_index = min(range(tcp + 1), key=lambda index: distance(frames[index], app))
    reference = frames[: app_index + 1]
    if len(reference) < 2:
        reference = frames[: tcp + 1]
    return reference


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--seeds", type=int, default=10)
    args = parser.parse_args()
    plan = json.loads(args.plan.read_text(encoding="utf-8"))
    rows = []
    for robot, stem in CASES:
        reference = reference_prefix(plan["actions"][robot][stem])
        reference_length = sum(distance(a, b) for a, b in zip(reference, reference[1:]))
        trials = []
        for seed in range(args.seeds):
            started = time.perf_counter()
            success, length, iterations = rrt_connect(reference, 1000 * int(robot[1:]) + seed)
            trials.append({
                "seed": seed,
                "success": success,
                "planning_ms": (time.perf_counter() - started) * 1000.0,
                "path_length": length,
                "iterations": iterations,
            })
        successful = [item for item in trials if item["success"]]
        rows.append({
            "case": f"{robot}:{stem}",
            "reference_frames": len(reference),
            "reference_length": reference_length,
            "success_rate": len(successful) / len(trials),
            "mean_planning_ms": statistics.mean(item["planning_ms"] for item in trials),
            "p95_planning_ms": sorted(item["planning_ms"] for item in trials)[max(0, math.ceil(0.95 * len(trials)) - 1)],
            "mean_path_length": statistics.mean(item["path_length"] for item in successful) if successful else None,
            "mean_iterations": statistics.mean(item["iterations"] for item in trials),
            "trials": trials,
        })
    if any(row["success_rate"] < 0.9 for row in rows):
        raise AssertionError("one or more RRT cases failed the 90% acceptance threshold")
    args.output.mkdir(parents=True, exist_ok=True)
    payload = {
        "scope": "offline RRT-Connect search in accepted joint-path tubes; geometry not rechecked",
        "cases": rows,
    }
    (args.output / "rrt_summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    lines = [
        "# RRT-Connect 最小离线验证",
        "",
        "> 有效性判定采用已验收关节轨迹周围的0.32 rad参考走廊；本实验验证搜索算法，"
        "不替代CoppeliaSim连续几何碰撞检查。",
        "",
        "| 代表路径 | 成功率 | 平均规划/ms | P95/ms | 平均迭代 |",
        "|---|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['case']} | {row['success_rate']:.0%} | "
            f"{row['mean_planning_ms']:.2f} | {row['p95_planning_ms']:.2f} | "
            f"{row['mean_iterations']:.1f} |"
        )
    lines.extend([
        "",
        f"六条代表路径各运行{args.seeds}个固定随机种子，均达到不低于90%的算法搜索成功率。",
    ])
    (args.output / "RRT_REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"validated {len(rows) * args.seeds} RRT-Connect trials")
    print(f"report: {args.output / 'RRT_REPORT.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
