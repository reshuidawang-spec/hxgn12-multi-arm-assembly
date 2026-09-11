#!/usr/bin/env python3
"""Minimal interval-CBS and R7/R8 coordination validation.

This is a scheduling-layer CBS: paths are represented by already-known public
workspace occupancy windows.  Continuous robot geometry remains the runtime
collision monitor's responsibility and is outside this benchmark's claim.
"""

from __future__ import annotations

import argparse
import heapq
import json
import sys
from dataclasses import asdict, dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


@dataclass(frozen=True)
class Segment:
    zone: str
    offset: float
    duration: float


@dataclass(frozen=True)
class Agent:
    name: str
    release: float
    priority: int
    segments: tuple[Segment, ...]


@dataclass(frozen=True)
class Constraint:
    agent: str
    zone: str
    start: float
    end: float


@dataclass(frozen=True)
class Window:
    agent: str
    zone: str
    start: float
    end: float
    priority: int


@dataclass(frozen=True)
class CBSResult:
    solved: bool
    expanded_nodes: int
    added_wait: float
    cost: float
    windows: tuple[Window, ...]


def interval_overlap(a_start: float, a_end: float,
                     b_start: float, b_end: float) -> bool:
    return max(a_start, b_start) < min(a_end, b_end) - 1e-9


def low_level(agent: Agent, constraints: tuple[Constraint, ...]) -> tuple[Window, ...]:
    """Find the earliest whole-trajectory delay satisfying one agent's constraints."""
    delay = 0.0
    relevant = [item for item in constraints if item.agent == agent.name]
    while True:
        shifted = False
        for segment in agent.segments:
            start = agent.release + delay + segment.offset
            end = start + segment.duration
            for constraint in relevant:
                if constraint.zone != segment.zone:
                    continue
                if interval_overlap(start, end, constraint.start, constraint.end):
                    delay += constraint.end - start
                    shifted = True
                    break
            if shifted:
                break
        if not shifted:
            return tuple(
                Window(
                    agent.name,
                    segment.zone,
                    agent.release + delay + segment.offset,
                    agent.release + delay + segment.offset + segment.duration,
                    agent.priority,
                )
                for segment in agent.segments
            )


def earliest_conflict(windows: tuple[Window, ...]) -> tuple[Window, Window] | None:
    candidates = []
    for index, first in enumerate(windows):
        for second in windows[index + 1 :]:
            if first.agent == second.agent or first.zone != second.zone:
                continue
            if interval_overlap(first.start, first.end, second.start, second.end):
                candidates.append((max(first.start, second.start), first, second))
    if not candidates:
        return None
    _, first, second = min(candidates, key=lambda item: item[0])
    return first, second


def node_cost(agents: tuple[Agent, ...], windows: tuple[Window, ...]) -> float:
    finishes = {agent.name: agent.release for agent in agents}
    for window in windows:
        finishes[window.agent] = max(finishes[window.agent], window.end)
    # Waiting by a high-priority agent is deliberately more expensive.
    return sum(
        finishes[agent.name] + 0.25 * agent.priority * (finishes[agent.name] - agent.release)
        for agent in agents
    )


def solve_cbs(agents: tuple[Agent, ...], max_nodes: int = 100) -> CBSResult:
    counter = 0

    def plan(constraints: tuple[Constraint, ...]) -> tuple[Window, ...]:
        return tuple(window for agent in agents for window in low_level(agent, constraints))

    root_constraints: tuple[Constraint, ...] = ()
    root_windows = plan(root_constraints)
    queue: list[tuple[float, int, tuple[Constraint, ...], tuple[Window, ...]]] = []
    heapq.heappush(queue, (node_cost(agents, root_windows), counter, root_constraints, root_windows))
    expanded = 0
    while queue and expanded < max_nodes:
        cost, _, constraints, windows = heapq.heappop(queue)
        expanded += 1
        conflict = earliest_conflict(windows)
        if conflict is None:
            total_wait = sum(
                min(w.start - agent.release - segment.offset
                    for w, segment in zip(
                        (window for window in windows if window.agent == agent.name),
                        agent.segments,
                    ))
                for agent in agents
            )
            return CBSResult(True, expanded, total_wait, cost, windows)
        first, second = conflict
        overlap_start = max(first.start, second.start)
        overlap_end = min(first.end, second.end)
        for constrained in (first, second):
            counter += 1
            child_constraints = constraints + (
                Constraint(constrained.agent, constrained.zone, overlap_start, overlap_end),
            )
            child_windows = plan(child_constraints)
            heapq.heappush(
                queue,
                (node_cost(agents, child_windows), counter, child_constraints, child_windows),
            )
    return CBSResult(False, expanded, 0.0, float("inf"), ())


def cases() -> dict[str, tuple[Agent, ...]]:
    return {
        "opposite_pass": (
            Agent("R4", 0, 1, (Segment("workspace_2", 0, 12),)),
            Agent("R5", 2, 1, (Segment("workspace_2", 0, 10),)),
        ),
        "crossing_two_zones": (
            Agent("R2", 0, 1, (Segment("workspace_1", 0, 8), Segment("transfer", 9, 7))),
            Agent("R3", 1, 1, (Segment("transfer", 0, 8), Segment("workspace_1", 6, 7))),
        ),
        "urgent_yield": (
            Agent("R7_NORMAL", 0, 1, (Segment("workspace_3", 0, 14),)),
            Agent("R8_URGENT", 2, 5, (Segment("workspace_3", 0, 9),)),
        ),
        "three_arm_chain": (
            Agent("R4", 0, 1, (Segment("workspace_2", 0, 10),)),
            Agent("R5", 2, 2, (Segment("workspace_2", 0, 8), Segment("workspace_3", 9, 8))),
            Agent("R6", 4, 1, (Segment("workspace_3", 0, 12),)),
        ),
    }


def r7_r8_comparison() -> list[dict]:
    normal = Agent("R7_NORMAL", 0, 1, (Segment("workspace_3", 0, 14),))
    urgent = Agent("R8_URGENT", 2, 5, (Segment("workspace_3", 0, 9),))
    agents = (normal, urgent)
    uncoordinated = tuple(window for agent in agents for window in low_level(agent, ()))
    conflicts = 1 if earliest_conflict(uncoordinated) else 0
    serial_constraints = (Constraint("R8_URGENT", "workspace_3", 0, 14),)
    serialized = tuple(window for agent in agents for window in low_level(agent, serial_constraints))
    cbs = solve_cbs(agents)
    urgent_base_finish = 11.0
    return [
        {
            "strategy": "uncoordinated",
            "conflicts": conflicts,
            "added_wait": 0.0,
            "urgent_completion": urgent_base_finish,
        },
        {
            "strategy": "fixed_normal_first_mutex",
            "conflicts": 0,
            "added_wait": min(w.start for w in serialized if w.agent == "R8_URGENT") - 2.0,
            "urgent_completion": max(w.end for w in serialized if w.agent == "R8_URGENT"),
        },
        {
            "strategy": "priority_cbs",
            "conflicts": 0 if earliest_conflict(cbs.windows) is None else 1,
            "added_wait": cbs.added_wait,
            "urgent_completion": max(w.end for w in cbs.windows if w.agent == "R8_URGENT"),
        },
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "data" / "quick_eight_arm_validation",
    )
    args = parser.parse_args()
    results = {}
    for name, agents in cases().items():
        result = solve_cbs(agents)
        if not result.solved or earliest_conflict(result.windows) is not None:
            raise AssertionError(f"CBS failed: {name}")
        results[name] = {
            "solved": result.solved,
            "expanded_nodes": result.expanded_nodes,
            "added_wait": result.added_wait,
            "cost": result.cost,
            "windows": [asdict(window) for window in result.windows],
        }
    comparison = r7_r8_comparison()
    if comparison[0]["conflicts"] != 1 or comparison[-1]["conflicts"] != 0:
        raise AssertionError("R7/R8 coordination comparison is invalid")

    args.output.mkdir(parents=True, exist_ok=True)
    payload = {
        "scope": "region-time-window CBS only; continuous geometry excluded",
        "cases": results,
        "r7_r8_comparison": comparison,
    }
    (args.output / "cbs_summary.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    lines = [
        "# 区域级 CBS 与 R7/R8 协调快速验证",
        "",
        "> 仅验证公共区时间窗冲突消解，不替代连续几何碰撞验证。",
        "",
        "| 用例 | 是否求解 | 展开节点 | 附加等待 |",
        "|---|---:|---:|---:|",
    ]
    for name, item in results.items():
        lines.append(
            f"| {name} | {item['solved']} | {item['expanded_nodes']} | "
            f"{item['added_wait']:.1f} |"
        )
    lines.extend([
        "",
        "## R7/R8 三策略",
        "",
        "| 策略 | 剩余冲突 | 附加等待 | 急单完成时刻 |",
        "|---|---:|---:|---:|",
    ])
    for row in comparison:
        lines.append(
            f"| {row['strategy']} | {row['conflicts']} | "
            f"{row['added_wait']:.1f} | {row['urgent_completion']:.1f} |"
        )
    lines.extend([
        "",
        "四类用例均在100个节点预算内得到无时间窗冲突解；优先级CBS让普通任务让行，"
        "避免固定普通任务优先造成的急单等待。",
    ])
    (args.output / "CBS_REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"validated {len(results)} CBS cases")
    print(f"report: {args.output / 'CBS_REPORT.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
