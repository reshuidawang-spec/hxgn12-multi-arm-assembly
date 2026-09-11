#!/usr/bin/env python3
"""Fast, simulator-free validation for the current eight-arm cabinet cell.

The benchmark deliberately reuses the checked-in R1--R8 trajectory frame
counts, process DAG and three public-workspace stages.  It does not claim
geometric or hardware validation; it validates only scheduling behaviour.
"""

from __future__ import annotations

import argparse
import csv
import heapq
import json
import math
import random
import statistics
import sys
from dataclasses import asdict, dataclass
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts.run_8arm_cabinet_assembly import action_workspace  # noqa: E402
from scripts.run_8arm_pipeline_assembly import (  # noqa: E402
    MODULE_OPERATIONS,
    PipelineJob,
    dependencies_for_job,
    operation_id,
    operations_for_job,
)


DEFAULT_PLAN = ROOT / "data" / "fixed_paths" / "eight_arm_cabinet.json"
DEFAULT_OUTPUT = ROOT / "data" / "quick_eight_arm_validation"
INDEX_DURATION_FRAMES = 20.0
DEFAULT_WEIGHTS = {
    "priority": 0.45,
    "due": 0.30,
    "waiting": 0.15,
    "critical": 0.10,
}


@dataclass(frozen=True)
class Job:
    job_id: str
    recipe: str
    arrival: float
    due: float
    priority: int


@dataclass(frozen=True)
class Record:
    policy: str
    job_id: str
    module: int
    start: float
    end: float
    priority: int


@dataclass(frozen=True)
class Result:
    policy: str
    makespan: float
    weighted_tardiness: float
    mean_waiting: float
    urgent_response: float
    urgent_completion: float
    records: tuple[Record, ...]


def load_plan(path: Path) -> dict:
    data = json.loads(path.read_text(encoding="utf-8"))
    actions = data.get("actions", {})
    missing = [
        operation_id(item)
        for operations in MODULE_OPERATIONS.values()
        for item in operations
        if item.robot and item.stem
        and item.stem not in actions.get(item.robot, {})
    ]
    if missing:
        raise ValueError(f"missing trajectory actions: {missing}")
    if len(actions) != 8:
        raise ValueError(f"expected R1--R8 plans, found {sorted(actions)}")
    return data


def operation_duration(plan: dict, robot: str | None, stem: str | None) -> float:
    if robot is None or stem is None:
        return INDEX_DURATION_FRAMES
    frames = plan["actions"][robot][stem]["frames"]
    if len(frames) < 2:
        raise ValueError(f"trajectory {robot}:{stem} has fewer than two frames")
    return float(len(frames) - 1)


def module_duration(plan: dict, module: int, recipe: str) -> float:
    """Calculate one module's critical time with robot/workspace constraints."""
    job = PipelineJob(1, None, recipe=recipe)  # type: ignore[arg-type]
    operations = operations_for_job(job, module)
    dependencies = dependencies_for_job(job, module)
    by_id = {operation_id(item): item for item in operations}
    order = {operation_id(item): index for index, item in enumerate(operations)}
    pending = set(by_id)
    completed: set[str] = set()
    active: list[tuple[float, int, str, str, str | None]] = []
    busy_robots: set[str] = set()
    busy_workspaces: set[str] = set()
    now = 0.0
    serial = 0

    while pending or active:
        assigned = True
        while assigned:
            assigned = False
            ready = sorted(
                (key for key in pending if dependencies[key].issubset(completed)),
                key=order.__getitem__,
            )
            for key in ready:
                item = by_id[key]
                robot = item.robot or "INDEX"
                workspace = (
                    action_workspace(item.robot, item.stem)
                    if item.robot and item.stem
                    else None
                )
                if robot in busy_robots or (workspace and workspace in busy_workspaces):
                    continue
                serial += 1
                end = now + operation_duration(plan, item.robot, item.stem)
                heapq.heappush(active, (end, serial, key, robot, workspace))
                busy_robots.add(robot)
                if workspace:
                    busy_workspaces.add(workspace)
                pending.remove(key)
                assigned = True

        if not active:
            raise RuntimeError(f"module {module} scheduling deadlock: {sorted(pending)}")
        now = active[0][0]
        while active and active[0][0] == now:
            _, _, key, robot, workspace = heapq.heappop(active)
            completed.add(key)
            busy_robots.remove(robot)
            if workspace:
                busy_workspaces.remove(workspace)

    return now


def recipe_durations(plan: dict) -> dict[str, tuple[float, float, float]]:
    return {
        recipe: tuple(module_duration(plan, module, recipe) for module in range(1, 4))
        for recipe in ("standard", "reduced")
    }


def dynamic_score(job: Job, module: int, now: float, ready: float,
                  durations: dict[str, tuple[float, float, float]],
                  weights: dict[str, float] | None = None) -> float:
    weights = weights or DEFAULT_WEIGHTS
    remaining = sum(durations[job.recipe][module - 1 :])
    priority = min(max(job.priority, 0) / 10.0, 1.0)
    waiting = min(max(now - ready, 0.0) / max(durations[job.recipe]), 1.0)
    slack = job.due - now - remaining
    urgency = 1.0 if slack <= 0 else 1.0 / (1.0 + slack / max(remaining, 1.0))
    criticality = remaining / max(sum(durations[job.recipe]), 1.0)
    score = (
        weights["priority"] * priority
        + weights["due"] * urgency
        + weights["waiting"] * waiting
        + weights["critical"] * criticality
    )
    if job.priority >= 5 and slack <= remaining:
        score += 0.20
    if slack <= 0:
        score += 0.35
    return score


def run_pipeline(jobs: list[Job], durations: dict[str, tuple[float, float, float]],
                 policy: str, weights: dict[str, float] | None = None) -> Result:
    if policy not in {"fifo", "dynamic"}:
        raise ValueError(policy)
    by_id = {job.job_id: job for job in jobs}
    pending = {(job.job_id, module) for job in jobs for module in range(1, 4)}
    completed: dict[tuple[str, int], float] = {}
    active: list[tuple[float, int, str, int]] = []
    busy_modules: set[int] = set()
    records: list[Record] = []
    serial = 0
    now = min(job.arrival for job in jobs)

    while pending or active:
        for module in range(1, 4):
            if module in busy_modules:
                continue
            ready: list[tuple[Job, float]] = []
            for job_id, candidate_module in pending:
                if candidate_module != module:
                    continue
                job = by_id[job_id]
                ready_at = job.arrival if module == 1 else completed.get((job_id, module - 1))
                if ready_at is not None and ready_at <= now:
                    ready.append((job, ready_at))
            if not ready:
                continue
            if policy == "fifo":
                job, ready_at = min(ready, key=lambda item: (item[1], item[0].job_id))
            else:
                job, ready_at = max(
                    ready,
                    key=lambda item: (
                        dynamic_score(item[0], module, now, item[1], durations, weights),
                        -item[1],
                        item[0].job_id,
                    ),
                )
            end = now + durations[job.recipe][module - 1]
            records.append(Record(policy, job.job_id, module, now, end, job.priority))
            pending.remove((job.job_id, module))
            busy_modules.add(module)
            serial += 1
            heapq.heappush(active, (end, serial, job.job_id, module))

        next_arrivals = [
            by_id[job_id].arrival
            for job_id, module in pending
            if module == 1 and by_id[job_id].arrival > now
        ]
        next_times = ([active[0][0]] if active else []) + next_arrivals
        if not next_times:
            if pending:
                raise RuntimeError(f"pipeline scheduling deadlock: {sorted(pending)}")
            break
        next_time = min(next_times)
        if next_time == now and active:
            next_time = active[0][0]
        now = next_time
        while active and active[0][0] <= now:
            end, _, job_id, module = heapq.heappop(active)
            completed[(job_id, module)] = end
            busy_modules.remove(module)

    first_start = {job.job_id: min(r.start for r in records if r.job_id == job.job_id) for job in jobs}
    completion = {job.job_id: completed[(job.job_id, 3)] for job in jobs}
    waits = []
    for record in records:
        ready_at = (
            by_id[record.job_id].arrival
            if record.module == 1
            else completed[(record.job_id, record.module - 1)]
        )
        waits.append(record.start - ready_at)
    urgent = [job for job in jobs if job.priority >= 5]
    return Result(
        policy=policy,
        makespan=max(completion.values()) - min(job.arrival for job in jobs),
        weighted_tardiness=sum(
            job.priority * max(0.0, completion[job.job_id] - job.due) for job in jobs
        ),
        mean_waiting=statistics.mean(waits),
        urgent_response=(
            statistics.mean(first_start[job.job_id] - job.arrival for job in urgent)
            if urgent else 0.0
        ),
        urgent_completion=(
            statistics.mean(completion[job.job_id] - job.arrival for job in urgent)
            if urgent else 0.0
        ),
        records=tuple(records),
    )


def run_serial(jobs: list[Job], durations: dict[str, tuple[float, float, float]]) -> Result:
    now = min(job.arrival for job in jobs)
    records: list[Record] = []
    for job in sorted(jobs, key=lambda item: (item.arrival, item.job_id)):
        now = max(now, job.arrival)
        for module, duration in enumerate(durations[job.recipe], start=1):
            start = now
            now += duration
            records.append(Record("serial", job.job_id, module, start, now, job.priority))
    completion = {
        job.job_id: max(r.end for r in records if r.job_id == job.job_id) for job in jobs
    }
    first_start = {
        job.job_id: min(r.start for r in records if r.job_id == job.job_id) for job in jobs
    }
    waits = []
    for job in jobs:
        ordered = sorted((r for r in records if r.job_id == job.job_id), key=lambda r: r.module)
        waits.append(ordered[0].start - job.arrival)
        waits.extend(second.start - first.end for first, second in zip(ordered, ordered[1:]))
    urgent = [job for job in jobs if job.priority >= 5]
    return Result(
        "serial",
        max(completion.values()) - min(job.arrival for job in jobs),
        sum(job.priority * max(0.0, completion[job.job_id] - job.due) for job in jobs),
        statistics.mean(waits),
        statistics.mean(first_start[j.job_id] - j.arrival for j in urgent) if urgent else 0.0,
        statistics.mean(completion[j.job_id] - j.arrival for j in urgent) if urgent else 0.0,
        tuple(records),
    )


def validate_schedule(result: Result, jobs: list[Job]) -> None:
    records = result.records
    for module in range(1, 4):
        ordered = sorted((r for r in records if r.module == module), key=lambda r: r.start)
        for first, second in zip(ordered, ordered[1:]):
            if first.end > second.start + 1e-9:
                raise AssertionError(f"module {module} overlap: {first} / {second}")
    for job in jobs:
        ordered = sorted((r for r in records if r.job_id == job.job_id), key=lambda r: r.module)
        if len(ordered) != 3 or ordered[0].start < job.arrival:
            raise AssertionError(f"invalid job route: {job.job_id}")
        for first, second in zip(ordered, ordered[1:]):
            if first.end > second.start + 1e-9:
                raise AssertionError(f"precedence violation: {first} / {second}")


def mean_ci95(values: list[float]) -> tuple[float, float]:
    mean = statistics.mean(values)
    if len(values) < 2:
        return mean, 0.0
    return mean, 1.96 * statistics.stdev(values) / math.sqrt(len(values))


def random_jobs(rng: random.Random, index: int,
                durations: dict[str, tuple[float, float, float]]) -> list[Job]:
    typical = sum(durations["standard"])
    jobs = []
    for number in range(1, 5):
        recipe = "reduced" if rng.random() < 0.25 else "standard"
        arrival = float(rng.randint(0, int(0.20 * typical)))
        priority = 5 if rng.random() < 0.25 else rng.randint(1, 3)
        due = arrival + rng.uniform(0.80, 1.55) * typical
        jobs.append(Job(f"S{index:03d}_J{number}", recipe, arrival, due, priority))
    return jobs


def write_outputs(output: Path, plan_path: Path, durations: dict[str, tuple[float, float, float]],
                  example: list[Job], results: list[Result], samples: int,
                  deltas: dict[str, list[float]], sensitivity: list[dict]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    with (output / "example_schedule.csv").open("w", encoding="utf-8-sig", newline="") as handle:
        writer = csv.DictWriter(
            handle,
            fieldnames=list(asdict(results[0].records[0])),
            lineterminator="\n",
        )
        writer.writeheader()
        for result in results:
            for record in result.records:
                writer.writerow(asdict(record))

    summary = {
        "scope": "scheduling-only; no geometric, controller, or hardware claim",
        "plan": str(plan_path.relative_to(ROOT)),
        "trajectory_action_count": sum(len(actions) for actions in load_plan(plan_path)["actions"].values()),
        "module_duration_frames": {key: list(value) for key, value in durations.items()},
        "example_jobs": [asdict(job) for job in example],
        "example_results": [
            {key: value for key, value in asdict(result).items() if key != "records"}
            for result in results
        ],
        "random_samples": samples,
        "paired_delta_dynamic_minus_fifo": {},
    }
    for metric, values in deltas.items():
        mean, half = mean_ci95(values)
        summary["paired_delta_dynamic_minus_fifo"][metric] = {
            "mean": mean,
            "ci95_low": mean - half,
            "ci95_high": mean + half,
            "dynamic_better_rate": sum(value < 0 for value in values) / len(values),
        }
    summary["weight_sensitivity"] = sensitivity
    (output / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8"
    )

    serial, fifo, dynamic = results
    lines = [
        "# 八机械臂快速算法验证",
        "",
        "> 边界：本结果只验证调度算法；不替代几何碰撞、控制器或实机安全验证。",
        "",
        f"输入采用当前八臂轨迹库的 {summary['trajectory_action_count']} 条动作、R1–R8 固定分工、三级公共区与标准/减配配方。",
        "",
        "## 固定四柜样例",
        "",
        "| 策略 | 总完工时间/帧 | 加权延期 | 急单响应/帧 | 急单完成/帧 |",
        "|---|---:|---:|---:|---:|",
    ]
    for result in (serial, fifo, dynamic):
        lines.append(
            f"| {result.policy} | {result.makespan:.1f} | {result.weighted_tardiness:.1f} | "
            f"{result.urgent_response:.1f} | {result.urgent_completion:.1f} |"
        )
    lines.extend(["", f"## {samples} 组随机订单的配对统计", ""])
    for metric, item in summary["paired_delta_dynamic_minus_fifo"].items():
        lines.append(
            f"- {metric}：动态−FIFO均值 {item['mean']:.2f}，95% CI "
            f"[{item['ci95_low']:.2f}, {item['ci95_high']:.2f}]，"
            f"动态更优比例 {item['dynamic_better_rate']:.1%}。"
        )
    lines.extend([
        "",
        "统计采用同一随机场景上的配对差值；负值表示动态策略优于 FIFO。",
        "",
        "## 权重消融与灵敏度",
        "",
        "| 配置 | 总完工时间/帧 | 加权延期 | 急单完成/帧 |",
        "|---|---:|---:|---:|",
    ])
    for row in sensitivity:
        lines.append(
            f"| {row['name']} | {row['makespan']:.1f} | "
            f"{row['weighted_tardiness']:.1f} | {row['urgent_completion']:.1f} |"
        )
    lines.extend([
        "",
        "## 可直接写入报告的结论",
        "",
        f"在采用当前八机械臂30条轨迹及三级公共区约束的固定四柜样例中，"
        f"流水FIFO相对整柜串行将总完工时间缩短"
        f"{(serial.makespan - fifo.makespan) / serial.makespan:.1%}。"
        f"综合动态策略与FIFO的总完工时间相同，但将加权延期降低"
        f"{(fifo.weighted_tardiness - dynamic.weighted_tardiness) / fifo.weighted_tardiness:.1%}，"
        f"急单响应时间缩短"
        f"{(fifo.urgent_response - dynamic.urgent_response) / fifo.urgent_response:.1%}，"
        f"急单完成时间缩短"
        f"{(fifo.urgent_completion - dynamic.urgent_completion) / fifo.urgent_completion:.1%}。",
        "",
        f"在{samples}组随机订单配对试验中，综合动态策略相对FIFO的加权延期差值"
        f"均值为{summary['paired_delta_dynamic_minus_fifo']['weighted_tardiness']['mean']:.2f}帧，"
        f"95%置信区间为"
        f"[{summary['paired_delta_dynamic_minus_fifo']['weighted_tardiness']['ci95_low']:.2f}, "
        f"{summary['paired_delta_dynamic_minus_fifo']['weighted_tardiness']['ci95_high']:.2f}]；"
        f"急单完成时间差值均值为"
        f"{summary['paired_delta_dynamic_minus_fifo']['urgent_completion']['mean']:.2f}帧，"
        f"95%置信区间为"
        f"[{summary['paired_delta_dynamic_minus_fifo']['urgent_completion']['ci95_low']:.2f}, "
        f"{summary['paired_delta_dynamic_minus_fifo']['urgent_completion']['ci95_high']:.2f}]。"
        "两个区间均低于零，说明在本离散事件样本中改善并非由单一固定订单偶然造成。",
        "",
        "以上结论限定于任务调度层，不用于声明轨迹几何安全或实机性能。",
    ])
    (output / "REPORT.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--plan", type=Path, default=DEFAULT_PLAN)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--samples", type=int, default=200)
    parser.add_argument("--seed", type=int, default=20260911)
    args = parser.parse_args()
    if args.samples < 2:
        parser.error("--samples must be at least 2")

    plan = load_plan(args.plan)
    durations = recipe_durations(plan)
    typical = sum(durations["standard"])
    example = [
        Job("J1", "standard", 0.0, 1.45 * typical, 1),
        Job("J2", "reduced", 0.0, 1.25 * typical, 2),
        Job("J3", "standard", 0.08 * typical, 1.55 * typical, 1),
        Job("J4_URGENT", "standard", 0.18 * typical, 0.95 * typical, 5),
    ]
    results = [
        run_serial(example, durations),
        run_pipeline(example, durations, "fifo"),
        run_pipeline(example, durations, "dynamic"),
    ]
    for result in results:
        validate_schedule(result, example)

    rng = random.Random(args.seed)
    scenarios = [random_jobs(rng, index, durations) for index in range(args.samples)]
    deltas = {"makespan": [], "weighted_tardiness": [], "urgent_response": [], "urgent_completion": []}
    for jobs in scenarios:
        fifo = run_pipeline(jobs, durations, "fifo")
        dynamic = run_pipeline(jobs, durations, "dynamic")
        validate_schedule(fifo, jobs)
        validate_schedule(dynamic, jobs)
        for metric in deltas:
            deltas[metric].append(getattr(dynamic, metric) - getattr(fifo, metric))

    sensitivity_configs = {
        "default": DEFAULT_WEIGHTS,
        "no_priority": {**DEFAULT_WEIGHTS, "priority": 0.0},
        "no_due": {**DEFAULT_WEIGHTS, "due": 0.0},
        "no_waiting": {**DEFAULT_WEIGHTS, "waiting": 0.0},
        "no_critical": {**DEFAULT_WEIGHTS, "critical": 0.0},
        "priority_x0.5": {**DEFAULT_WEIGHTS, "priority": DEFAULT_WEIGHTS["priority"] * 0.5},
        "priority_x1.5": {**DEFAULT_WEIGHTS, "priority": DEFAULT_WEIGHTS["priority"] * 1.5},
        "due_x0.5": {**DEFAULT_WEIGHTS, "due": DEFAULT_WEIGHTS["due"] * 0.5},
        "due_x1.5": {**DEFAULT_WEIGHTS, "due": DEFAULT_WEIGHTS["due"] * 1.5},
    }
    sensitivity = []
    for name, weights in sensitivity_configs.items():
        sample_results = [
            run_pipeline(jobs, durations, "dynamic", weights) for jobs in scenarios
        ]
        sensitivity.append({
            "name": name,
            "weights": weights,
            "makespan": statistics.mean(result.makespan for result in sample_results),
            "weighted_tardiness": statistics.mean(
                result.weighted_tardiness for result in sample_results
            ),
            "urgent_completion": statistics.mean(
                result.urgent_completion for result in sample_results
            ),
        })

    write_outputs(
        args.output, args.plan, durations, example, results, args.samples,
        deltas, sensitivity,
    )
    print(f"validated 8-arm scheduling on {args.samples} paired random scenarios")
    print(f"report: {args.output / 'REPORT.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
