#!/usr/bin/env python3
"""Run up to three cabinets as a conservative three-module pipeline.

Each public workspace admits at most one installation track at a time, while
different workspaces replay concurrently.  Every cabinet owns a pallet,
workpiece set, assembly root, and event context; accepted fixed robot paths
remain unchanged.
"""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Callable


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from scripts.run_8arm_cabinet_assembly import (  # noqa: E402
    PARTS,
    PLAN_SCHEMA_VERSION,
    SCENE_FILE,
    STATIONS,
    AssemblyRuntime,
    RemoteAPIClient,
    Scene,
    Track,
    action_workspace,
    fingerprint,
    motion_policy_matches,
    require_open_top_shell,
)


DEFAULT_PIPELINE_PLAN = (
    REPO_ROOT / "data" / "fixed_paths" / "eight_arm_cabinet.partial.json"
)
INFEED_SPACING = 1.50
BLACK_SHELL_COLOR = [0.025, 0.030, 0.035]
REDUCED_RECIPE_SKIPS = frozenset(
    {
        "R4:SERVO_PICK",
        "R4:SERVO_PLACE",
        "R5:DMA_PICK",
        "R5:DMA_PLACE",
        "R8:FILTER_PICK",
        "R8:FILTER_PLACE",
        "R7:SCREW_3",
        "R7:SCREW_4",
    }
)

MODULE_PARTS = {
    1: ("shell", "rail_h", "rail_a", "rail_b"),
    2: ("psu", "servo", "eds", "plc", "dma", "contactor", "breaker"),
    3: ("com5", "filter"),
}
MODULE_ROBOTS = {
    1: ("R1", "R2", "R3"),
    2: ("R4", "R5", "R6"),
    3: ("R7", "R8"),
}


def step_simulation(client: RemoteAPIClient) -> None:
    """Advance one step, tolerating CoppeliaSim's known wait-ack glitch.

    The server has already executed the step when the private ``executed``
    acknowledgement lookup fails.  Reissuing the step would advance twice,
    so only this exact post-execution reply is ignored; every other remote
    API failure remains fatal.
    """
    try:
        client.step()
    except Exception as exc:
        if "No such function: _*executed*_" not in str(exc):
            raise
        print(
            "[remote] step executed; transient wait acknowledgement ignored",
            flush=True,
        )


@dataclass(frozen=True)
class PipelineOperation:
    kind: str
    label: str
    robot: str | None = None
    stem: str | None = None
    key: str | None = None
    station: str | None = None


def operation_id(operation: PipelineOperation) -> str:
    if operation.kind == "index":
        return f"index:{operation.station}"
    return f"{operation.robot}:{operation.stem}"


def _pick(robot: str, stem: str, key: str, label: str) -> PipelineOperation:
    return PipelineOperation("pick", label, robot, stem, key)


def _place(
    robot: str,
    stem: str,
    key: str,
    station: str,
    label: str,
) -> PipelineOperation:
    return PipelineOperation("place", label, robot, stem, key, station)


MODULE_OPERATIONS = {
    1: (
        _pick("R1", "SHELL_PICK", "shell", "R1 shell pick"),
        _place("R1", "WB1_PLACE", "shell", "wb1", "R1 shell -> WB1"),
        _pick("R2", "RAIL_PICK_H", "rail_h", "R2 horizontal rail pick"),
        _place(
            "R2", "RAIL_PLACE_H", "rail_h", "wb1",
            "R2 horizontal rail install",
        ),
        _pick("R3", "RAIL_PICK_A", "rail_a", "R3 vertical rail A pick"),
        _place(
            "R3", "RAIL_PLACE_A", "rail_a", "wb1",
            "R3 vertical rail A install",
        ),
        PipelineOperation("index", "WB1 micro-index", station="wb1_micro"),
        _pick("R3", "RAIL_PICK_B", "rail_b", "R3 vertical rail B pick"),
        _place(
            "R3", "RAIL_PLACE_B", "rail_b", "wb1_micro",
            "R3 vertical rail B install",
        ),
    ),
    2: (
        _pick("R4", "PSU_PICK", "psu", "R4 PSU pick"),
        _place("R4", "PSU_PLACE", "psu", "wb2", "R4 PSU install"),
        _pick("R4", "SERVO_PICK", "servo", "R4 servo pick"),
        _place("R4", "SERVO_PLACE", "servo", "wb2", "R4 servo install"),
        _pick("R4", "EDS_PICK", "eds", "R4 EDS pick"),
        _place("R4", "EDS_PLACE", "eds", "wb2", "R4 EDS install"),
        _pick("R5", "PLC_PICK", "plc", "R5 PLC pick"),
        _place("R5", "PLC_PLACE", "plc", "wb2", "R5 PLC install"),
        _pick("R5", "DMA_PICK", "dma", "R5 DMA pick"),
        _place("R5", "DMA_PLACE", "dma", "wb2", "R5 DMA install"),
        PipelineOperation("index", "WB2 micro-index", station="wb2_micro"),
        _pick("R6", "CONTACTOR_PICK", "contactor", "R6 contactor pick"),
        _place(
            "R6", "CONTACTOR_PLACE", "contactor", "wb2_micro",
            "R6 contactor install",
        ),
        _pick("R6", "BREAKER_PICK", "breaker", "R6 breaker pick"),
        _place(
            "R6", "BREAKER_PLACE", "breaker", "wb2_micro",
            "R6 breaker install",
        ),
    ),
    3: (
        _pick("R8", "COM5_PICK", "com5", "R8 COM5 pick"),
        _place("R8", "COM5_PLACE", "com5", "staging", "R8 COM5 install"),
        _pick("R8", "FILTER_PICK", "filter", "R8 filter pick"),
        _place(
            "R8", "FILTER_PLACE", "filter", "staging",
            "R8 filter install",
        ),
        *tuple(
            PipelineOperation(
                "screw", f"R7 screw {index}/4", "R7", f"SCREW_{index}"
            )
            for index in range(1, 5)
        ),
    ),
}


def _after(*operation_ids: str) -> frozenset[str]:
    return frozenset(operation_ids)


# Dependencies express process truth, not artificial synchronized waves.
# Source picks have no public-workspace reservation, so the next robot may
# prefetch while the current robot installs.  Place and screw paths retain
# the single-entry workspace interlock enforced by the scheduler.
MODULE_DEPENDENCIES: dict[int, dict[str, frozenset[str]]] = {
    1: {
        "R1:SHELL_PICK": _after(),
        "R1:WB1_PLACE": _after("R1:SHELL_PICK"),
        "R2:RAIL_PICK_H": _after("R1:SHELL_PICK"),
        "R2:RAIL_PLACE_H": _after("R1:WB1_PLACE", "R2:RAIL_PICK_H"),
        # R2 place and R3 pick look independent at the task level, but their
        # measured paths intersect (R2 Link3 versus R3 Link2).  Keep this
        # physical interlock while retaining R2 prefetch during R1 place.
        "R3:RAIL_PICK_A": _after("R2:RAIL_PLACE_H"),
        "R3:RAIL_PLACE_A": _after("R2:RAIL_PLACE_H", "R3:RAIL_PICK_A"),
        "index:wb1_micro": _after("R3:RAIL_PLACE_A"),
        "R3:RAIL_PICK_B": _after("index:wb1_micro"),
        "R3:RAIL_PLACE_B": _after("R3:RAIL_PICK_B"),
    },
    2: {
        "R4:PSU_PICK": _after(),
        "R4:PSU_PLACE": _after("R4:PSU_PICK"),
        "R5:PLC_PICK": _after("R4:PSU_PICK"),
        "R4:SERVO_PICK": _after("R4:PSU_PLACE"),
        "R4:SERVO_PLACE": _after("R4:SERVO_PICK"),
        "R4:EDS_PICK": _after("R4:SERVO_PLACE"),
        "R5:PLC_PLACE": _after("R4:SERVO_PLACE", "R5:PLC_PICK"),
        "R4:EDS_PLACE": _after("R4:EDS_PICK", "R5:PLC_PLACE"),
        "R5:DMA_PICK": _after("R5:PLC_PLACE"),
        "R5:DMA_PLACE": _after("R4:EDS_PLACE", "R5:DMA_PICK"),
        "R6:CONTACTOR_PICK": _after("R4:EDS_PLACE"),
        "index:wb2_micro": _after("R5:DMA_PLACE", "R6:CONTACTOR_PICK"),
        "R6:CONTACTOR_PLACE": _after(
            "index:wb2_micro", "R6:CONTACTOR_PICK"
        ),
        "R6:BREAKER_PICK": _after("R6:CONTACTOR_PLACE"),
        "R6:BREAKER_PLACE": _after("R6:BREAKER_PICK"),
    },
    3: {
        "R8:COM5_PICK": _after(),
        "R8:COM5_PLACE": _after("R8:COM5_PICK"),
        "R8:FILTER_PICK": _after("R8:COM5_PLACE"),
        "R8:FILTER_PLACE": _after("R8:FILTER_PICK"),
        "R7:SCREW_1": _after("R8:FILTER_PLACE"),
        "R7:SCREW_2": _after("R7:SCREW_1"),
        "R7:SCREW_3": _after("R7:SCREW_2"),
        "R7:SCREW_4": _after("R7:SCREW_3"),
    },
}


@dataclass
class PipelineJob:
    number: int
    runtime: AssemblyRuntime
    recipe: str = "standard"
    cabinet_color: str = "standard"
    completed_module: int = 0
    retired: bool = False


def operations_for_job(
    job: PipelineJob, module: int
) -> tuple[PipelineOperation, ...]:
    skipped = REDUCED_RECIPE_SKIPS if job.recipe == "reduced" else frozenset()
    return tuple(
        operation
        for operation in MODULE_OPERATIONS[module]
        if operation_id(operation) not in skipped
    )


def dependencies_for_job(
    job: PipelineJob, module: int
) -> dict[str, frozenset[str]]:
    """Bypass skipped recipe nodes while preserving their prerequisites."""
    enabled = {
        operation_id(operation) for operation in operations_for_job(job, module)
    }

    def expand(key: str, trail: frozenset[str]) -> set[str]:
        if key in enabled:
            return {key}
        if key in trail:
            raise RuntimeError(f"cyclic recipe dependency at {key}")
        result: set[str] = set()
        for dependency in MODULE_DEPENDENCIES[module][key]:
            result.update(expand(dependency, trail | {key}))
        return result

    return {
        key: frozenset(
            dependency
            for source in MODULE_DEPENDENCIES[module][key]
            for dependency in expand(source, frozenset({key}))
        )
        for key in enabled
    }


@dataclass
class ActiveMotion:
    """One independently advancing robot path in a pipeline module."""

    module: int
    job: PipelineJob
    operation: PipelineOperation
    track: Track
    callback: Callable[[], None]
    indices: list[int]
    robot_pair: tuple[int, int]
    contact_robot_pair: tuple[int, int] | None
    strict_part_pair: tuple[int, int] | None
    contact_part_pair: tuple[int, int] | None
    all_pairs: list[tuple[int, int]]
    cursor: int = 0
    callback_fired: bool = False

    @property
    def frame_index(self) -> int:
        return self.indices[self.cursor]

    def active_pairs(self) -> list[tuple[int, int]]:
        frame = self.frame_index
        track = self.track
        outbound_app = 2 * track.tcp_frame - track.app_frame
        in_contact_leg = track.app_frame <= frame <= outbound_app
        pairs = [
            self.contact_robot_pair
            if self.contact_robot_pair is not None and in_contact_leg
            else self.robot_pair
        ]
        if self.strict_part_pair is None or self.contact_part_pair is None:
            return pairs
        if track.carried_mode == "pick":
            pairs.append(
                self.contact_part_pair
                if frame <= outbound_app
                else self.strict_part_pair
            )
        elif track.carried_mode == "place":
            contact_start = (
                track.contact_start_frame
                if track.contact_start_frame is not None
                else track.app_frame
            )
            if frame < contact_start:
                pairs.append(self.strict_part_pair)
            elif frame <= track.tcp_frame:
                pairs.append(self.contact_part_pair)
        return pairs


def pipeline_takts(job_count: int) -> list[dict[int, int]]:
    """Return module -> job assignments for fill, steady state, and drain."""
    if job_count < 1:
        raise ValueError("job_count must be positive")
    takts: list[dict[int, int]] = []
    for takt in range(job_count + 2):
        assignments = {
            module: job
            for module in range(1, 4)
            if 1 <= (job := takt - module + 2) <= job_count
        }
        if assignments:
            takts.append(assignments)
    return takts


def _tree_root(sim, handles: list[int]) -> int:
    copied = set(int(handle) for handle in handles)
    roots = [
        int(handle)
        for handle in handles
        if int(sim.getObjectParent(int(handle))) not in copied
    ]
    if len(roots) != 1:
        raise RuntimeError(f"expected one copied tree root, found {len(roots)}")
    return roots[0]


def clone_tree(sim, source: int, prefix: str) -> int:
    source_tree = list(sim.getObjectsInTree(source, sim.handle_all, 0))
    copies = [int(handle) for handle in sim.copyPasteObjects(source_tree, 0)]
    root = _tree_root(sim, copies)
    for handle in copies:
        alias = str(sim.getObjectAlias(handle, 0))
        sim.setObjectAlias(handle, f"{prefix}_{alias}")
    sim.setObjectAlias(root, prefix)
    return root


def remove_prefixed_trees(sim, scene: Scene, prefixes: tuple[str, ...]) -> None:
    matches = {
        int(handle)
        for handle in sim.getObjectsInTree(scene.cell, sim.handle_all, 0)
        if str(sim.getObjectAlias(int(handle), 0)).startswith(prefixes)
    }
    roots = [
        handle for handle in matches
        if int(sim.getObjectParent(handle)) not in matches
    ]
    for root in roots:
        sim.removeObjects(list(sim.getObjectsInTree(root, sim.handle_all, 0)))


def park_pallet(runtime: AssemblyRuntime, job_number: int) -> None:
    runtime.sim.setObjectPosition(
        runtime.pallet,
        -1,
        [-4.2, -1.2 - 0.55 * job_number, runtime.pallet_z],
    )
    runtime.pallet_station = "buffer"


def park_parts(runtime: AssemblyRuntime, job_number: int) -> None:
    for index, handle in enumerate(runtime.part_handles.values()):
        runtime.sim.setObjectParent(handle, runtime.scene.parts_parent, True)
        runtime.sim.setObjectPosition(
            handle,
            -1,
            [8.0 + job_number, -3.0 + index * 0.35, -1.0],
        )


def stage_infeed_queue(jobs: list[PipelineJob]) -> None:
    """Place all empty cabinets one slot upstream of the R1 pick point."""
    for job in jobs:
        runtime = job.runtime
        initial = runtime.plan["initial_parts"]["shell"]
        matrix = [float(value) for value in initial["matrix"]]
        # J1 also starts one slot upstream so the first cabinet visibly feeds
        # into the cell instead of materializing at the grasp point.
        matrix[3] -= INFEED_SPACING * job.number
        parent = int(runtime.sim.getObject(initial["parent"]))
        shell = runtime.part_handles["shell"]
        runtime.sim.setObjectParent(shell, parent, True)
        runtime.sim.setObjectMatrix(shell, -1, matrix)


def advance_infeed_queue(jobs: list[PipelineJob], entering: PipelineJob) -> None:
    """Advance the entering cabinet to R1 and close the visible upstream queue."""
    pending = [job for job in jobs if job.completed_module == 0]
    if entering not in pending:
        raise RuntimeError(f"J{entering.number} is not waiting on the infeed")
    starts = {
        job.number: [
            float(value)
            for value in job.runtime.sim.getObjectPosition(
                job.runtime.part_handles["shell"], -1
            )
        ]
        for job in pending
    }
    targets = {}
    pick_matrix = entering.runtime.plan["initial_parts"]["shell"]["matrix"]
    pick_position = [
        float(pick_matrix[3]),
        float(pick_matrix[7]),
        float(pick_matrix[11]),
    ]
    for job in pending:
        slot = job.number - entering.number
        targets[job.number] = [
            pick_position[0] - INFEED_SPACING * slot,
            pick_position[1],
            pick_position[2],
        ]
    steps = entering.runtime.scaled_transport_steps(60, minimum=12)
    print(
        f"[infeed] cabinet queue -> J{entering.number} R1 pick position",
        flush=True,
    )
    for index in range(1, steps + 1):
        blend = index / steps
        blend = blend * blend * blend * (blend * (blend * 6.0 - 15.0) + 10.0)
        for job in pending:
            start = starts[job.number]
            end = targets[job.number]
            job.runtime.sim.setObjectPosition(
                job.runtime.part_handles["shell"],
                -1,
                [a + (b - a) * blend for a, b in zip(start, end)],
            )
        if entering.runtime.simulate:
            step_simulation(entering.runtime.scene.client)


def activate_module_parts(job: PipelineJob, module: int) -> None:
    runtime = job.runtime
    enabled_keys = {
        operation.key
        for operation in operations_for_job(job, module)
        if operation.key is not None
    }
    for key in MODULE_PARTS[module]:
        if key not in enabled_keys:
            continue
        if module == 1 and key == "shell":
            # The shell has already arrived via the animated infeed queue.
            continue
        initial = runtime.plan["initial_parts"][key]
        parent = int(runtime.sim.getObject(initial["parent"]))
        handle = runtime.part_handles[key]
        runtime.sim.setObjectParent(handle, parent, True)
        runtime.sim.setObjectMatrix(handle, -1, initial["matrix"])


def prepare_operation(
    job: PipelineJob,
    operation: PipelineOperation,
) -> tuple[Track, Callable[[], None]] | None:
    runtime = job.runtime
    if operation.kind == "pick":
        assert operation.robot and operation.stem and operation.key
        return runtime.pick(operation.robot, operation.stem, operation.key)
    if operation.kind == "place":
        assert (
            operation.robot and operation.stem and operation.key
            and operation.station
        )
        return runtime.place(
            operation.robot, operation.stem, operation.key, operation.station
        )
    if operation.kind == "screw":
        assert operation.robot and operation.stem
        return runtime.track(operation.robot, operation.stem), lambda: None
    if operation.kind == "index":
        return None
    raise ValueError(f"unknown pipeline operation: {operation.kind}")


def start_motion(
    master: AssemblyRuntime,
    module: int,
    job: PipelineJob,
    operation: PipelineOperation,
) -> ActiveMotion:
    prepared = prepare_operation(job, operation)
    if prepared is None:
        raise RuntimeError("index operations are not robot motions")
    track, callback = prepared
    moving_exclusions = (
        [track.carried_part] if track.carried_part is not None else []
    )
    all_pairs: list[tuple[int, int]] = []
    try:
        robot_pair = master.scene.create_collision_pair(
            track.robot,
            track.exclusions,
            moving_exclusions=moving_exclusions,
        )
        all_pairs.append(robot_pair)
        contact_robot_pair: tuple[int, int] | None = None
        if track.contact_moving_exclusions or (
            track.contact_exclusions != track.exclusions
        ):
            contact_robot_pair = master.scene.create_collision_pair(
                track.robot,
                track.contact_exclusions,
                moving_exclusions=(
                    moving_exclusions + track.contact_moving_exclusions
                ),
            )
            all_pairs.append(contact_robot_pair)

        strict_part_pair: tuple[int, int] | None = None
        contact_part_pair: tuple[int, int] | None = None
        if track.carried_part is not None:
            strict_part_pair = (
                master.scene.create_carried_object_collision_pair(
                    track.robot, track.carried_part, []
                )
            )
            all_pairs.append(strict_part_pair)
            contact_part_pair = (
                master.scene.create_carried_object_collision_pair(
                    track.robot,
                    track.carried_part,
                    list(track.carried_contact_exclusions),
                )
            )
            all_pairs.append(contact_part_pair)
        return ActiveMotion(
            module=module,
            job=job,
            operation=operation,
            track=track,
            callback=callback,
            indices=master.visual_frame_indices([track], len(track.frames)),
            robot_pair=robot_pair,
            contact_robot_pair=contact_robot_pair,
            strict_part_pair=strict_part_pair,
            contact_part_pair=contact_part_pair,
            all_pairs=all_pairs,
        )
    except Exception:
        for pair in all_pairs:
            master.scene.destroy_collision_pair(pair)
        raise


def finish_motion(master: AssemblyRuntime, motion: ActiveMotion) -> None:
    for pair in motion.all_pairs:
        master.scene.destroy_collision_pair(pair)


def enter_module(job: PipelineJob, module: int) -> None:
    runtime = job.runtime
    if module != job.completed_module + 1:
        raise RuntimeError(
            f"J{job.number} cannot enter M{module} after M{job.completed_module}"
        )
    if module == 1:
        runtime.sim.setObjectPosition(
            runtime.pallet,
            -1,
            [STATIONS["wb1"][0], STATIONS["wb1"][1], runtime.pallet_z],
        )
        runtime.pallet_station = "wb1"
    else:
        station = "wb2" if module == 2 else "staging"
        runtime.index_pallet(
            station,
            wait_for=(),
            emits=f"J{job.number}_M{module}_READY",
        )
    activate_module_parts(job, module)
    print(
        f"[pipeline] J{job.number} entered M{module}",
        flush=True,
    )


def retire_job(job: PipelineJob) -> None:
    if job.retired:
        return
    runtime = job.runtime
    runtime.index_pallet(
        "output",
        wait_for=(),
        emits=f"J{job.number}_OUTPUT_READY",
    )
    runtime.conveyor_to_bin(job.number)
    assert runtime.assembly is not None
    runtime.sim.setObjectParent(runtime.assembly, runtime.scene.parts_parent, True)
    park_pallet(runtime, job.number)
    job.retired = True
    print(f"[pipeline] J{job.number} finished and queued", flush=True)


def execute_takt(
    master: AssemblyRuntime,
    assignments: dict[int, PipelineJob],
) -> None:
    """Run dependency-ready picks and workspace-safe installs concurrently."""
    pending = {
        module: {
            operation_id(operation): operation
            for operation in operations_for_job(assignments[module], module)
        }
        for module in assignments
    }
    dependencies = {
        module: dependencies_for_job(assignments[module], module)
        for module in assignments
    }
    completed = {module: set() for module in assignments}
    active: dict[tuple[int, str], ActiveMotion] = {}
    launch_number = 0
    try:
        while any(pending.values()) or active:
            launched_or_indexed = False
            # Scan until no additional dependency-ready work fits the current
            # robot/workspace reservations.  This lets a source prefetch start
            # in the same simulation frame as another robot's installation.
            rescan = True
            while rescan:
                rescan = False
                active_robots = {
                    motion.track.robot for motion in active.values()
                }
                occupied_workspaces = {
                    str(motion.track.workspace)
                    for motion in active.values()
                    if motion.track.workspace is not None
                }
                for module in sorted(pending):
                    for key, operation in list(pending[module].items()):
                        prerequisites = dependencies[module][key]
                        if not prerequisites.issubset(completed[module]):
                            continue
                        job = assignments[module]
                        if operation.kind == "index":
                            # Every arm in this module must have returned to
                            # its accepted stow before its carrier moves.
                            if any(
                                motion.module == module
                                for motion in active.values()
                            ):
                                continue
                            assert operation.station
                            print(
                                f"[pipeline event] M{module}/J{job.number}: "
                                f"{operation.label}",
                                flush=True,
                            )
                            job.runtime.index_pallet(
                                operation.station,
                                wait_for=(),
                                emits=(
                                    f"J{job.number}_{operation.station.upper()}"
                                ),
                                interlock_robots=MODULE_ROBOTS[module],
                            )
                            del pending[module][key]
                            completed[module].add(key)
                            launched_or_indexed = True
                            rescan = True
                            break

                        assert operation.robot
                        if operation.robot in active_robots:
                            continue
                        # ``start_motion`` resolves the authoritative action
                        # workspace.  The cheap lookup below avoids allocating
                        # collision collections for a currently occupied one.
                        workspace = action_workspace(
                            operation.robot, str(operation.stem)
                        )
                        if (
                            workspace is not None
                            and workspace in occupied_workspaces
                        ):
                            continue
                        launch_number += 1
                        motion = start_motion(
                            master, module, job, operation
                        )
                        active[(module, key)] = motion
                        del pending[module][key]
                        active_robots.add(motion.track.robot)
                        if motion.track.workspace is not None:
                            occupied_workspaces.add(
                                str(motion.track.workspace)
                            )
                        launched_or_indexed = True
                        rescan = True
                        print(
                            f"[pipeline launch {launch_number}] "
                            f"M{module}/J{job.number}: {operation.label}",
                            flush=True,
                        )
                    if rescan:
                        break

            if not active:
                if any(pending.values()) and not launched_or_indexed:
                    blocked = {
                        module: {
                            key: sorted(
                                dependencies[module][key]
                                - completed[module]
                            )
                            for key in operations
                        }
                        for module, operations in pending.items()
                        if operations
                    }
                    raise RuntimeError(
                        f"pipeline dependency deadlock: {blocked}"
                    )
                continue

            positions: dict[str, list[float]] = {}
            active_pairs: list[tuple[int, int]] = []
            spin: tuple[int, float] | None = None
            for motion in active.values():
                frame = motion.frame_index
                positions[motion.track.robot] = motion.track.frames[frame]
                active_pairs.extend(motion.active_pairs())
                if motion.operation.kind == "screw":
                    spin = (master.screw_spin, frame * 0.45)
            try:
                master.scene.apply_frame(positions, active_pairs, spin)
            except RuntimeError as exc:
                if int(master.sim.getSimulationState()) != int(
                    master.sim.simulation_stopped
                ):
                    master.sim.pauseSimulation()
                context = ", ".join(
                    f"M{motion.module}/J{motion.job.number}/"
                    f"{motion.track.robot}:{motion.frame_index}"
                    for motion in active.values()
                )
                raise RuntimeError(f"{exc} during {context}") from exc

            finished: list[tuple[int, str]] = []
            for active_key, motion in active.items():
                if (
                    not motion.callback_fired
                    and motion.frame_index >= motion.track.tcp_frame
                ):
                    motion.callback()
                    motion.callback_fired = True
                motion.cursor += 1
                if motion.cursor >= len(motion.indices):
                    finished.append(active_key)
            step_simulation(master.scene.client)
            for active_key in finished:
                motion = active.pop(active_key)
                module, key = active_key
                finish_motion(master, motion)
                completed[module].add(key)
                print(
                    f"[pipeline ready] M{module}/J{motion.job.number}: "
                    f"{motion.operation.label} complete; dependencies released",
                    flush=True,
                )
    finally:
        for motion in active.values():
            finish_motion(master, motion)


def create_jobs(
    scene: Scene,
    plan: dict,
    speed: float,
    count: int,
    black_job: int = 0,
    reduced_job: int = 0,
) -> list[PipelineJob]:
    sim = scene.sim

    # Recover original workpieces from either the serial or an earlier
    # pipeline assembly before removing runtime-generated clone trees.
    serial = AssemblyRuntime(scene, plan, speed)
    serial.reset_product()
    serial.clear_finished_products()
    first = AssemblyRuntime(
        scene,
        plan,
        speed,
        assembly_alias="Pipeline_J1_Cabinet",
    )
    first.reset_product()
    remove_prefixed_trees(
        sim,
        scene,
        tuple(f"Pipeline_J{number}_" for number in range(2, 9)),
    )

    jobs = [
        PipelineJob(
            1,
            first,
            recipe="reduced" if reduced_job == 1 else "standard",
            cabinet_color="black" if black_job == 1 else "standard",
        )
    ]
    original_pallet = first.pallet
    original_pallet_parent = int(sim.getObjectParent(original_pallet))
    for number in range(2, count + 1):
        prefix = f"Pipeline_J{number}"
        handles = {
            key: clone_tree(sim, first.part_handles[key], f"{prefix}_{key}")
            for key in PARTS
        }
        pallet = clone_tree(sim, original_pallet, f"{prefix}_Pallet")
        sim.setObjectParent(pallet, original_pallet_parent, True)
        runtime = AssemblyRuntime(
            scene,
            plan,
            speed,
            part_handles=handles,
            pallet=pallet,
            assembly_alias=f"{prefix}_Cabinet",
        )
        park_parts(runtime, number)
        park_pallet(runtime, number)
        jobs.append(
            PipelineJob(
                number,
                runtime,
                recipe="reduced" if reduced_job == number else "standard",
                cabinet_color="black" if black_job == number else "standard",
            )
        )
    for job in jobs:
        if job.cabinet_color != "black":
            continue
        shell = job.runtime.part_handles["shell"]
        for shape in sim.getObjectsInTree(shell, sim.object_shape_type, 0):
            alias = str(sim.getObjectAlias(int(shape), 0)).lower()
            if (
                "mounting_panel" not in alias
                and int(shape) not in scene.proxied_visual_shapes
            ):
                sim.setShapeColor(
                    int(shape),
                    None,
                    sim.colorcomponent_ambient_diffuse,
                    BLACK_SHELL_COLOR,
                )
        print(
            f"[recipe] J{job.number}: black shell, {job.recipe} process",
            flush=True,
        )
    stage_infeed_queue(jobs)
    return jobs


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--scene", type=Path, default=SCENE_FILE)
    parser.add_argument("--plan", type=Path, default=DEFAULT_PIPELINE_PLAN)
    parser.add_argument("--jobs", type=int, choices=(1, 2, 3, 4), default=3)
    parser.add_argument(
        "--black-job", type=int, choices=(0, 1, 2, 3, 4), default=0,
        help="job number rendered with a black cabinet shell; 0 disables",
    )
    parser.add_argument(
        "--reduced-job", type=int, choices=(0, 1, 2, 3, 4), default=0,
        help="job number using the reduced process recipe; 0 disables",
    )
    parser.add_argument("--speed", type=float, default=3.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 0.2 <= args.speed <= 3.0:
        raise RuntimeError("--speed must be between 0.2 and 3.0")
    for option, number in {
        "--black-job": args.black_job,
        "--reduced-job": args.reduced_job,
    }.items():
        if number > args.jobs:
            raise RuntimeError(f"{option}={number} exceeds --jobs={args.jobs}")
    require_open_top_shell()
    scene_path = args.scene.expanduser().resolve()
    plan_path = args.plan.expanduser().resolve()
    plan = json.loads(plan_path.read_text(encoding="utf-8"))
    if int(plan.get("schema_version", 0)) != PLAN_SCHEMA_VERSION:
        raise RuntimeError("pipeline requires a schema-compatible fixed plan")
    if not motion_policy_matches(plan):
        raise RuntimeError("pipeline plan motion-policy fingerprint is stale")
    expected_scene = {key: plan["scene"][key] for key in ("size", "sha256")}
    if expected_scene != fingerprint(scene_path):
        raise RuntimeError("pipeline plan is bound to a different scene")

    client = RemoteAPIClient(args.host, args.port)
    client.timeout = 900.0
    scene = Scene(client)
    live_path = Path(
        scene.sim.getStringParam(scene.sim.stringparam_scene_path_and_name)
    ).resolve()
    if live_path != scene_path:
        raise RuntimeError(f"open scene is {live_path}, expected {scene_path}")
    if int(scene.sim.getSimulationState()) != int(scene.sim.simulation_stopped):
        raise RuntimeError("stop the CoppeliaSim simulation before pipeline replay")

    jobs = create_jobs(
        scene,
        plan,
        args.speed,
        args.jobs,
        black_job=args.black_job,
        reduced_job=args.reduced_job,
    )
    master = jobs[0].runtime
    scene.set_all_home()
    master.move_to_stows(simulate=False)
    scene.install_batch_script()
    started = False
    succeeded = False
    try:
        client.setStepping(True)
        scene.sim.startSimulation()
        started = True
        step_simulation(client)
        for takt_number, raw_assignments in enumerate(
            pipeline_takts(args.jobs), start=1
        ):
            active_numbers = set(raw_assignments.values())
            for job in jobs:
                if (
                    job.completed_module == 3
                    and not job.retired
                    and job.number not in active_numbers
                ):
                    retire_job(job)

            assignments = {
                module: jobs[number - 1]
                for module, number in raw_assignments.items()
            }
            print(
                f"[takt {takt_number}] "
                + " | ".join(
                    f"M{module}=J{job.number}"
                    for module, job in sorted(assignments.items())
                ),
                flush=True,
            )
            if 1 in assignments:
                advance_infeed_queue(jobs, assignments[1])
            for module in sorted(assignments, reverse=True):
                enter_module(assignments[module], module)
            execute_takt(master, assignments)
            for module, job in assignments.items():
                job.completed_module = module

        for job in jobs:
            retire_job(job)
        scene.sim.pauseSimulation()
        client.setStepping(False)
        succeeded = True
    finally:
        if not succeeded and started:
            try:
                if int(scene.sim.getSimulationState()) != int(scene.sim.simulation_stopped):
                    scene.sim.stopSimulation()
            finally:
                client.setStepping(False)
        scene.remove_batch_script()
        scene.remove_planner_script()

    print(
        f"[done] {args.jobs} cabinets completed with three-module pipeline; "
        "simulation paused at finished queue",
        flush=True,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
