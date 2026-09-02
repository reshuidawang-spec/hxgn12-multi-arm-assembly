#!/usr/bin/env python3
"""Rename the 8 scene robots to the new production-line numbering.

New numbering:
    copy R6/R7/R8   -> R1/R2/R3  (left workspace, former Workspace_Left_Copy)
    original R1/R2/R3 -> R4/R5/R6
    original R4/R5  -> R7/R8

Order of operations (each step keeps every scene alias globally unique,
because embedded scripts and the Python bridge resolve objects by alias
scan as well as by path):

    1. copy robots/targets take temporary names  R6->R9, R7->R10, R8->R11
    2. originals move aside                        R4->R7, R5->R8
    3. originals move aside                        R1->R4, R2->R5, R3->R6
    4. copies take the vacated names               R9->R1, R10->R2, R11->R3
    5. copy robots/bases/targets reparent to the standard paths
       (/R{n}, RobotBases/R{n}_Base, Targets/R{n}_Targets)
    6. the obsolete Sensor_Targets group is removed

The script is idempotent: rerunning it on a finished scene prints a skip
notice.  Nothing is saved until every check passes; a failed run leaves the
scene file untouched.  Run it against a live CoppeliaSim scene (simulation
stopped), e.g.:

    python3 scripts/rename_scene_robots.py
    python3 scripts/rename_scene_robots.py --check   # read-only report
"""

from __future__ import annotations

import argparse
import math
import re
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


SCENE_ROOT = "/FiveCR5A_Cell"
WORKSPACE_PATH = f"{SCENE_ROOT}/Workspace_Left_Copy"
BASES_PATH = f"{SCENE_ROOT}/RobotBases"
TARGETS_PATH = f"{SCENE_ROOT}/Targets"

DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "scenes" / "compact_cell.ttt"

ARM_JOINT_ALIASES = tuple(f"joint{index}" for index in range(1, 7))

# (old, new) pairs applied in this exact order.
COPY_TEMP_RENAMES = [("R6", "R9"), ("R7", "R10"), ("R8", "R11")]
ORIGINAL_RENAMES = [
    ("R4", "R7"),
    ("R5", "R8"),
    ("R1", "R4"),
    ("R2", "R5"),
    ("R3", "R6"),
]
COPY_FINAL_RENAMES = [("R9", "R1"), ("R10", "R2"), ("R11", "R3")]
COPY_BASE_NAMES = {
    "R9": ("WS2_R1_Base", "R1_Base"),
    "R10": ("WS2_R2_Base", "R2_Base"),
    "R11": ("WS2_R3_Base", "R3_Base"),
}

# World X/Y positions used to tell the copy robots from the originals.
ORIGINAL_ROBOT_POSITIONS = {
    "R1": (-1.60, 0.65),
    "R2": (-1.56, -0.22),
    "R3": (-0.60, 0.40),
    "R4": (0.65, 0.25),
    "R5": (0.35, -0.50),
}
COPY_ROBOT_POSITIONS = {
    "R1": (-3.60, 0.75),
    "R2": (-3.51, -0.22),
    "R3": (-2.55, 0.40),
}
POSITION_TOLERANCE = 0.20

EXPECTED_TIPS = {
    "R1": "R1_gripper_tip",
    "R2": "R2_vacuum_tip",
    "R3": "R3_gripper_tip",
    "R4": "R4_gripper_tip",
    "R5": "R5_vacuum_tip",
    "R6": "R6_gripper_tip",
    "R7": "R7_tool_tip",
    "R8": "R8_gripper_tip",
}


def _tree(sim, root: int) -> list[int]:
    return list(sim.getObjectsInTree(root, sim.handle_all, 0))


def _get(sim, path: str) -> int:
    """Path lookup returning -1 when the object does not exist."""
    try:
        return int(sim.getObject(path))
    except Exception:
        return -1


def _aliases(sim, handle: int) -> list[str]:
    aliases = []
    for index in range(2):
        try:
            alias = str(sim.getObjectAlias(handle, index))
        except Exception:
            break
        if alias:
            aliases.append(alias)
    return aliases


def _set_alias(sim, handle: int, alias: str) -> None:
    # aliasIndex 0 also refreshes the secondary "/alias" path mirror.
    sim.setObjectAlias(handle, alias, {"aliasIndex": 0})


def _rename_tree(sim, root: int, old: str, new: str) -> int:
    """Substring-replace ``old`` with ``new`` in every primary alias of the
    tree.  CoppeliaSim keeps a secondary ``/alias`` path mirror; renaming the
    primary with ``aliasIndex: 0`` syncs both slots."""
    changed = 0
    for handle in _tree(sim, root):
        alias = str(sim.getObjectAlias(handle, 0))
        if old in alias:
            _set_alias(sim, handle, alias.replace(old, new))
            changed += 1
    return changed


def _rename_robot_bundle(sim, robot_path: str, old: str, new: str) -> None:
    """Rename a robot root, its base and its target group consistently."""
    robot = _get(sim, robot_path)
    if robot != -1:
        _rename_tree(sim, robot, old, new)
    base = _get(sim, f"{BASES_PATH}/{old}_Base")
    if base != -1:
        _rename_tree(sim, base, old, new)
    targets = _get(sim, f"{TARGETS_PATH}/{old}_Targets")
    if targets != -1:
        _rename_tree(sim, targets, old, new)


def _assert_unique_aliases(sim, scope: str) -> None:
    """Assert every robot-numbered scene alias belongs to at most one object.

    Generic aliases shared inside each imported robot (``joint1``,
    ``world_joint``, ...) are legitimately duplicated and are ignored.
    """
    seen: dict[str, int] = {}
    for handle in _tree(sim, int(sim.handle_scene)):
        for alias in _aliases(sim, handle):
            if not re.search(r"R\d", alias):
                continue
            if alias in seen:
                other = seen[alias]
                raise RuntimeError(
                    f"[{scope}] duplicate alias '{alias}' on "
                    f"{sim.getObjectAlias(handle)} and {sim.getObjectAlias(other)}"
                )
            seen[alias] = handle


def _xy_position(sim, path: str) -> tuple[float, float]:
    handle = _get(sim, path)
    if handle == -1:
        raise RuntimeError(f"missing object: {path}")
    position = sim.getObjectPosition(handle, -1)
    return float(position[0]), float(position[1])


def _near(position: tuple[float, float], expected: tuple[float, float]) -> bool:
    return math.hypot(
        position[0] - expected[0],
        position[1] - expected[1],
    ) < POSITION_TOLERANCE


def detect_state(sim) -> str:
    """Return 'original' | 'intermediate' | 'final' | 'unknown'.

    'intermediate' covers every partially renamed layout; the rename steps
    are object-idempotent and resume from any of them.
    """
    r1 = _get(sim, "/R1")
    if r1 != -1:
        r1_pos = _xy_position(sim, "/R1")
        if _near(r1_pos, COPY_ROBOT_POSITIONS["R1"]) and _get(sim, "/R4") != -1:
            r4_pos = _xy_position(sim, "/R4")
            if _near(r4_pos, ORIGINAL_ROBOT_POSITIONS["R1"]):
                return "final"
            return "intermediate"
        if _near(r1_pos, ORIGINAL_ROBOT_POSITIONS["R1"]):
            return "intermediate"
    for old in ("R6", "R7", "R8", "R9", "R10", "R11"):
        if _get(sim, f"{WORKSPACE_PATH}/{old}") != -1:
            return "intermediate"
    return "unknown"


def verify_final_state(sim) -> dict:
    """Validate the fully renamed scene and return a report."""
    for robot_id in range(1, 9):
        name = f"R{robot_id}"
        root = _get(sim, f"/{name}")
        if root == -1:
            raise RuntimeError(f"missing robot root: /{name}")
        joint_aliases = {
            sim.getObjectAlias(handle)
            for handle in _tree(sim, root)
            if sim.getObjectType(handle) == sim.object_joint_type
        }
        if set(ARM_JOINT_ALIASES) - joint_aliases:
            raise RuntimeError(
                f"/{name} arm joints incomplete: {sorted(joint_aliases)}"
            )
        tip_alias = EXPECTED_TIPS[name]
        tool = _get(sim, f"/{name}/{name}T")
        if tool == -1:
            raise RuntimeError(f"missing tool root: /{name}/{name}T")
        if not any(
            tip_alias in alias
            for handle in _tree(sim, root)
            for alias in _aliases(sim, handle)
        ):
            raise RuntimeError(f"/{name} lacks tip alias {tip_alias}")
    for robot_id in range(1, 9):
        name = f"R{robot_id}"
        if _get(sim, f"{BASES_PATH}/{name}_Base") == -1:
            raise RuntimeError(f"missing base: {BASES_PATH}/{name}_Base")
        if _get(sim, f"{TARGETS_PATH}/{name}_Targets") == -1:
            raise RuntimeError(f"missing target group: {TARGETS_PATH}/{name}_Targets")
    if _get(sim, f"{TARGETS_PATH}/Sensor_Targets") != -1:
        raise RuntimeError("Sensor_Targets still present")
    return {
        "robots": {
            f"R{robot_id}": {
                "position": list(sim.getObjectPosition(_get(sim, f"/R{robot_id}"), -1)),
                "tool": sim.getObjectAlias(_get(sim, f"/R{robot_id}/R{robot_id}T")),
            }
            for robot_id in range(1, 9)
        },
    }


def rename_scene(sim, output: Path) -> dict:
    state = detect_state(sim)
    print(f"rename state: {state}")
    if state == "final":
        print("scene already renamed — skipping")
        return verify_final_state(sim)
    if state == "unknown":
        raise RuntimeError("scene layout not recognised; aborting")

    if int(sim.getSimulationState()) != int(sim.simulation_stopped):
        raise RuntimeError("stop the simulation before renaming")

    # 1. normalise the workspace copies onto their temporary names so the
    # original numbers are free for step 2/3.  This also unwinds any partial
    # rename left by an interrupted earlier run (copies may already carry
    # temporary OR final names).
    for temp, final in COPY_FINAL_RENAMES:
        robot = _get(sim, f"{WORKSPACE_PATH}/{final}")
        if robot != -1 and _get(sim, f"{WORKSPACE_PATH}/{temp}") == -1:
            _rename_tree(sim, robot, final, temp)
        targets = _get(sim, f"{WORKSPACE_PATH}/{final}_Targets")
        if targets != -1 and _get(sim, f"{WORKSPACE_PATH}/{temp}_Targets") == -1:
            _rename_tree(sim, targets, final, temp)
    for old, temp in COPY_TEMP_RENAMES:
        robot = _get(sim, f"{WORKSPACE_PATH}/{old}")
        if robot == -1:
            continue
        _rename_tree(sim, robot, old, temp)
        targets = _get(sim, f"{WORKSPACE_PATH}/{old}_Targets")
        if targets != -1:
            _rename_tree(sim, targets, old, temp)
    # copy bases that took their final names early go back to the WS2_ names
    for temp, (old_base, new_base) in COPY_BASE_NAMES.items():
        base = _get(sim, f"{WORKSPACE_PATH}/{new_base}")
        if base == -1:
            continue
        _rename_tree(sim, base, new_base, old_base)
    _assert_unique_aliases(sim, "normalised copies")

    # 2+3. originals move aside into the vacated numbers.  Every object is
    # checked independently so the script can resume from any failure point.
    for old, new in ORIGINAL_RENAMES:
        for path in (
            f"/{old}",
            f"{BASES_PATH}/{old}_Base",
            f"{TARGETS_PATH}/{old}_Targets",
        ):
            handle = _get(sim, path)
            if handle == -1:
                continue
            _rename_tree(sim, handle, old, new)
        _assert_unique_aliases(sim, f"original {old}->{new}")

    # 4. copies take the final names, bases are renamed, everything is
    # reparented to the standard paths.
    bases_parent = _get(sim, BASES_PATH)
    targets_parent = _get(sim, TARGETS_PATH)
    for old, new in COPY_FINAL_RENAMES:
        robot = _get(sim, f"{WORKSPACE_PATH}/{old}")
        if robot == -1:
            robot = _get(sim, f"{WORKSPACE_PATH}/{new}")
        if robot == -1:
            continue
        _rename_tree(sim, robot, old, new)
        targets = _get(sim, f"{WORKSPACE_PATH}/{old}_Targets")
        if targets == -1:
            targets = _get(sim, f"{WORKSPACE_PATH}/{new}_Targets")
        if targets != -1:
            _rename_tree(sim, targets, old, new)
        old_base, new_base = COPY_BASE_NAMES[old]
        base = _get(sim, f"{WORKSPACE_PATH}/{old_base}")
        if base == -1:
            base = _get(sim, f"{WORKSPACE_PATH}/{new_base}")
        if base != -1:
            _rename_tree(sim, base, old_base, new_base)
        sim.setObjectParent(robot, -1, True)
        if base != -1:
            sim.setObjectParent(base, bases_parent, True)
        if targets != -1:
            sim.setObjectParent(targets, targets_parent, True)
        _assert_unique_aliases(sim, f"copy {old}->{new}")

    # 6. remove the obsolete camera target group
    sensor_targets = _get(sim, f"{TARGETS_PATH}/Sensor_Targets")
    if sensor_targets != -1:
        sim.removeObjects(_tree(sim, sensor_targets))

    _assert_unique_aliases(sim, "final")
    report = verify_final_state(sim)
    sim.saveScene(str(output))
    report["saved_scene"] = str(output)
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=23000, type=int)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--check",
        action="store_true",
        help="read-only: report the rename state without modifying anything",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = RemoteAPIClient(args.host, args.port)
    sim = client.require("sim")
    if args.check:
        print(f"rename state: {detect_state(sim)}")
        return 0
    report = rename_scene(sim, args.output)
    for robot_id, info in sorted(report["robots"].items()):
        position = ", ".join(f"{value:.3f}" for value in info["position"])
        print(f"{robot_id}: {info['tool']} @ ({position})")
    print(f"saved_scene: {report.get('saved_scene', '(not saved)')}")
    print("renamed R6-R8 -> R1-R3, original R1-R3 -> R4-R6, original R4-R5 -> R7-R8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
