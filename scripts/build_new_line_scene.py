#!/usr/bin/env python3
"""Clear the legacy process objects and build the new 8-robot line scene.

Prerequisite: scripts/rename_scene_robots.py has already run, so the scene
has /R1..R8 with the left workspace robots as R1-R3.

This script:
    1. removes every legacy process object (supply parts, old conveyors,
       areas/fixtures, carts, camera, old target groups, the
       Workspace_Left_Copy leftovers) and all embedded child scripts;
    2. builds the new production line content:
       - a cabinet conveyor at R1 carrying blue/red hollow cubes;
       - baskets at R2-R6 (R2: back panels, R3: two trusses, R4-R6 empty);
       - workbench 1/2 area markers and the R3->R4 handoff table;
       - 30 process target dummies (11 APP/TCP pairs + 8 HOME_REF);
    3. saves the scene.

Run against a live CoppeliaSim scene (simulation stopped):

    python3 scripts/build_new_line_scene.py
"""

from __future__ import annotations

import argparse
import re
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


SCENE_ROOT = "/FiveCR5A_Cell"
PARTS_PATH = f"{SCENE_ROOT}/Parts"
CONVEYORS_PATH = f"{SCENE_ROOT}/Conveyors"
AREAS_PATH = f"{SCENE_ROOT}/Areas"
SENSORS_PATH = f"{SCENE_ROOT}/Sensors"
TARGETS_PATH = f"{SCENE_ROOT}/Targets"
BASKETS_PATH = f"{SCENE_ROOT}/Baskets"
WORKSPACE_LEFT_COPY = f"{SCENE_ROOT}/Workspace_Left_Copy"

DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "scenes" / "compact_cell.ttt"

# ---- geometry (metres) -------------------------------------------------
CABINET_L = 0.36
CABINET_W = 0.24
CABINET_H = 0.22
CABINET_T = 0.012
SURFACE_Z = 0.12           # measured round-table top (table centre z 0.06 + 0.06)
BELT_TOP_Z = 0.270         # conveyor belt top (matches legacy belt height)
APP_LIFT_Z = 0.180         # approach dummy lift above each TCP dummy

PANEL_SIZE = (0.232, 0.208, 0.010)
PANEL_COUNT = 6
TRUSS_SIZE = (0.330, 0.020, 0.030)
TRUSS_COUNT = 2

# ---- colours ------------------------------------------------------------
COLOR_CABINET_BLUE = [0.10, 0.42, 0.90]
COLOR_CABINET_RED = [0.85, 0.16, 0.12]
COLOR_METAL = [0.50, 0.50, 0.50]
COLOR_DARK = [0.02, 0.02, 0.02]
COLOR_BLACK = [0.005, 0.005, 0.005]
COLOR_BASKET = [0.62, 0.55, 0.38]
COLOR_PANEL = [0.55, 0.58, 0.62]
COLOR_TRUSS = [0.78, 0.78, 0.62]
COLOR_AREA = [0.86, 0.82, 0.68]
COLOR_HANDOFF = [0.70, 0.70, 0.74]

ROBOT_TARGET_COLORS = {
    "R1": [1.00, 0.25, 0.25],
    "R2": [0.25, 0.45, 1.00],
    "R3": [0.25, 0.80, 0.35],
    "R4": [1.00, 0.55, 0.10],
    "R5": [0.10, 0.80, 0.80],
    "R6": [0.60, 0.30, 0.90],
    "R7": [0.95, 0.85, 0.20],
    "R8": [0.90, 0.30, 0.70],
}

# ---- layout -------------------------------------------------------------
# (alias, x, y) — world coordinates.  Initial estimates; tune the dummies
# in the scene after the build and re-sync configs/points.yaml.
CONVEYOR_CENTER = (-3.55, 1.30)
CONVEYOR_LENGTH = 2.0
CONVEYOR_WIDTH = 0.34
CABINET_BLUE_POS = (-3.65, 1.30)   # pick station
CABINET_RED_POS = (-3.15, 1.30)    # next unit behind the pick station

WORKBENCH1_CENTER = (-3.15, 0.25)  # left copied round table
WORKBENCH2_CENTER = (-1.20, 0.25)  # original left round table
HANDOFF_CENTER = (-2.05, 0.50)     # between R3 and R4

BASKET_POSITIONS = {
    "R2_Panel_Basket": (-3.30, -0.55),
    "R3_Truss_Basket": (-2.15, 0.75),
    "R4_Empty_Basket": (-1.95, 1.05),
    "R5_Empty_Basket": (-1.20, -0.55),
    "R6_Empty_Basket": (-0.30, 0.75),
}

HOME_REF_POSITIONS = {
    "R1": (-3.50, 0.55, 0.70),
    "R2": (-3.46, -0.32, 0.70),
    "R3": (-2.55, 0.35, 0.70),
    "R4": (-1.55, 0.55, 0.70),
    "R5": (-1.55, -0.20, 0.70),
    "R6": (-0.60, 0.35, 0.70),
    "R7": (0.62, 0.25, 0.70),
    "R8": (0.35, -0.45, 0.70),
}

# name -> (x, y, z) of the TCP dummy; APP dummies are +APP_LIFT_Z above.
PAIRED_TARGETS = {
    "R1_CABINET_PICK": (*CABINET_BLUE_POS, BELT_TOP_Z + CABINET_H / 2),
    "R1_WB1_PLACE": (*WORKBENCH1_CENTER, SURFACE_Z + CABINET_H / 2),
    "R2_PANEL_PICK": (
        BASKET_POSITIONS["R2_Panel_Basket"][0],
        BASKET_POSITIONS["R2_Panel_Basket"][1],
        # top panel of the stack (computed by make_panel_stack)
        0.0,
    ),
    "R2_PANEL_PLACE": (
        WORKBENCH1_CENTER[0],
        WORKBENCH1_CENTER[1] - CABINET_W / 2,
        SURFACE_Z + CABINET_H / 2,
    ),
    "R3_TRUSS_PICK": (
        BASKET_POSITIONS["R3_Truss_Basket"][0],
        BASKET_POSITIONS["R3_Truss_Basket"][1],
        0.0,  # top truss of the stack (computed by make_truss_stack)
    ),
    "R3_TRUSS_PLACE_13": (
        *WORKBENCH1_CENTER,
        SURFACE_Z + CABINET_H / 3 + TRUSS_SIZE[2] / 2,
    ),
    "R3_TRUSS_PLACE_23": (
        *WORKBENCH1_CENTER,
        SURFACE_Z + 2 * CABINET_H / 3 + TRUSS_SIZE[2] / 2,
    ),
    "R3_WB1_PICK": (*WORKBENCH1_CENTER, SURFACE_Z + CABINET_H / 2),
    "R3_HANDOFF_PLACE": (*HANDOFF_CENTER, SURFACE_Z + CABINET_H / 2),
    "R4_HANDOFF_PICK": (*HANDOFF_CENTER, SURFACE_Z + CABINET_H / 2),
    "R4_WB2_PLACE": (*WORKBENCH2_CENTER, SURFACE_Z + CABINET_H / 2),
}

# -------------------------------------------------------------------------


def _tree(sim, root: int) -> list[int]:
    return list(sim.getObjectsInTree(root, sim.handle_all, 0))


def _get(sim, path: str) -> int:
    try:
        return int(sim.getObject(path))
    except Exception:
        return -1


def _group(sim, alias: str, parent: int, position=(0.0, 0.0, 0.0)) -> int:
    handle = int(sim.createDummy(0.01))
    sim.setObjectAlias(handle, alias)
    if parent != -1:
        sim.setObjectParent(handle, parent, True)
    sim.setObjectPosition(handle, -1, list(position))
    return handle


def _ensure_group(sim, path: str, alias: str) -> int:
    handle = _get(sim, path)
    if handle == -1:
        parent_path, _, _ = path.rpartition("/")
        handle = _group(sim, alias, _get(sim, parent_path))
    return handle


def _shape(
    sim,
    primitive: int,
    size: list[float],
    position: tuple[float, float, float],
    alias: str,
    parent: int,
    color: list[float],
) -> int:
    handle = int(sim.createPrimitiveShape(primitive, size, 0))
    sim.setObjectAlias(handle, alias)
    if parent != -1:
        sim.setObjectParent(handle, parent, True)
    sim.setObjectPosition(handle, -1, list(position))
    sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, color)
    sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)
    return handle


def _cuboid(
    sim,
    size: tuple[float, float, float],
    position: tuple[float, float, float],
    alias: str,
    parent: int,
    color: list[float],
) -> int:
    return _shape(sim, sim.primitiveshape_cuboid, list(size), position, alias, parent, color)


def _cylinder(
    sim,
    diameter: float,
    height: float,
    position: tuple[float, float, float],
    alias: str,
    parent: int,
    color: list[float],
) -> int:
    return _shape(
        sim,
        sim.primitiveshape_cylinder,
        [diameter, diameter, height],
        position,
        alias,
        parent,
        color,
    )


def _dummy(
    sim,
    position: tuple[float, float, float],
    alias: str,
    parent: int,
    color: list[float],
) -> int:
    handle = int(sim.createDummy(0.02))
    sim.setObjectAlias(handle, alias)
    if parent != -1:
        sim.setObjectParent(handle, parent, True)
    sim.setObjectPosition(handle, -1, list(position))
    try:
        sim.setObjectColor(handle, None, color)
    except Exception:
        pass
    return handle


# -------------------------------------------------------------------------
# legacy cleanup
# -------------------------------------------------------------------------


def _remove_children(sim, path: str) -> int:
    """Remove every descendant of ``path`` (keeping the group itself).

    ``sim.removeObjects`` only removes the listed objects in this
    CoppeliaSim version — children survive as orphans reparented to the
    group's parent — so the full descendant list must be passed explicitly.
    """
    parent = _get(sim, path)
    if parent == -1:
        return 0
    descendants = [
        handle for handle in _tree(sim, parent) if handle != parent
    ]
    if descendants:
        sim.removeObjects(descendants)
    return len(descendants)


def _remove_if_present(sim, path: str) -> bool:
    handle = _get(sim, path)
    if handle == -1:
        return False
    sim.removeObjects(_tree(sim, handle))
    return True


def remove_legacy_objects(sim) -> dict:
    removed: dict[str, bool | int] = {}
    removed["parts"] = _remove_children(sim, PARTS_PATH)
    removed["parts_b"] = int(_remove_if_present(sim, f"{SCENE_ROOT}/PartsB"))
    removed["conveyors"] = _remove_children(sim, CONVEYORS_PATH)
    removed["areas"] = _remove_children(sim, AREAS_PATH)
    removed["sensors"] = _remove_children(sim, SENSORS_PATH)
    removed["targets"] = _remove_children(sim, TARGETS_PATH)
    removed["baskets"] = _remove_children(sim, BASKETS_PATH)
    removed["carts"] = int(
        _remove_if_present(sim, "/CartA")
        or _remove_if_present(sim, f"{SCENE_ROOT}/Carts/CartA")
        or _remove_if_present(sim, "/CartB")
        or _remove_if_present(sim, f"{SCENE_ROOT}/Carts/CartB")
    )
    # NOTE: never delete the bare "/Parts" path — CoppeliaSim resolves it
    # through the group's secondary alias to the REAL /FiveCR5A_Cell/Parts
    # group (every object gets a "/alias" mirror when renamed via ZMQ).
    removed["workspace_left_copy"] = int(_remove_if_present(sim, WORKSPACE_LEFT_COPY))
    # Orphaned auto-named dummies at the scene root (left behind when their
    # parent group was removed without its subtree).
    orphans = [
        handle
        for handle in _tree(sim, int(sim.handle_scene))
        if int(sim.getObjectParent(handle)) == -1
        and re.match(r"^dummy(\[\d+\])?$", str(sim.getObjectAlias(handle, 0)))
    ]
    if orphans:
        sim.removeObjects(orphans)
    removed["orphan_dummies"] = len(orphans)
    return removed


SCRIPT_OBJECT_TYPE = 17  # sim.objecttype_script; the client constant is broken


def remove_child_scripts(sim) -> int:
    """Remove every script object (the legacy embedded child scripts).

    The ZMQ ``sim.getScript(type, index)`` enumeration is unreliable in this
    CoppeliaSim version, so scan the scene for script-typed objects instead.
    The robot model scripts are not stored as script objects and are kept.
    """
    script_objects = [
        handle
        for handle in _tree(sim, int(sim.handle_scene))
        if sim.getObjectType(handle) == SCRIPT_OBJECT_TYPE
    ]
    if script_objects:
        sim.removeObjects(script_objects)
    return len(script_objects)


# -------------------------------------------------------------------------
# new content
# -------------------------------------------------------------------------


def make_conveyor(
    sim,
    parent: int,
    prefix: str = "Cabinet_Conveyor",
    center: tuple = CONVEYOR_CENTER,
    length: float = CONVEYOR_LENGTH,
    width: float = CONVEYOR_WIDTH,
    belt_z: float = 0.18,
) -> int:
    x, y, z = (*center, belt_z)
    root = _group(sim, prefix, parent, (x, y, z))
    frame_h = 0.12
    leg_w = 0.035
    leg_h = max(z - frame_h / 2, 0.02)
    leg_z = leg_h / 2

    _cuboid(sim, (length, width + 0.10, frame_h), (x, y, z), f"{prefix}_Frame", root, COLOR_METAL)
    _cuboid(sim, (length - 0.08, width, 0.030), (x, y, z + 0.075), f"{prefix}_Belt", root, COLOR_BLACK)
    for index, (dx, dy) in enumerate(
        ((-length / 2 + 0.12, -width / 2), (length / 2 - 0.12, -width / 2),
         (-length / 2 + 0.12, width / 2), (length / 2 - 0.12, width / 2)),
        start=1,
    ):
        _cuboid(
            sim,
            (leg_w, leg_w, leg_h),
            (x + dx, y + dy, leg_z),
            f"{prefix}_Leg_{index}",
            root,
            COLOR_METAL,
        )
    return root


def make_cabinet(
    sim,
    alias: str,
    position: tuple[float, float, float],
    parent: int,
    color: list[float],
) -> tuple[int, dict]:
    """Hollow cube: bottom + left/right/front walls + 4 posts; back and top
    open.  Inner ledge blocks on the left/right walls hold the two trusses
    at 1/3 and 2/3 of the cabinet height."""
    x, y = position[0], position[1]
    bottom_z = position[2]
    root = _group(sim, alias, parent, (x, y, bottom_z))
    half_l = CABINET_L / 2
    half_w = CABINET_W / 2

    _cuboid(sim, (CABINET_L, CABINET_W, CABINET_T), (x, y, bottom_z + CABINET_T / 2), f"{alias}_Bottom", root, color)
    _cuboid(sim, (CABINET_T, CABINET_W, CABINET_H), (x - half_l + CABINET_T / 2, y, bottom_z + CABINET_H / 2), f"{alias}_Left_Wall", root, color)
    _cuboid(sim, (CABINET_T, CABINET_W, CABINET_H), (x + half_l - CABINET_T / 2, y, bottom_z + CABINET_H / 2), f"{alias}_Right_Wall", root, color)
    _cuboid(sim, (CABINET_L, CABINET_T, CABINET_H), (x, y + half_w - CABINET_T / 2, bottom_z + CABINET_H / 2), f"{alias}_Front_Wall", root, color)

    for index, (px, py) in enumerate(
        ((-CABINET_L * 0.36, -CABINET_W * 0.32), (CABINET_L * 0.36, -CABINET_W * 0.32),
         (-CABINET_L * 0.36, CABINET_W * 0.32), (CABINET_L * 0.36, CABINET_W * 0.32)),
        start=1,
    ):
        _cylinder(sim, 0.020, 0.050, (x + px, y + py, bottom_z + CABINET_T + 0.025), f"{alias}_Post_{index}", root, COLOR_METAL)

    ledges = {}
    inner_x = half_l - CABINET_T
    for fraction, suffix in ((1 / 3, "13"), (2 / 3, "23")):
        ledge_z = bottom_z + CABINET_H * fraction - 0.006
        for side, sign in (("L", -1), ("R", 1)):
            name = f"{alias}_Ledge_{side}_{suffix}"
            ledges[name] = _cuboid(
                sim,
                (CABINET_T, 0.05, 0.012),
                (x + sign * (inner_x - 0.006), y, ledge_z),
                name,
                root,
                color,
            )
    return root, ledges


def make_basket(
    sim,
    alias: str,
    position: tuple[float, float],
    parent: int,
    inner_size: tuple[float, float] = (0.26, 0.18),
) -> tuple[int, float]:
    """Open-top bin with four legs; returns (root, floor_top_z)."""
    x, y = position
    bottom_z = SURFACE_Z
    wall_t = 0.02
    wall_h = 0.12
    outer_x = inner_size[0] + 2 * wall_t
    outer_y = inner_size[1] + 2 * wall_t
    root = _group(sim, alias, parent, (x, y, bottom_z))

    _cuboid(sim, (outer_x, outer_y, 0.012), (x, y, bottom_z + 0.006), f"{alias}_Floor", root, COLOR_BASKET)
    _cuboid(sim, (wall_t, outer_y, wall_h), (x - outer_x / 2 + wall_t / 2, y, bottom_z + wall_h / 2), f"{alias}_Wall_L", root, COLOR_BASKET)
    _cuboid(sim, (wall_t, outer_y, wall_h), (x + outer_x / 2 - wall_t / 2, y, bottom_z + wall_h / 2), f"{alias}_Wall_R", root, COLOR_BASKET)
    _cuboid(sim, (outer_x, wall_t, wall_h), (x, y - outer_y / 2 + wall_t / 2, bottom_z + wall_h / 2), f"{alias}_Wall_F", root, COLOR_BASKET)
    _cuboid(sim, (outer_x, wall_t, wall_h), (x, y + outer_y / 2 - wall_t / 2, bottom_z + wall_h / 2), f"{alias}_Wall_B", root, COLOR_BASKET)

    leg_w = 0.03
    leg_h = max(bottom_z - 0.006 - 0.02, 0.02)
    leg_z = leg_h / 2
    for index, (dx, dy) in enumerate(
        ((-outer_x / 2 + leg_w / 2, -outer_y / 2 + leg_w / 2),
         (outer_x / 2 - leg_w / 2, -outer_y / 2 + leg_w / 2),
         (-outer_x / 2 + leg_w / 2, outer_y / 2 - leg_w / 2),
         (outer_x / 2 - leg_w / 2, outer_y / 2 - leg_w / 2)),
        start=1,
    ):
        _cuboid(sim, (leg_w, leg_w, leg_h), (x + dx, y + dy, leg_z), f"{alias}_Leg_{index}", root, COLOR_METAL)

    return root, bottom_z + 0.012


def make_panel_stack(sim, parent: int, position: tuple[float, float], floor_top: float) -> float:
    """Back panels for R2; returns the top panel centre z."""
    root = _group(sim, "Back_Panel_Stack", parent, (*position, floor_top))
    top_z = floor_top + (PANEL_COUNT - 1) * PANEL_SIZE[2] + PANEL_SIZE[2] / 2
    for index in range(PANEL_COUNT):
        _cuboid(
            sim,
            PANEL_SIZE,
            (position[0], position[1], floor_top + index * PANEL_SIZE[2] + PANEL_SIZE[2] / 2),
            f"Back_Panel_{index + 1}",
            root,
            COLOR_PANEL,
        )
    return top_z


def make_truss_stack(sim, parent: int, position: tuple[float, float], floor_top: float) -> float:
    """Two trusses for R3; returns the top truss centre z."""
    root = _group(sim, "Truss_Stack", parent, (*position, floor_top))
    top_z = floor_top + (TRUSS_COUNT - 1) * TRUSS_SIZE[2] + TRUSS_SIZE[2] / 2
    for index in range(TRUSS_COUNT):
        _cuboid(
            sim,
            TRUSS_SIZE,
            (position[0], position[1], floor_top + index * TRUSS_SIZE[2] + TRUSS_SIZE[2] / 2),
            f"Truss_{index + 1}",
            root,
            COLOR_TRUSS,
        )
    return top_z


def make_workbench_marker(sim, parent: int, alias: str, center: tuple[float, float]) -> None:
    _cuboid(sim, (0.40, 0.30, 0.010), (*center, SURFACE_Z), alias, parent, COLOR_AREA)


def make_handoff_table(sim, parent: int, alias: str, center: tuple[float, float]) -> None:
    x, y = center
    root = _group(sim, alias, parent, (*center, SURFACE_Z))
    _cuboid(sim, (0.35, 0.28, 0.010), (x, y, SURFACE_Z), f"{alias}_Top", root, COLOR_HANDOFF)
    leg_w = 0.035
    leg_h = max(SURFACE_Z - 0.005 - 0.02, 0.02)
    leg_z = leg_h / 2
    for index, (dx, dy) in enumerate(
        ((-0.35 / 2 + leg_w / 2, -0.28 / 2 + leg_w / 2),
         (0.35 / 2 - leg_w / 2, -0.28 / 2 + leg_w / 2),
         (-0.35 / 2 + leg_w / 2, 0.28 / 2 - leg_w / 2),
         (0.35 / 2 - leg_w / 2, 0.28 / 2 - leg_w / 2)),
        start=1,
    ):
        _cuboid(sim, (leg_w, leg_w, leg_h), (x + dx, y + dy, leg_z), f"{alias}_Leg_{index}", root, COLOR_METAL)


def create_targets(sim) -> int:
    """Create fresh R1..R8 target groups with the new process dummies."""
    targets_parent = _ensure_group(sim, TARGETS_PATH, "Targets")
    count = 0
    for robot_id in range(1, 9):
        name = f"R{robot_id}"
        group = _group(sim, f"{name}_Targets", targets_parent)
        color = ROBOT_TARGET_COLORS[name]
        home = HOME_REF_POSITIONS[name]
        _dummy(sim, tuple(home), f"{name}_HOME_REF", group, color)
        count += 1
        for target_name, (x, y, z) in PAIRED_TARGETS.items():
            if not target_name.startswith(f"{name}_"):
                continue
            _dummy(sim, (x, y, z), f"{target_name}_TCP", group, color)
            _dummy(sim, (x, y, z + APP_LIFT_Z), f"{target_name}_APP", group, color)
            count += 2
    return count


# -------------------------------------------------------------------------


def build_new_line_scene(sim, output: Path) -> dict:
    if int(sim.getSimulationState()) != int(sim.simulation_stopped):
        raise RuntimeError("stop the simulation before rebuilding the scene")

    report: dict = {}
    report["legacy_removed"] = remove_legacy_objects(sim)
    report["child_scripts_removed"] = remove_child_scripts(sim)

    conveyors_parent = _ensure_group(sim, CONVEYORS_PATH, "Conveyors")
    parts_parent = _ensure_group(sim, PARTS_PATH, "Parts")
    areas_parent = _ensure_group(sim, AREAS_PATH, "Areas")
    baskets_parent = _ensure_group(sim, BASKETS_PATH, "Baskets")

    make_conveyor(sim, conveyors_parent)
    make_cabinet(sim, "Cabinet_Blue_Body", (*CABINET_BLUE_POS, BELT_TOP_Z), parts_parent, COLOR_CABINET_BLUE)
    make_cabinet(sim, "Cabinet_Red_Body", (*CABINET_RED_POS, BELT_TOP_Z), parts_parent, COLOR_CABINET_RED)

    # baskets: R2 with panels, R3 with trusses, R4-R6 empty
    for alias, (x, y) in BASKET_POSITIONS.items():
        inner = (0.36, 0.20) if alias == "R3_Truss_Basket" else (0.26, 0.18)
        basket, floor_top = make_basket(sim, alias, (x, y), baskets_parent, inner)
        if alias == "R2_Panel_Basket":
            top_z = make_panel_stack(sim, basket, (x, y), floor_top)
            PAIRED_TARGETS["R2_PANEL_PICK"] = (*PAIRED_TARGETS["R2_PANEL_PICK"][:2], top_z)
        elif alias == "R3_Truss_Basket":
            top_z = make_truss_stack(sim, basket, (x, y), floor_top)
            PAIRED_TARGETS["R3_TRUSS_PICK"] = (*PAIRED_TARGETS["R3_TRUSS_PICK"][:2], top_z)

    make_workbench_marker(sim, areas_parent, "Workbench1_Area", WORKBENCH1_CENTER)
    make_workbench_marker(sim, areas_parent, "Workbench2_Area", WORKBENCH2_CENTER)
    make_handoff_table(sim, areas_parent, "Handoff_Area", HANDOFF_CENTER)

    report["targets_created"] = create_targets(sim)
    report["child_scripts_remaining"] = sum(
        1
        for handle in _tree(sim, int(sim.handle_scene))
        if sim.getObjectType(handle) == SCRIPT_OBJECT_TYPE
    )

    sim.saveScene(str(output))
    report["saved_scene"] = str(output)
    return report


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=23000, type=int)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = RemoteAPIClient(args.host, args.port)
    sim = client.require("sim")
    report = build_new_line_scene(sim, args.output)
    for key, value in report.items():
        print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
