#!/usr/bin/env python3
"""Build the real-cabinet product scene on top of the rebuilt 8-robot line.

Prerequisites: scripts/rename_scene_robots.py and scripts/build_new_line_scene.py
have run (8 robots R1-R8, conveyor, workbenches, handoff, baskets), and
scripts/preprocess_cabinet_models.py has produced models/cabinet/processed/.

This script:
    1. removes the primitive placeholder product content (blue/red hollow
       cabinets, back-panel and truss stacks, old targets/baskets);
    2. imports the 14 processed cabinet parts (27 instances) via
       sim.importShape with a sim.createShape fallback;
    3. places them: two shells on the conveyor, two top plates on the new
       R2 plate stand, three frame rails on the enlarged R3 rack, large
       devices in the R5 basket, small devices in the R6 basket, and a fully
       assembled reference cabinet on the right round table;
    4. adds the S78 staging table and the finished-product conveyor near R8;
    5. creates the 72 process target dummies for the 8-robot assembly flow;
    6. saves the scene.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import numpy as np
import yaml
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from scripts.build_new_line_scene import (
    AREAS_PATH,
    BASKETS_PATH,
    CONVEYORS_PATH,
    PARTS_PATH,
    SCENE_ROOT,
    TARGETS_PATH,
    COLOR_AREA,
    COLOR_BASKET,
    COLOR_HANDOFF,
    COLOR_METAL,
    ROBOT_TARGET_COLORS,
    _cuboid,
    _cylinder,
    _dummy,
    _ensure_group,
    _get,
    _group,
    _remove_children,
    _tree,
    make_conveyor,
)

PROCESSED_DIR = REPO_ROOT / "models" / "cabinet" / "processed"
DEFAULT_OUTPUT = REPO_ROOT / "scenes" / "compact_cell.ttt"

BASE_TABLE_SURFACE_Z = 0.12
SURFACE_Z = 0.27           # 150 mm above the original round-table surface
BELT_TOP_Z = 0.270
INDEX_BELT_TOP_Z = 0.245
PALLET_THICKNESS = 0.025
PALLET_SIZE = (0.58, 0.38)
APP_LIFT_Z = 0.120
R1_EDGE_APP_LIFT_Z = 0.090
SHELL_EDGE_GRIP_OFFSET_Y = 0.1125
SHELL_EDGE_GRIP_DEPTH_Z = 0.020
R2_RAIL_GRIP_OFFSET_X = 0.140
R2_TRANSPORT_TCP_Z = 0.500
R2_MAGNET_DIAMETER = 0.014
R2_MAGNET_THICKNESS = 0.006
R2_MAGNET_COMPRESSION = 0.001
# Calibrated from the real magnetic grasp transform so the rail reaches its
# reference assembly pose without a release-time snap.
R2_RAIL_PLACE_TCP_POSITION = (-3.010146121109595, 0.24994297146556427, 0.3164417337761357)
R3_RAIL_GRIP_OFFSET_Y = -0.270
R3_ASSEMBLY_GRIP_OFFSET_Y = -0.200
R3_ASSEMBLY_GRIP_DEPTH_Z = 0.020
R3_ASSEMBLY_APP_Z = 0.480
R3_HANDOFF_APP_Z = 0.460
R4_ASSEMBLY_GRIP_OFFSET_X = 0.200
R4_ASSEMBLY_GRIP_DEPTH_Z = 0.020
R4_ASSEMBLY_APP_Z = 0.480
SCREW_LIFT_Z = 0.080

# ---- floor layout ----------------------------------------------------------
FLOOR_CENTER = (-1.70, -0.40)
FLOOR_SIZE = (8.40, 4.60)  # spans x[-5.90, 2.50] y[-2.70, 1.90]
COLOR_FLOOR_BASE = [0.30, 0.32, 0.35]
COLOR_FLOOR_ZONE = [0.55, 0.57, 0.60]
COLOR_FLOOR_ZONE_FEED = [0.42, 0.47, 0.56]
COLOR_FLOOR_PATH = [0.95, 0.78, 0.20]
COLOR_FLOOR_PAD = [0.24, 0.26, 0.29]
COLOR_FLOOR_BORDER = [0.58, 0.58, 0.58]

FLOOR_ZONES = [
    ((-3.55, 1.35), (2.00, 1.20), COLOR_FLOOR_ZONE_FEED, "Floor_Zone_Conveyor"),
    ((-3.15, 0.25), (2.30, 2.30), COLOR_FLOOR_ZONE, "Floor_Zone_WB1"),
    ((-2.45, 0.40), (1.00, 1.00), [0.50, 0.52, 0.55], "Floor_Zone_Handoff"),
    ((-1.20, 0.25), (2.30, 2.30), COLOR_FLOOR_ZONE, "Floor_Zone_WB2"),
    ((0.05, 0.25), (1.10, 1.10), [0.50, 0.52, 0.55], "Floor_Zone_Staging"),
    ((0.75, -0.10), (2.30, 2.30), COLOR_FLOOR_ZONE, "Floor_Zone_Display"),
    ((0.62, -0.92), (2.60, 1.30), COLOR_FLOOR_ZONE_FEED, "Floor_Zone_Output"),
]
ROBOT_BASE_POSITIONS = {
    "R1": (-3.60, 0.75),
    "R2": (-3.07, -0.48),
    "R3": (-2.75, -0.35),
    # The northeast offset keeps the complete cabinet south/west of R4's
    # Link2 while preserving paired down-facing reach to handoff and WB2.
    # At (-1.50, 0.55) the cabinet occupied Link2's sweep for 88 frames;
    # the scanned (-1.30, 0.70) branch crosses the two stations in 96 frames.
    "R4": (-1.30, 0.70),
    "R5": (-1.56, -0.22),
    "R6": (-0.60, 0.40),
    "R7": (0.65, 0.25),
    "R8": (0.35, -0.50),
}
FLOW_PATH = [
    (-3.65, 1.25),
    (-3.15, 0.25),
    (-1.20, 0.25),
    (0.05, 0.25),
    (0.75, 0.25),
]


def build_floor(sim) -> int:
    """Rebuild the ground as a styled floor adapted to the new line."""
    import math

    ground_group = _ensure_group(sim, f"{SCENE_ROOT}/Ground_Group", "Ground_Group")
    cx, cy = FLOOR_CENTER
    length, width = FLOOR_SIZE
    created = 0

    _cuboid(sim, (length, width, 0.02), (cx, cy, -0.01), "Ground", ground_group, COLOR_FLOOR_BASE)
    created += 1
    border_h = 0.008
    border_w = 0.08
    _cuboid(sim, (length, border_w, border_h), (cx, cy - width / 2 + border_w / 2, border_h / 2), "Floor_Border_S", ground_group, COLOR_FLOOR_BORDER)
    _cuboid(sim, (length, border_w, border_h), (cx, cy + width / 2 - border_w / 2, border_h / 2), "Floor_Border_N", ground_group, COLOR_FLOOR_BORDER)
    _cuboid(sim, (border_w, width, border_h), (cx - length / 2 + border_w / 2, cy, border_h / 2), "Floor_Border_W", ground_group, COLOR_FLOOR_BORDER)
    _cuboid(sim, (border_w, width, border_h), (cx + length / 2 - border_w / 2, cy, border_h / 2), "Floor_Border_E", ground_group, COLOR_FLOOR_BORDER)
    created += 4

    for (zx, zy), (zl, zw), color, alias in FLOOR_ZONES:
        _cuboid(sim, (zl, zw, 0.006), (zx, zy, 0.003), alias, ground_group, color)
        created += 1

    for robot_id, (px, py) in ROBOT_BASE_POSITIONS.items():
        _cuboid(sim, (0.34, 0.34, 0.006), (px, py, 0.003), f"Floor_Pad_{robot_id}", ground_group, COLOR_FLOOR_PAD)
        created += 1

    for start, end in zip(FLOW_PATH, FLOW_PATH[1:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_seg = math.hypot(dx, dy)
        angle = math.degrees(math.atan2(dy, dx))
        mid = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
        handle = _cuboid(sim, (length_seg - 0.10, 0.08, 0.006), (*mid, 0.005), "Floor_Path_Seg", ground_group, COLOR_FLOOR_PATH)
        sim.setObjectOrientation(handle, -1, [0.0, 0.0, angle])
        created += 1
    # chevron arrows at every second segment midpoint
    for start, end in zip(FLOW_PATH[1:-1], FLOW_PATH[2:]):
        dx, dy = end[0] - start[0], end[1] - start[1]
        angle = math.degrees(math.atan2(dy, dx))
        mid = ((start[0] + end[0]) / 2, (start[1] + end[1]) / 2)
        for side in (-1, 1):
            handle = _cuboid(sim, (0.26, 0.045, 0.006), (*mid, 0.005), "Floor_Path_Arrow", ground_group, COLOR_FLOOR_PATH)
            sim.setObjectOrientation(handle, -1, [0.0, 0.0, angle + side * 38.0])
        created += 2
    return created


def position_robot_bases(sim) -> int:
    """Apply the checked-in cell layout while preserving each base height."""
    positioned = 0
    for robot_id, (x, y) in ROBOT_BASE_POSITIONS.items():
        handle = int(sim.getObject(f"/{robot_id}"))
        position = list(sim.getObjectPosition(handle, -1))
        sim.setObjectPosition(handle, -1, [x, y, position[2]])
        positioned += 1
    return positioned


def configure_r2_magnetic_pad(sim) -> int:
    """Replace R2's visual TCP cube with a real narrow-rail magnetic pad.

    The processed horizontal rail is only 17.5 mm wide, so the four 32 mm
    vacuum cups cannot make a physically meaningful contact.  A 14 mm
    central square magnetic pad fits the rail and spans its open channel.
    Its lower working face is exactly
    coincident with ``R2_vacuum_tip``; process targets may therefore use the
    tip as the actual magnetic contact point.
    """
    robot = int(sim.getObject("/R2"))
    tool_tcp = -1
    stale: list[int] = []
    for handle in _tree(sim, robot):
        alias = str(sim.getObjectAlias(handle, 0))
        if alias == "R2T_tool_tcp":
            tool_tcp = int(handle)
        elif alias in {"R2T_tcp_marker", "R2T_magnetic_pad"}:
            stale.append(int(handle))
    if tool_tcp < 0:
        raise RuntimeError("R2T_tool_tcp not found; cannot install magnetic pad")
    if stale:
        sim.removeObjects(stale)

    pad = int(
        sim.createPrimitiveShape(
            sim.primitiveshape_cuboid,
            [R2_MAGNET_DIAMETER, R2_MAGNET_DIAMETER, R2_MAGNET_THICKNESS],
            0,
        )
    )
    sim.setObjectAlias(pad, "R2T_magnetic_pad")
    sim.setObjectParent(pad, tool_tcp, False)
    # Local +Z points back into the tool.  With the centre at half the pad
    # thickness, the lower magnetic face lies on the TCP plane (local Z=0).
    sim.setObjectPose(
        pad,
        tool_tcp,
        [0.0, 0.0, R2_MAGNET_THICKNESS / 2.0, 0.0, 0.0, 0.0, 1.0],
    )
    sim.setShapeColor(
        pad,
        None,
        sim.colorcomponent_ambient_diffuse,
        [0.10, 0.18, 0.24],
    )
    sim.setObjectInt32Param(pad, sim.shapeintparam_static, 1)
    sim.setObjectInt32Param(pad, sim.shapeintparam_respondable, 0)
    special = (
        sim.objectspecialproperty_collidable
        | sim.objectspecialproperty_measurable
        | sim.objectspecialproperty_detectable_all
        | sim.objectspecialproperty_renderable
    )
    sim.setObjectSpecialProperty(pad, special)
    return pad


def configure_r3_slim_rail_fingers(sim) -> int:
    """Slim R3's bilateral fingers to fit beside the installed frame rail.

    The original generic gripper uses 19--26 mm-wide finger members.  At the
    rail's real edge grasp those members overlap the cabinet side wall even
    though the 17.5 mm rail itself is reachable.  Narrow collidable members
    preserve a true bilateral grasp and keep all gripper geometry in the
    collision model.
    """
    # alias: (cross-rail width, along-rail length, outward local-Z offset)
    desired_sizes = {
        "R3T_left_upper_integrated_finger": (0.008, 0.008, 0.014),
        "R3T_left_lower_integrated_finger": (0.008, 0.008, 0.026),
        "R3T_right_upper_integrated_finger": (0.008, 0.008, 0.014),
        "R3T_right_lower_integrated_finger": (0.008, 0.008, 0.026),
        "R3T_left_front_vertical_jaw": (0.008, 0.008, 0.005),
        "R3T_right_front_vertical_jaw": (0.008, 0.008, 0.005),
        "R3T_left_inner_rubber_pad": (0.006, 0.004, 0.003),
        "R3T_right_inner_rubber_pad": (0.006, 0.004, 0.003),
        "R3T_left_screw_1": (0.004, 0.004, 0.014),
        "R3T_left_screw_2": (0.004, 0.004, 0.026),
        "R3T_right_screw_1": (0.004, 0.004, 0.014),
        "R3T_right_screw_2": (0.004, 0.004, 0.026),
    }
    robot = int(sim.getObject("/R3"))
    by_alias = {
        str(sim.getObjectAlias(handle, 0)): int(handle)
        for handle in _tree(sim, robot)
    }
    updated = 0
    for alias, (desired_y, desired_z, local_z) in desired_sizes.items():
        handle = by_alias[alias]
        size, _ = sim.getShapeBB(handle)
        scale_y = desired_y / float(size[1])
        scale_z = (
            desired_z / float(size[2]) if desired_z is not None else 1.0
        )
        if abs(scale_y - 1.0) > 1e-5 or abs(scale_z - 1.0) > 1e-5:
            sim.scaleObject(handle, 1.0, scale_y, scale_z, 0)
            updated += 1
        if local_z is not None:
            parent = int(sim.getObjectParent(handle))
            position = list(sim.getObjectPosition(handle, parent))
            position[2] = local_z
            sim.setObjectPosition(handle, parent, position)
    return updated


def build_wb1_table(sim) -> None:
    """Idempotently recreate the round table removed with the workspace copy."""
    tables_parent = _ensure_group(sim, f"{SCENE_ROOT}/Tables", "Tables")
    stale = [
        handle
        for handle in _tree(sim, tables_parent)
        if handle != tables_parent
        and str(sim.getObjectAlias(handle, 0)).startswith("WB1_")
    ]
    if stale:
        sim.removeObjects(stale)
    _cylinder(sim, 1.9, 0.12, (*WB1_CENTER, 0.06), "WB1_Table", tables_parent, [0.62, 0.62, 0.62])
    ring = 0.76
    for index, (dx, dy) in enumerate(
        ((ring, 0.0), (ring / 2**0.5, ring / 2**0.5), (0.0, ring),
         (-ring / 2**0.5, ring / 2**0.5), (-ring, 0.0),
         (-ring / 2**0.5, -ring / 2**0.5), (0.0, -ring),
         (ring / 2**0.5, -ring / 2**0.5)),
        start=1,
    ):
        _cuboid(sim, (0.09, 0.09, 0.06), (WB1_CENTER[0] + dx, WB1_CENTER[1] + dy, 0.03), f"WB1_Pad_{index}", tables_parent, [0.30, 0.30, 0.30])


def rebuild_areas(sim, areas_parent: int) -> None:
    """Keep only the reference display riser.

    The former WB1/WB2/handoff/staging stands blocked a straight cabinet
    flow.  Their support function is now provided by the indexing pallet.
    """
    riser_height = SURFACE_Z - BASE_TABLE_SURFACE_Z
    riser_z = BASE_TABLE_SURFACE_Z + riser_height / 2.0
    _cuboid(
        sim,
        (0.42, 0.32, riser_height),
        (*REF_CENTER, riser_z),
        "Display_Elevated_Fixture",
        areas_parent,
        COLOR_METAL,
    )


def build_indexing_conveyor(sim, conveyors_parent: int) -> int:
    """Create a four-stop conveyor and one cabinet locating pallet.

    The belt top is 25 mm below the process plane.  Consequently the pallet
    top is exactly ``SURFACE_Z`` and all existing assembly TCPs retain their
    calibrated heights.
    """
    stale_aliases = {"Central_Indexing_Conveyor", "Indexing_Pallet_1"}
    stale = [
        int(handle)
        for handle in _tree(sim, conveyors_parent)
        if handle != conveyors_parent
        and str(sim.getObjectAlias(handle, 0)) in stale_aliases
    ]
    if stale:
        sim.removeObjects(stale)

    conveyor = make_conveyor(
        sim,
        conveyors_parent,
        prefix="Central_Indexing_Conveyor",
        center=INDEX_CONVEYOR_CENTER,
        length=INDEX_CONVEYOR_LENGTH,
        width=INDEX_CONVEYOR_WIDTH,
        belt_z=INDEX_BELT_TOP_Z - 0.090,
    )
    for label, (x, y) in INDEX_STATIONS:
        for side, dy in (("S", -0.265), ("N", 0.265)):
            _cuboid(
                sim,
                (0.12, 0.025, 0.055),
                (x, y + dy, INDEX_BELT_TOP_Z + 0.0275),
                f"Pallet_Lock_{label}_{side}",
                conveyor,
                [0.92, 0.55, 0.08],
            )

    x, y = WB1_CENTER
    pallet = _group(
        sim,
        "Indexing_Pallet_1",
        conveyors_parent,
        (x, y, INDEX_BELT_TOP_Z),
    )
    _cuboid(
        sim,
        (*PALLET_SIZE, PALLET_THICKNESS),
        (x, y, INDEX_BELT_TOP_Z + PALLET_THICKNESS / 2.0),
        "Indexing_Pallet_Deck",
        pallet,
        [0.18, 0.36, 0.52],
    )
    # Four pins locate the cabinet outside its 452.5 x 240.5 mm footprint.
    for index, (dx, dy) in enumerate(
        ((-0.245, -0.135), (-0.245, 0.135),
         (0.245, -0.135), (0.245, 0.135)),
        start=1,
    ):
        _cylinder(
            sim,
            0.012,
            0.018,
            (x + dx, y + dy, SURFACE_Z + 0.009),
            f"Indexing_Pallet_Locator_{index}",
            pallet,
            [0.96, 0.74, 0.16],
        )
    return pallet


# ---- station layout ------------------------------------------------------
WB1_CENTER = (-3.15, 0.25)
WB2_CENTER = (-1.20, 0.25)
HANDOFF_CENTER = (-2.45, 0.40)
R4_HANDOFF_CENTER = (-2.05, 0.45)
STAGING_CENTER = (0.05, 0.25)          # S78 staging table
REF_CENTER = (0.75, -0.10)             # right round table (reference product)
CONVEYOR_CENTER = (-3.55, 1.35)
SHELL_POSITIONS = [(-3.65, 1.25)]
PLATE_STAND_CENTER = (-3.90, -0.62)
RAIL_RACK_CENTER = (-2.45, -0.65)
# Object origins compensate the large, opposite STL-origin offsets so the
# *physical* rail centres are at x=-2.35 and x=-2.55 in the south-side rack.
R3_RAIL_SOURCE_ORIGINS = (
    (-2.55155, -0.64755),
    (-2.34855, -0.65125),
)
R3_RAIL_PICK_POSITIONS = (
    (-2.35, -0.76625, 0.29725),
    (-2.55, -0.76625, 0.29725),
)
R5_BASKET_CENTER = (-1.20, -0.55)
R6_BASKET_CENTER = (-0.30, 0.75)
FINISHED_CONVEYOR = ((0.62, -0.92), 2.2, 0.42)
INDEX_STATIONS = (
    ("WB1", WB1_CENTER),
    ("WB2", WB2_CENTER),
    ("FASTEN", STAGING_CENTER),
    ("OUTPUT", (0.75, 0.25)),
)
INDEX_CONVEYOR_CENTER = (-1.20, 0.25)
INDEX_CONVEYOR_LENGTH = 4.50
INDEX_CONVEYOR_WIDTH = 0.46

HOME_REF_POSITIONS = {
    "R1": (-3.65, 1.3625, 0.482),
    "R2": (-3.76, -0.725, 0.500),
    "R3": (-2.4948, -0.4486, 0.500),
    "R4": (-1.55, 0.55, 0.70),
    "R5": (-1.55, -0.20, 0.70),
    "R6": (-0.60, 0.35, 0.70),
    "R7": (0.62, 0.25, 0.70),
    "R8": (0.35, -0.45, 0.70),
}


def _load_manifest() -> dict:
    return json.loads((PROCESSED_DIR / "manifest.json").read_text(encoding="utf-8"))


def _world_bbox(
    sim, handles: list[int], info_lo: list, info_hi: list
) -> tuple[np.ndarray, np.ndarray]:
    """World-space bbox of the imported shapes, computed from the manifest
    local bbox corners projected through each handle's world matrix."""
    lo = np.array(info_lo, dtype=float)
    hi = np.array(info_hi, dtype=float)
    corners = np.array(
        [
            [x, y, z]
            for x in (lo[0], hi[0])
            for y in (lo[1], hi[1])
            for z in (lo[2], hi[2])
        ]
    )
    world = []
    for handle in handles:
        matrix = np.array(sim.getObjectMatrix(handle, -1), dtype=float).reshape(3, 4)
        world.append(corners @ matrix[:3, :3].T + matrix[:3, 3])
    stacked = np.vstack(world)
    return stacked.min(axis=0), stacked.max(axis=0)


def import_part(sim, part_id: str, alias: str, parent: int, info: dict) -> list[int]:
    """Import one processed STL and return its shape handles."""
    path = PROCESSED_DIR / f"{part_id}.stl"
    try:
        result = sim.importShape(0, str(path), 0, 0, 1.0)
        if isinstance(result, int):
            handles = [result]
        elif result is None:
            handles = []
        else:
            handles = [int(h) for h in result]
    except Exception as exc:
        raise RuntimeError(f"importShape failed for {part_id}: {exc}")
    if not handles:
        raise RuntimeError(f"importShape returned no handles for {part_id}")
    for index, handle in enumerate(handles):
        sim.setObjectAlias(handle, alias if index == 0 else f"{alias}_{index}", {"aliasIndex": 0})
        if parent != -1:
            sim.setObjectParent(handle, parent, True)
        sim.setObjectInt32Param(handle, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(handle, sim.shapeintparam_respondable, 0)
        sim.setShapeColor(handle, None, sim.colorcomponent_ambient_diffuse, info["color"])
    return handles


RX90 = np.array(
    [[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]]
)  # rotate about X by +90 deg (lay a vertical panel flat)


def place_part(
    sim,
    handles: list[int],
    world_position: tuple,
    info: dict,
    *,
    snap_z_min: bool = False,
    orientation: np.ndarray | None = None,
) -> None:
    """Place an imported part at ``world_position`` with the given world
    orientation (identity = upright assembly pose).

    The mesh is stored in the shape exactly as written by the preprocessor
    (verified: no baked importer rotation), so the object frame is set
    directly.  With ``snap_z_min`` the oriented mesh's world z-min lands on
    ``world_position[2]`` (belt/stand/rack/basket floors).
    """
    rotation = np.eye(3) if orientation is None else np.asarray(orientation, dtype=float)
    lo = np.array(info["bbox_lo"], dtype=float)
    hi = np.array(info["bbox_hi"], dtype=float)
    corners = np.array(
        [
            [x, y, z]
            for x in (lo[0], hi[0])
            for y in (lo[1], hi[1])
            for z in (lo[2], hi[2])
        ]
    )
    expected = corners @ rotation.T + np.array(world_position, dtype=float)
    expected_center = expected.mean(axis=0)
    if snap_z_min:
        expected_center[2] += world_position[2] - expected[:, 2].min()
    for handle in handles:
        matrix = np.array(sim.getObjectMatrix(handle, -1), dtype=float).reshape(3, 4)
        # project the local corners through the TARGET rotation plus the
        # current frame position: the delta must account for the rotation
        # change that is about to be applied.
        projected = corners @ rotation.T + matrix[:3, 3]
        delta = expected_center - projected.mean(axis=0)
        corrected = np.hstack((rotation, (matrix[:3, 3] + delta)[:, None]))
        sim.setObjectMatrix(handle, -1, corrected.reshape(-1).tolist())
    check_lo, check_hi = _world_bbox(sim, handles, info["bbox_lo"], info["bbox_hi"])
    residual = np.max(np.abs((check_lo + check_hi) / 2.0 - expected_center))
    if residual > 0.005:
        print(
            f"  WARNING: {sim.getObjectAlias(handles[0], 0)} bbox residual "
            f"{residual:.4f} m (expected centre {expected_center}, "
            f"got {(check_lo + check_hi) / 2.0})"
        )


def remove_placeholder_products(sim) -> dict:
    removed = {
        "targets": _remove_children(sim, TARGETS_PATH),
        "parts": _remove_children(sim, PARTS_PATH),
        "baskets": _remove_children(sim, BASKETS_PATH),
        "areas": _remove_children(sim, AREAS_PATH),
        "ground": _remove_children(sim, f"{SCENE_ROOT}/Ground_Group"),
    }
    orphans = [
        handle
        for handle in _tree(sim, int(sim.handle_scene))
        if int(sim.getObjectParent(handle)) == -1
        and (
            str(sim.getObjectAlias(handle, 0)).startswith("dummy")
            or str(sim.getObjectAlias(handle, 0)).startswith("extracted")
            or str(sim.getObjectAlias(handle, 0)).startswith("shape")
            or str(sim.getObjectAlias(handle, 0)).startswith("Cart")
            or str(sim.getObjectAlias(handle, 0)) in ("Floor", "box", "shell")
        )
        and not re.match(r"^R\d$", str(sim.getObjectAlias(handle, 0)))
    ]
    if orphans:
        sim.removeObjects(orphans)
    removed["orphan_dummies"] = len(orphans)
    return removed


def ensure_robots_at_root(sim) -> int:
    """Defensive fix: any robot model that is no longer at the scene root
    gets reparented back (CoppeliaSim occasionally misparents objects while
    large batches are removed).  Returns how many robots were moved."""
    moved = 0
    for robot_id in range(1, 9):
        name = f"R{robot_id}"
        handle = -1
        for candidate in _tree(sim, int(sim.handle_scene)):
            if str(sim.getObjectAlias(candidate, 0)) == name:
                handle = candidate
                break
        if handle == -1:
            raise RuntimeError(f"robot {name} not found while re-parenting")
        if int(sim.getObjectParent(handle)) != -1:
            sim.setObjectParent(handle, -1, True)
            moved += 1
    return moved


def make_finished_bin(sim, parent: int, center: tuple) -> int:
    """U-shaped catch bin at the east end of the finished conveyor.

    The west wall is left open so products sliding off the belt end drop
    in; the rim sits at belt level (0.27).
    """
    x, y = center
    inner = (0.70, 0.55)
    wall_t = 0.02
    wall_h = 0.115
    floor_z = 0.15
    outer = (inner[0] + 2 * wall_t, inner[1] + 2 * wall_t)
    root = _group(sim, "Finished_Bin", parent, (x, y, floor_z))
    _cuboid(sim, (outer[0], outer[1], 0.012), (x, y, floor_z + 0.006), "Finished_Bin_Floor", root, [0.62, 0.55, 0.38])
    for alias, (wx, wy) in (
        ("Finished_Bin_Wall_E", (x + outer[0] / 2 - wall_t / 2, y)),
        ("Finished_Bin_Wall_N", (x, y + outer[1] / 2 - wall_t / 2)),
        ("Finished_Bin_Wall_S", (x, y - outer[1] / 2 + wall_t / 2)),
    ):
        size = (wall_t, outer[1], wall_h) if "E" in alias else (outer[0], wall_t, wall_h)
        _cuboid(sim, size, (wx, wy, floor_z + wall_h / 2), alias, root, [0.62, 0.55, 0.38])
    return root


def make_stand(sim, parent: int, alias: str, center: tuple, size: tuple) -> int:
    """Flat platform on legs (top at SURFACE_Z)."""
    x, y = center
    length, width = size
    root = _group(sim, alias, parent, (*center, SURFACE_Z))
    _cuboid(sim, (length, width, 0.010), (x, y, SURFACE_Z), f"{alias}_Top", root, COLOR_HANDOFF)
    leg_w = 0.035
    leg_h = max(SURFACE_Z - 0.005 - 0.02, 0.02)
    leg_z = leg_h / 2
    for index, (dx, dy) in enumerate(
        ((-length / 2 + leg_w / 2, -width / 2 + leg_w / 2),
         (length / 2 - leg_w / 2, -width / 2 + leg_w / 2),
         (-length / 2 + leg_w / 2, width / 2 - leg_w / 2),
         (length / 2 - leg_w / 2, width / 2 - leg_w / 2)),
        start=1,
    ):
        _cuboid(sim, (leg_w, leg_w, leg_h), (x + dx, y + dy, leg_z), f"{alias}_Leg_{index}", root, COLOR_METAL)
    return root


def make_r2_fixture(sim, parent: int, center: tuple) -> int:
    """Rear locator frame that leaves R2's elbow sweep volume open."""
    x, y = center
    root = _group(sim, "R2_Stand", parent, (*center, SURFACE_Z))
    rear_y = y - 0.24
    post_height = SURFACE_Z - 0.02
    post_z = post_height / 2.0
    for side, support_x in (("L", x - 0.23), ("R", x + 0.23)):
        _cuboid(
            sim,
            (0.024, 0.024, post_height),
            (support_x, rear_y, post_z),
            f"R2_Stand_Rear_Post_{side}",
            root,
            COLOR_METAL,
        )
    _cuboid(
        sim,
        (0.50, 0.024, 0.024),
        (x, rear_y, SURFACE_Z),
        "R2_Stand_Rear_Beam",
        root,
        COLOR_METAL,
    )
    return root


def make_r3_open_tray(sim, parent: int, center: tuple) -> int:
    """Low open tray for the offset-origin vertical rails.

    Tall basket sidewalls intersect R3 Link2 during the second pick.  The
    rails only need a horizontal support and rear location, so keep all arm
    approach sides open.
    """
    x, y = center
    root = _group(sim, "R3_Rail_Rack", parent, (*center, SURFACE_Z))
    _cuboid(
        sim, (0.54, 0.36, 0.012), (x, y, SURFACE_Z + 0.006),
        "R3_Rail_Rack_Floor", root, COLOR_BASKET,
    )
    return root


def make_basket(
    sim,
    parent: int,
    alias: str,
    center: tuple,
    inner: tuple,
    wall_h: float,
    *,
    front_wall: bool = True,
) -> int:
    """Open-top bin on legs (top rim at SURFACE_Z + wall_h)."""
    x, y = center
    wall_t = 0.02
    outer = (inner[0] + 2 * wall_t, inner[1] + 2 * wall_t)
    root = _group(sim, alias, parent, (*center, SURFACE_Z))
    _cuboid(sim, (outer[0], outer[1], 0.012), (x, y, SURFACE_Z + 0.006), f"{alias}_Floor", root, COLOR_BASKET)
    _cuboid(sim, (wall_t, outer[1], wall_h), (x - outer[0] / 2 + wall_t / 2, y, SURFACE_Z + wall_h / 2), f"{alias}_Wall_L", root, COLOR_BASKET)
    _cuboid(sim, (wall_t, outer[1], wall_h), (x + outer[0] / 2 - wall_t / 2, y, SURFACE_Z + wall_h / 2), f"{alias}_Wall_R", root, COLOR_BASKET)
    if front_wall:
        _cuboid(sim, (outer[0], wall_t, wall_h), (x, y - outer[1] / 2 + wall_t / 2, SURFACE_Z + wall_h / 2), f"{alias}_Wall_F", root, COLOR_BASKET)
    _cuboid(sim, (outer[0], wall_t, wall_h), (x, y + outer[1] / 2 - wall_t / 2, SURFACE_Z + wall_h / 2), f"{alias}_Wall_B", root, COLOR_BASKET)
    leg_w = 0.03
    leg_h = max(SURFACE_Z - 0.006 - 0.02, 0.02)
    leg_z = leg_h / 2
    for index, (dx, dy) in enumerate(
        ((-outer[0] / 2 + leg_w / 2, -outer[1] / 2 + leg_w / 2),
         (outer[0] / 2 - leg_w / 2, -outer[1] / 2 + leg_w / 2),
         (-outer[0] / 2 + leg_w / 2, outer[1] / 2 - leg_w / 2),
         (outer[0] / 2 - leg_w / 2, outer[1] / 2 - leg_w / 2)),
        start=1,
    ):
        _cuboid(sim, (leg_w, leg_w, leg_h), (x + dx, y + dy, leg_z), f"{alias}_Leg_{index}", root, COLOR_METAL)
    return root


def _stack_z(manifest: dict, part_id: str, surface: float, offset: float = 0.0) -> float:
    """World Z placing a part's z-min on ``surface`` plus an extra offset."""
    lo = manifest["parts"][part_id]["bbox_lo"][2]
    return surface + offset - lo


def build_product_scene(sim, output: Path) -> dict:
    if int(sim.getSimulationState()) != int(sim.simulation_stopped):
        raise RuntimeError("stop the simulation before rebuilding the scene")

    manifest = _load_manifest()
    report: dict = {}
    report["removed"] = remove_placeholder_products(sim)

    parts_parent = _ensure_group(sim, PARTS_PATH, "Parts")
    baskets_parent = _ensure_group(sim, BASKETS_PATH, "Baskets")
    areas_parent = _ensure_group(sim, AREAS_PATH, "Areas")
    conveyors_parent = _ensure_group(sim, CONVEYORS_PATH, "Conveyors")
    targets_parent = _ensure_group(sim, TARGETS_PATH, "Targets")

    source_conveyor = int(
        sim.getObject(f"{CONVEYORS_PATH}/Cabinet_Conveyor")
    )
    source_conveyor_position = list(
        sim.getObjectPosition(source_conveyor, -1)
    )
    sim.setObjectPosition(
        source_conveyor,
        -1,
        [CONVEYOR_CENTER[0], CONVEYOR_CENTER[1], source_conveyor_position[2]],
    )
    old_output = [
        int(handle)
        for handle in _tree(sim, conveyors_parent)
        if str(sim.getObjectAlias(handle, 0)) == "Finished_Conveyor"
    ]
    if old_output:
        sim.removeObjects(old_output)

    report["robots_reparented"] = ensure_robots_at_root(sim)
    report["robots_positioned"] = position_robot_bases(sim)
    report["r2_magnetic_pad"] = configure_r2_magnetic_pad(sim)
    report["r3_slim_rail_fingers"] = configure_r3_slim_rail_fingers(sim)
    report["floor_objects"] = build_floor(sim)
    build_wb1_table(sim)
    rebuild_areas(sim, areas_parent)
    report["indexing_pallet"] = build_indexing_conveyor(
        sim, conveyors_parent
    )

    # ---- source stations -------------------------------------------------
    basket_floor = SURFACE_Z + 0.012  # top of a basket's floor plate
    shell_stack = _group(sim, "Shell_Stack", parts_parent, (*CONVEYOR_CENTER, BELT_TOP_Z))
    for index, (x, y) in enumerate(SHELL_POSITIONS, start=1):
        handles = import_part(sim, "shell", f"Shell_{index}", shell_stack, manifest["parts"]["shell"])
        place_part(sim, handles, (x, y, BELT_TOP_Z), manifest["parts"]["shell"], snap_z_min=True)

    # R2 rear locator rack: only the horizontal rail participates in this
    # process.  Keeping the unused spare door out of this station preserves
    # the collision-free elbow sweep for a vertical flange approach.
    r2_stand = make_r2_fixture(sim, baskets_parent, PLATE_STAND_CENTER)
    rail_h_info = manifest["parts"]["rail_h1"]
    rail_h_handles = import_part(sim, "rail_h1", "Rail_H1", r2_stand, rail_h_info)
    place_part(
        sim,
        rail_h_handles,
        (PLATE_STAND_CENTER[0], PLATE_STAND_CENTER[1] - 0.105, SURFACE_Z),
        rail_h_info,
        snap_z_min=True,
    )

    rail_rack = make_r3_open_tray(sim, baskets_parent, RAIL_RACK_CENTER)
    # Both vertical rails are directly accessible.  Stacking B on top of A
    # made the stated A-then-B process physically impossible.
    rail_info = manifest["parts"]["rail_v1"]
    rail_height = rail_info["bbox_hi"][2] - rail_info["bbox_lo"][2]
    for index, part_id in enumerate(("rail_v1", "rail_v2")):
        handles = import_part(sim, part_id, f"Rail_{index + 1}", rail_rack, manifest["parts"][part_id])
        place_part(
            sim,
            handles,
            (*R3_RAIL_SOURCE_ORIGINS[index], basket_floor),
            manifest["parts"][part_id],
            snap_z_min=True,
        )

    r5_basket = make_basket(sim, baskets_parent, "R5_Device_Basket", R5_BASKET_CENTER, (0.26, 0.18), 0.15)
    for part_id, alias, (x, y) in (
        ("plc", "PLC_1", (R5_BASKET_CENTER[0] - 0.055, R5_BASKET_CENTER[1])),
        ("psu", "PSU_1", (R5_BASKET_CENTER[0] + 0.055, R5_BASKET_CENTER[1])),
    ):
        handles = import_part(sim, part_id, alias, r5_basket, manifest["parts"][part_id])
        place_part(sim, handles, (x, y, basket_floor), manifest["parts"][part_id], snap_z_min=True)

    r6_basket = make_basket(sim, baskets_parent, "R6_Device_Basket", R6_BASKET_CENTER, (0.26, 0.18), 0.15)
    for part_id, alias, (x, y) in (
        ("servo", "Servo_1", (R6_BASKET_CENTER[0] - 0.06, R6_BASKET_CENTER[1])),
        ("dma", "DMA_1", (R6_BASKET_CENTER[0] - 0.02, R6_BASKET_CENTER[1])),
        ("contactor", "Contactor_1", (R6_BASKET_CENTER[0] + 0.01, R6_BASKET_CENTER[1])),
        ("breaker", "Breaker_1", (R6_BASKET_CENTER[0] + 0.05, R6_BASKET_CENTER[1])),
    ):
        handles = import_part(sim, part_id, alias, r6_basket, manifest["parts"][part_id])
        place_part(sim, handles, (x, y, basket_floor), manifest["parts"][part_id], snap_z_min=True)

    # ---- reference finished cabinet on the right round table -------------
    # (without the door: the product is displayed open-front)
    ref_root = _group(sim, "Cabinet_Product_REF", parts_parent, (*REF_CENTER, SURFACE_Z))
    for part_id, info in manifest["parts"].items():
        if part_id == "door":
            continue
        handles = import_part(sim, part_id, f"REF_{part_id}", ref_root, info)
        place_part(sim, handles, (*REF_CENTER, SURFACE_Z), info)

    # ---- process targets --------------------------------------------------
    paired_targets = build_paired_targets(manifest)
    report["targets_created"] = create_targets(
        sim, targets_parent, paired_targets
    )

    sim.saveScene(str(output))
    sync_points_config(paired_targets)
    sync_scene_contract(sim, output)
    report["saved_scene"] = str(output)
    report["synced_configs"] = [
        str(REPO_ROOT / "configs" / "points.yaml"),
        str(REPO_ROOT / "configs" / "scene_contract.yaml"),
    ]
    return report


def _center(part: dict) -> tuple:
    lo, hi = part["bbox_lo"], part["bbox_hi"]
    return tuple((lo[i] + hi[i]) / 2.0 for i in range(3))


def build_paired_targets(manifest: dict) -> dict:
    """Target table derived from the processed-model manifest."""
    shell = manifest["parts"]["shell"]
    rail_h = manifest["parts"]["rail_h1"]
    rail_v1 = manifest["parts"]["rail_v1"]
    rail_v2 = manifest["parts"]["rail_v2"]
    shell_half_z = shell["bbox_hi"][2] / 2.0
    shell_height = shell["bbox_hi"][2] - shell["bbox_lo"][2]
    shell_edge_grip_z = BELT_TOP_Z + shell_height - SHELL_EDGE_GRIP_DEPTH_Z
    basket_floor = SURFACE_Z + 0.012
    rail_h_height = rail_h["bbox_hi"][2] - rail_h["bbox_lo"][2]
    rail_height = rail_v1["bbox_hi"][2] - rail_v1["bbox_lo"][2]
    dev_centers = {
        part_id: _center(manifest["parts"][part_id])
        for part_id in ("plc", "psu", "servo", "dma", "contactor", "breaker")
    }

    def pick_z(part_id: str) -> float:
        part = manifest["parts"][part_id]
        return basket_floor + (dev_centers[part_id][2] - part["bbox_lo"][2])

    targets: dict[str, tuple[tuple, float]] = {
        "R1_SHELL_PICK": (
            (
                SHELL_POSITIONS[0][0],
                SHELL_POSITIONS[0][1] + SHELL_EDGE_GRIP_OFFSET_Y,
                shell_edge_grip_z,
            ),
            R1_EDGE_APP_LIFT_Z,
        ),
        "R1_WB1_PLACE": (
            (
                WB1_CENTER[0],
                WB1_CENTER[1] + SHELL_EDGE_GRIP_OFFSET_Y,
                SURFACE_Z + shell_height - SHELL_EDGE_GRIP_DEPTH_Z,
            ),
            R1_EDGE_APP_LIFT_Z,
        ),
        "R2_RAIL_PICK_H": (
            (
                PLATE_STAND_CENTER[0] + R2_RAIL_GRIP_OFFSET_X,
                PLATE_STAND_CENTER[1] - 0.105,
                SURFACE_Z + rail_h_height - R2_MAGNET_COMPRESSION,
            ),
            R2_TRANSPORT_TCP_Z
            - (SURFACE_Z + rail_h_height - R2_MAGNET_COMPRESSION),
        ),
        "R2_RAIL_PLACE_H": (
            R2_RAIL_PLACE_TCP_POSITION,
            R2_TRANSPORT_TCP_Z - R2_RAIL_PLACE_TCP_POSITION[2],
        ),
        "R3_RAIL_PICK_A": (
            R3_RAIL_PICK_POSITIONS[0],
            0.480 - R3_RAIL_PICK_POSITIONS[0][2],
        ),
        "R3_RAIL_PICK_B": (
            R3_RAIL_PICK_POSITIONS[1],
            0.440 - R3_RAIL_PICK_POSITIONS[1][2],
        ),
        "R3_RAIL_PLACE_A": (
            (-2.94845, 0.120, 0.29725),
            0.480 - 0.29725,
        ),
        "R3_RAIL_PLACE_B": (
            (-3.35145, 0.080, 0.29725),
            0.400 - 0.29725,
        ),
        "R3_WB1_PICK": (
            (
                WB1_CENTER[0],
                WB1_CENTER[1] + R3_ASSEMBLY_GRIP_OFFSET_Y,
                SURFACE_Z + shell_height - R3_ASSEMBLY_GRIP_DEPTH_Z,
            ),
            R3_ASSEMBLY_APP_Z
            - (SURFACE_Z + shell_height - R3_ASSEMBLY_GRIP_DEPTH_Z),
        ),
        "R3_HANDOFF_PLACE": (
            (
                HANDOFF_CENTER[0],
                HANDOFF_CENTER[1] + R3_ASSEMBLY_GRIP_OFFSET_Y,
                SURFACE_Z + shell_height - R3_ASSEMBLY_GRIP_DEPTH_Z,
            ),
            R3_HANDOFF_APP_Z
            - (SURFACE_Z + shell_height - R3_ASSEMBLY_GRIP_DEPTH_Z),
        ),
        "R4_HANDOFF_PICK": (
            (
                R4_HANDOFF_CENTER[0] + R4_ASSEMBLY_GRIP_OFFSET_X,
                R4_HANDOFF_CENTER[1],
                SURFACE_Z + shell_height - R4_ASSEMBLY_GRIP_DEPTH_Z,
            ),
            R4_ASSEMBLY_APP_Z
            - (SURFACE_Z + shell_height - R4_ASSEMBLY_GRIP_DEPTH_Z),
        ),
        "R4_WB2_PLACE": (
            (
                WB2_CENTER[0] + R4_ASSEMBLY_GRIP_OFFSET_X,
                WB2_CENTER[1],
                SURFACE_Z + shell_height - R4_ASSEMBLY_GRIP_DEPTH_Z,
            ),
            R4_ASSEMBLY_APP_Z
            - (SURFACE_Z + shell_height - R4_ASSEMBLY_GRIP_DEPTH_Z),
        ),
        "R5_PLC_PICK": ((R5_BASKET_CENTER[0] - 0.055, R5_BASKET_CENTER[1], pick_z("plc")), APP_LIFT_Z),
        "R5_PLC_PLACE": (
            (WB2_CENTER[0] + dev_centers["plc"][0], WB2_CENTER[1] + dev_centers["plc"][1],
             SURFACE_Z + dev_centers["plc"][2]),
            APP_LIFT_Z,
        ),
        "R5_PSU_PICK": ((R5_BASKET_CENTER[0] + 0.055, R5_BASKET_CENTER[1], pick_z("psu")), APP_LIFT_Z),
        "R5_PSU_PLACE": (
            (WB2_CENTER[0] + dev_centers["psu"][0], WB2_CENTER[1] + dev_centers["psu"][1],
             SURFACE_Z + dev_centers["psu"][2]),
            APP_LIFT_Z,
        ),
        "R6_SERVO_PICK": ((R6_BASKET_CENTER[0] - 0.06, R6_BASKET_CENTER[1], pick_z("servo")), APP_LIFT_Z),
        "R6_SERVO_PLACE": (
            (WB2_CENTER[0] + dev_centers["servo"][0], WB2_CENTER[1] + dev_centers["servo"][1],
             SURFACE_Z + dev_centers["servo"][2]),
            APP_LIFT_Z,
        ),
        "R6_DMA_PICK": ((R6_BASKET_CENTER[0] - 0.02, R6_BASKET_CENTER[1], pick_z("dma")), APP_LIFT_Z),
        "R6_DMA_PLACE": (
            (WB2_CENTER[0] + dev_centers["dma"][0], WB2_CENTER[1] + dev_centers["dma"][1],
             SURFACE_Z + dev_centers["dma"][2]),
            APP_LIFT_Z,
        ),
        "R6_CONTACTOR_PICK": ((R6_BASKET_CENTER[0] + 0.01, R6_BASKET_CENTER[1], pick_z("contactor")), APP_LIFT_Z),
        "R6_CONTACTOR_PLACE": (
            (WB2_CENTER[0] + dev_centers["contactor"][0], WB2_CENTER[1] + dev_centers["contactor"][1],
             SURFACE_Z + dev_centers["contactor"][2]),
            APP_LIFT_Z,
        ),
        "R6_BREAKER_PICK": ((R6_BASKET_CENTER[0] + 0.05, R6_BASKET_CENTER[1], pick_z("breaker")), APP_LIFT_Z),
        "R6_BREAKER_PLACE": (
            (WB2_CENTER[0] + dev_centers["breaker"][0], WB2_CENTER[1] + dev_centers["breaker"][1],
             SURFACE_Z + dev_centers["breaker"][2]),
            APP_LIFT_Z,
        ),
        "R6_WB2_PICK": ((*WB2_CENTER, SURFACE_Z + shell_half_z), APP_LIFT_Z),
        "R6_STAGING_PLACE": ((*STAGING_CENTER, SURFACE_Z + shell_half_z), APP_LIFT_Z),
        "R7_SCREW_1": ((STAGING_CENTER[0] - 0.1615, STAGING_CENTER[1] + 0.115,
                        SURFACE_Z + shell["bbox_hi"][2]), SCREW_LIFT_Z),
        "R7_SCREW_2": ((STAGING_CENTER[0] - 0.1615, STAGING_CENTER[1] - 0.115,
                        SURFACE_Z + shell["bbox_hi"][2]), SCREW_LIFT_Z),
        "R7_SCREW_3": ((STAGING_CENTER[0] + 0.1615, STAGING_CENTER[1] + 0.115,
                        SURFACE_Z + shell["bbox_hi"][2]), SCREW_LIFT_Z),
        "R7_SCREW_4": ((STAGING_CENTER[0] + 0.1615, STAGING_CENTER[1] - 0.115,
                        SURFACE_Z + shell["bbox_hi"][2]), SCREW_LIFT_Z),
        "R8_STAGING_PICK": ((*STAGING_CENTER, SURFACE_Z + shell_half_z), APP_LIFT_Z),
        "R8_OUTPUT_PLACE": (
            (FINISHED_CONVEYOR[0][0] - 0.32, FINISHED_CONVEYOR[0][1],
             BELT_TOP_Z + shell_half_z),
            APP_LIFT_Z,
        ),
    }
    return targets


def create_targets(sim, targets_parent: int, paired_targets: dict) -> int:
    count = 0
    for robot_id in range(1, 9):
        name = f"R{robot_id}"
        group = _group(sim, f"{name}_Targets", targets_parent)
        color = ROBOT_TARGET_COLORS[name]
        _dummy(sim, tuple(HOME_REF_POSITIONS[name]), f"{name}_HOME_REF", group, color)
        count += 1
        for target_name, (position, lift) in paired_targets.items():
            if not target_name.startswith(f"{name}_"):
                continue
            _dummy(sim, tuple(position), f"{target_name}_TCP", group, color)
            _dummy(sim, (position[0], position[1], position[2] + lift), f"{target_name}_APP", group, color)
            count += 2
    return count


def sync_points_config(paired_targets: dict) -> None:
    """Keep checked-in target coordinates identical to generated dummies."""
    path = REPO_ROOT / "configs" / "points.yaml"
    points = yaml.safe_load(path.read_text(encoding="utf-8"))
    for robot, position in HOME_REF_POSITIONS.items():
        points[f"{robot}_HOME_REF"]["position"] = [float(v) for v in position]
    for name, (position, lift) in paired_targets.items():
        tcp = [float(value) for value in position]
        app = [tcp[0], tcp[1], tcp[2] + float(lift)]
        points[f"{name}_TCP"]["position"] = tcp
        points[f"{name}_APP"]["position"] = app
    path.write_text(
        yaml.safe_dump(points, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


def sync_scene_contract(sim, output: Path) -> None:
    """Refresh fingerprint and generated object counts after saving."""
    path = REPO_ROOT / "configs" / "scene_contract.yaml"
    contract = yaml.safe_load(path.read_text(encoding="utf-8"))
    root = int(sim.getObject(SCENE_ROOT))
    targets = int(sim.getObject(TARGETS_PATH))
    contract["scene"]["size"] = output.stat().st_size
    contract["scene"]["sha256"] = hashlib.sha256(output.read_bytes()).hexdigest()
    contract["counts"]["cell_objects"] = len(
        sim.getObjectsInTree(root, sim.handle_all, 0)
    )
    contract["counts"]["target_tree_dummies"] = len(
        sim.getObjectsInTree(targets, sim.object_dummy_type, 0)
    )
    path.write_text(
        yaml.safe_dump(contract, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )


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
    report = build_product_scene(sim, args.output)
    for key, value in report.items():
        print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
