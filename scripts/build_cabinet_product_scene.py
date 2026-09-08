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
    3. places them at task-owned source fixtures (R4 power/drive, R5
       control/filter, R6 switch/communication and R8 door supply) plus a
       fully assembled reference cabinet on an out-of-process display riser;
    4. removes the obsolete round tables and detached robot-base discs, then
       marks three shared workspaces along the indexing conveyor;
    5. creates every task-owned APP/TCP pair for the 8-robot assembly flow;
    6. saves the scene.
"""

from __future__ import annotations

import argparse
import hashlib
import itertools
import json
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

import numpy as np
import yaml
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from sim_bridge.cabinet_geometry import flat_patch_height, top_surface_z, vacuum_grasp_point, require_open_top_shell

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
WB1_MICRO_INDEX_X = 0.120
WB2_MICRO_INDEX_X = 0.120
APP_LIFT_Z = 0.120
R1_EDGE_APP_LIFT_Z = 0.090
SHELL_EDGE_GRIP_OFFSET_Y = 0.11349
SHELL_EDGE_GRIP_DEPTH_Z = 0.007
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
VACUUM_COMPRESSION_Z = 0.001
DOOR_APP_LIFT_Z = 0.150
DOOR_VACUUM_LOCAL_X = 0.089
DOOR_VACUUM_LOCAL_Y = -0.011
DOOR_VACUUM_SURFACE_Z = 0.1348
R8_VACUUM_CUP_DIAMETER = 0.050
R8_VACUUM_CUP_THICKNESS = 0.012

# A down-facing quaternion per robot.  R3/R4/R6 rotate the jaw closing
# direction by 90 degrees.  R8's vacuum cup is yaw-symmetric, but retaining
# its established down-facing yaw avoids unnecessary joint-branch changes.
ROBOT_TCP_QUATERNIONS = {
    "R1": (1.0, 0.0, 0.0, 0.0),
    "R2": (1.0, 0.0, 0.0, 0.0),
    "R3": (2**-0.5, 2**-0.5, 0.0, 0.0),
    "R4": (2**-0.5, 2**-0.5, 0.0, 0.0),
    "R5": (1.0, 0.0, 0.0, 0.0),
    "R6": (2**-0.5, 2**-0.5, 0.0, 0.0),
    "R7": (1.0, 0.0, 0.0, 0.0),
    "R8": (2**-0.5, 2**-0.5, 0.0, 0.0),
}
TARGET_QUATERNION_OVERRIDES = {
    # Projected from the previously verified R3-B down-facing branch.  The
    # 12-degree yaw offset clears the west cabinet edge while retaining a
    # vertical tool axis.
}

# ---- floor layout ----------------------------------------------------------
FLOOR_CENTER = (-1.70, -0.40)
FLOOR_SIZE = (8.40, 4.60)  # spans x[-5.90, 2.50] y[-2.70, 1.90]
COLOR_FLOOR_BASE = [0.30, 0.32, 0.35]
COLOR_FLOOR_ZONE = [0.55, 0.57, 0.60]
COLOR_FLOOR_ZONE_FEED = [0.42, 0.47, 0.56]
COLOR_FLOOR_PATH = [0.95, 0.78, 0.20]
COLOR_FLOOR_PAD = [0.24, 0.26, 0.29]
COLOR_FLOOR_BORDER = [0.58, 0.58, 0.58]

PUBLIC_WORKSPACES = (
    ("Public_Workspace_1", (-3.15, 0.25), (1.55, 1.50), ("R1", "R2", "R3"), [0.42, 0.52, 0.62]),
    ("Public_Workspace_2", (-1.20, 0.25), (1.55, 1.50), ("R4", "R5", "R6"), [0.46, 0.56, 0.64]),
    ("Public_Workspace_3", (0.05, 0.25), (1.25, 1.50), ("R6", "R7", "R8"), [0.50, 0.60, 0.66]),
)

FLOOR_ZONES = [
    ((-3.55, 1.35), (2.00, 1.20), COLOR_FLOOR_ZONE_FEED, "Floor_Zone_Conveyor"),
    *[
        (center, size, color, f"Floor_Zone_{alias}")
        for alias, center, size, _members, color in PUBLIC_WORKSPACES
    ],
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
    # R6 and R7 previously sat inside the central conveyor footprint.  They
    # now stand on opposite sides of shared workspace 3, leaving the complete
    # belt/frame envelope unobstructed.
    "R6": (-0.50, -0.08),
    "R7": (0.10, 0.75),
    "R8": (0.35, -0.22),
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
    # Remove the legacy four-cup carrier as well as its TCP marker. Keeping
    # those cups made the nominal narrow magnet collide with the cabinet.
    configure_compact_axial_tool(sim, 'R2', magnetic=True)
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


def configure_r8_vacuum_tool(sim) -> int:
    """Replace R8's parallel fingers with a 50 mm door vacuum cup.

    The cabinet-door STL has a broad planar face but no through-slot or
    opposed vertical flange suitable for the original down-facing jaws.  The
    existing tool TCP is preserved exactly; only its visible/collidable tool
    geometry is replaced.  Local +X of this R8 TCP points back toward the
    flange, so each cylinder is rotated from its native local-Z axis to +X
    and its working face remains on the TCP plane.
    """
    robot = int(sim.getObject("/R8"))
    tool_root = -1
    tool_tcp = -1
    tip = -1
    for handle in _tree(sim, robot):
        alias = str(sim.getObjectAlias(handle, 0))
        if alias == "R8T":
            tool_root = int(handle)
        elif alias == "R8T_tool_tcp":
            tool_tcp = int(handle)
        elif alias in {"R8_gripper_tip", "R8_vacuum_tip"}:
            tip = int(handle)
    if min(tool_root, tool_tcp, tip) < 0:
        raise RuntimeError("R8 tool/TCP/tip not found; cannot install vacuum cup")

    keep = {tool_root, tool_tcp, tip}
    stale = [
        int(handle)
        for handle in _tree(sim, tool_root)
        if int(handle) not in keep
    ]
    if stale:
        sim.removeObjects(stale)
    sim.setObjectAlias(tip, "R8_vacuum_tip")

    quarter_turn_y = [0.0, 2**-0.5, 0.0, 2**-0.5]
    specifications = (
        ("R8T_vacuum_shaft", 0.035, 0.280, 0.230, [0.24, 0.27, 0.31]),
        ("R8T_vacuum_body", 0.070, 0.080, 0.050, [0.16, 0.20, 0.24]),
        (
            "R8T_vacuum_cup",
            R8_VACUUM_CUP_DIAMETER,
            R8_VACUUM_CUP_THICKNESS,
            R8_VACUUM_CUP_THICKNESS / 2.0,
            [0.08, 0.12, 0.16],
        ),
    )
    special = (
        sim.objectspecialproperty_collidable
        | sim.objectspecialproperty_measurable
        | sim.objectspecialproperty_detectable_all
        | sim.objectspecialproperty_renderable
    )
    for alias, diameter, length, local_x, color in specifications:
        shape = int(
            sim.createPrimitiveShape(
                sim.primitiveshape_cylinder,
                [diameter, diameter, length],
                0,
            )
        )
        sim.setObjectAlias(shape, alias)
        sim.setObjectParent(shape, tool_tcp, False)
        sim.setObjectPose(shape, tool_tcp, [local_x, 0.0, 0.0, *quarter_turn_y])
        sim.setShapeColor(
            shape, None, sim.colorcomponent_ambient_diffuse, color
        )
        sim.setObjectInt32Param(shape, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(shape, sim.shapeintparam_respondable, 0)
        sim.setObjectSpecialProperty(shape, special)
    return len(specifications)


def configure_r5_vacuum_tool(sim) -> int:
    """A single 12 mm cup fits the 25--27 mm DMA/filter housings.

    The legacy 150 x 90 mm four-cup plate could not seal on these parts.
    Keep the real TCP and its ancestor frames, replace only tool geometry.
    """
    return configure_compact_axial_tool(sim, 'R5')


def configure_compact_axial_tool(sim, robot_id: str, magnetic: bool=False) -> int:
    """Preserve TCP frames; replace oversized carrier with a narrow tool."""
    robot = int(sim.getObject('/'+robot_id))
    aliases = {sim.getObjectAlias(h,0): int(h) for h in _tree(sim,robot)}
    root, tip = aliases[robot_id+'T'], aliases[robot_id+'_vacuum_tip']
    keep = {root,tip}
    h = tip
    while h != root:
        h = int(sim.getObjectParent(h))
        if h < 0:
            raise RuntimeError(robot_id+' TCP is not a descendant of its tool')
        keep.add(h)
    stale = [h for h in _tree(sim,root) if h not in keep]
    if stale:
        sim.removeObjects(stale)
    axis = np.asarray(sim.getObjectPosition(aliases['Link6_visual'],tip))
    length = float(np.linalg.norm(axis))
    axis /= length
    q = np.array([-axis[1],axis[0],0,1+axis[2]])
    if np.linalg.norm(q) < 1e-8:
        q=np.array([1.,0,0,0])
    q /= np.linalg.norm(q)
    specs = [(robot_id+'T_body',.012 if magnetic else .018,.014,.013),
             (robot_id+'T_shaft',.010,length-.020,(length+.020)/2)]
    if not magnetic:
        specs.append((robot_id+'T_vacuum_cup',.012,.006,.003))
    for alias, diameter, height, offset in specs:
        h=int(sim.createPrimitiveShape(sim.primitiveshape_cylinder,[diameter,diameter,height],0))
        sim.setObjectAlias(h,alias)
        sim.setObjectParent(h,tip,False)
        sim.setObjectPose(h,tip,[*(axis*offset).tolist(),*q.tolist()])
        sim.setObjectInt32Param(h,sim.shapeintparam_static,1)
        sim.setObjectInt32Param(h,sim.shapeintparam_respondable,0)
        sim.setObjectSpecialProperty(h,sim.objectspecialproperty_collidable | sim.objectspecialproperty_measurable | sim.objectspecialproperty_renderable)
        sim.setShapeColor(h,None,sim.colorcomponent_ambient_diffuse,[.12,.17,.21])
    return len(specs)


def remove_legacy_round_tables(sim) -> int:
    """Remove the three 1.9 m legacy damping tables and their rubber pads.

    The indexing pallet is now the sole workpiece support through all three
    public workspaces.  Keeping the old discs under the conveyor visually and
    geometrically contradicts that layout.
    """
    return _remove_children(sim, f"{SCENE_ROOT}/Tables")


def remove_legacy_robot_base_discs(sim) -> int:
    """Remove detached decorative R1_Base..R8_Base shapes.

    The CR5 models already contain their own base_link visual and collision
    geometry.  These independent discs have no children and several retain
    stale pre-layout coordinates, so they are neither supports nor kinematic
    parents.
    """
    root = _get(sim, f"{SCENE_ROOT}/RobotBases")
    if root == -1:
        return 0
    objects = [int(handle) for handle in _tree(sim, root)]
    if objects:
        sim.removeObjects(objects)
    return len(objects)


def rebuild_areas(sim, areas_parent: int) -> None:
    """Create three logical shared workspaces and the reference display.

    The former WB1/WB2/handoff/staging stands blocked a straight cabinet
    flow.  Their support function is now provided by the indexing pallet.
    """
    for alias, center, _size, members, _color in PUBLIC_WORKSPACES:
        workspace = _group(sim, alias, areas_parent, (*center, 0.0))
        sim.setObjectInt32Param(workspace, sim.objintparam_visibility_layer, 0)
        for robot in members:
            member = _group(sim, f"{alias}_Member_{robot}", workspace)
            sim.setObjectInt32Param(member, sim.objintparam_visibility_layer, 0)

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
    stale_prefixes = (
        "Central_Indexing_Conveyor_",
        "Indexing_Pallet_",
        "Pallet_Lock_",
    )
    stale = [
        int(handle)
        for handle in _tree(sim, conveyors_parent)
        if handle != conveyors_parent
        and (
            str(sim.getObjectAlias(handle, 0)) in stale_aliases
            or str(sim.getObjectAlias(handle, 0)).startswith(stale_prefixes)
        )
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
WB1_MICRO_CENTER = (WB1_CENTER[0] + WB1_MICRO_INDEX_X, WB1_CENTER[1])
WB2_CENTER = (-1.20, 0.25)
WB2_MICRO_CENTER = (WB2_CENTER[0] + WB2_MICRO_INDEX_X, WB2_CENTER[1])
HANDOFF_CENTER = (-2.45, 0.40)
R4_HANDOFF_CENTER = (-2.05, 0.45)
STAGING_CENTER = (0.05, 0.25)          # S78 staging table
REF_CENTER = (1.65, 1.25)              # compact reference display outside public workspaces
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
R4_BASKET_CENTER = (-1.58, 1.12)
R5_BASKET_CENTER = (-1.20, -0.65)
R6_BASKET_CENTER = (-0.50, -0.62)
DOOR_SUPPLY_CENTER = (0.70, -0.50)

# Desired physical footprint centres.  The processed CAD origins are highly
# offset, so build_product_scene converts these centres back to object origins
# before placement.  This keeps each TCP over the visible part, not its STL
# origin.
DEVICE_SOURCE_CENTERS = {
    "psu": (-1.69, 1.12),
    "servo": (-1.58, 1.12),
    "eds": (-1.47, 1.12),
    "plc": (-1.32, -0.65),
    "dma": (-1.18, -0.65),
    "filter": (0.70, -0.50),
    "contactor": (-0.59, -0.62),
    "breaker": (-0.49, -0.62),
    "com5": (-0.39, -0.62),
}
SOURCE_PART_ALIASES = {
    "psu": "PSU_1", "servo": "Servo_1", "eds": "EDS_1",
    "plc": "PLC_1", "dma": "DMA_1", "filter": "Filter_1",
    "contactor": "Contactor_1", "breaker": "Breaker_1",
    "com5": "COM5_1", "mounting_panel": "Mounting_Panel_1",
}
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
    "R6": (-0.30, -0.35, 0.70),
    "R7": (0.10, 0.55, 0.70),
    "R8": (0.35, -0.45, 0.70),
}


def _load_manifest() -> dict:
    return json.loads((PROCESSED_DIR / "manifest.json").read_text(encoding="utf-8"))


def _world_bbox(
    sim, handles: list[int], info_lo: list, info_hi: list
) -> tuple[np.ndarray, np.ndarray]:
    """World-space bbox from each imported shape's actual local BB.

    CoppeliaSim may choose a non-identity reference frame while importing an
    STL.  Manifest corners are expressed in CAD coordinates and therefore
    cannot be projected directly through the post-import object matrix.
    """
    world = []
    for handle in handles:
        # Shape bounding boxes have their own pose in CoppeliaSim 4.10.
        # Mesh vertices, unlike BB extrema, are in the shape frame.
        corners = np.asarray(sim.getShapeMesh(handle)[0], dtype=float).reshape(-1, 3)
        matrix = np.array(sim.getObjectMatrix(handle, -1), dtype=float).reshape(3, 4)
        world.append(corners @ matrix[:3, :3].T + matrix[:3, 3])
    stacked = np.vstack(world)
    return stacked.min(axis=0), stacked.max(axis=0)


def _axis_alignment_rotation(
    local_size: np.ndarray, manifest_size: np.ndarray
) -> np.ndarray:
    """Map an importer's local BB axes back to manifest CAD axes.

    CoppeliaSim can choose a different axis-aligned shape frame per STL.  The
    mapping is therefore inferred per part from its three side lengths.  A
    proper rotation is returned; when two signs are geometrically equivalent,
    the positive permutation is preferred and the last row is flipped only
    when required to keep determinant +1.
    """
    best: tuple[float, tuple[int, int, int]] | None = None
    scale = max(float(np.max(manifest_size)), 1e-9)
    for permutation in itertools.permutations(range(3)):
        error = float(
            np.max(
                np.abs(local_size[list(permutation)] - manifest_size)
            )
            / scale
        )
        if best is None or error < best[0]:
            best = (error, permutation)
    assert best is not None
    if best[0] > 0.03:
        raise RuntimeError(
            "imported STL bounding-box dimensions do not match manifest: "
            f"local={local_size.tolist()}, manifest={manifest_size.tolist()}"
        )
    rotation = np.zeros((3, 3), dtype=float)
    for world_axis, local_axis in enumerate(best[1]):
        rotation[world_axis, local_axis] = 1.0
    if np.linalg.det(rotation) < 0.0:
        rotation[2, :] *= -1.0
    return rotation


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
        # Preserve imported CAD geometry and explicitly make the shape frame
        # the CAD frame. Never infer a mesh rotation from oriented BB sizes.
        sim.relocateShapeFrame(handle, [0, 0, 0, 0, 0, 0, 1])
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

    CoppeliaSim may choose a different axis-aligned local bounding-box frame
    for each imported STL.  The frame is mapped back to the manifest CAD axes
    before placement.  With ``snap_z_min`` the oriented mesh's world z-min
    lands on ``world_position[2]`` (belt/stand/rack/basket floors).
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
    root_frame_correction = np.eye(3)
    root_local_center = (lo + hi) / 2.0
    for handle in handles:
        vertices = np.asarray(sim.getShapeMesh(handle)[0]).reshape(-1, 3)
        if max(np.max(np.abs(vertices.min(0) - lo)),
               np.max(np.abs(vertices.max(0) - hi))) > 0.0002:
            raise RuntimeError(f'{sim.getObjectAlias(handle, 0)} mesh is not in the canonical CAD frame')
        translation = expected_center - rotation @ ((lo + hi) / 2.0)
        matrix = np.column_stack((rotation, translation))
        sim.setObjectMatrix(handle, -1, matrix.reshape(-1).tolist())
    check_lo, check_hi = _world_bbox(sim, handles, info["bbox_lo"], info["bbox_hi"])
    residual = np.max(np.abs((check_lo + check_hi) / 2.0 - expected_center))
    extent_error = np.max(np.abs((check_hi-check_lo) - np.ptp(expected, axis=0)))
    if max(residual, extent_error) > 0.0002:
        raise RuntimeError(f'mesh placement mismatch: centre={residual}, extent={extent_error}')
    if root_frame_correction is None or root_local_center is None:
        raise RuntimeError("imported part has no root shape")
    add_part_collision_proxies(
        sim,
        handles,
        info,
        root_frame_correction,
        root_local_center,
    )


def add_part_collision_proxies(
    sim,
    handles: list[int],
    info: dict,
    frame_correction: np.ndarray,
    root_local_center: np.ndarray,
) -> None:
    """Attach conservative primitive collision geometry to a visual STL.

    Detailed cabinet meshes are retained for rendering.  Collision queries
    use these proxies: a full bounding box for rails/devices and four open
    perimeter walls for the shell, so top-down tools can enter the cabinet.
    """
    root = int(handles[0])
    alias = str(sim.getObjectAlias(root, 0))
    if 'shell' in alias.lower() or 'mounting_panel' in alias.lower():
        # Keep the real rim, openings and mounting faces in collision checks.
        return
    lo = np.array(info["bbox_lo"], dtype=float)
    hi = np.array(info["bbox_hi"], dtype=float)
    center = (lo + hi) / 2.0
    size = hi - lo
    specifications: list[tuple[np.ndarray, np.ndarray]]
    if "shell" in alias.lower():
        wall = min(0.014, float(size[0]) * 0.08, float(size[1]) * 0.08)
        specifications = [
            (np.array([wall, size[1], size[2]]),
             np.array([lo[0] + wall / 2.0, center[1], center[2]])),
            (np.array([wall, size[1], size[2]]),
             np.array([hi[0] - wall / 2.0, center[1], center[2]])),
            (np.array([max(size[0] - 2 * wall, wall), wall, size[2]]),
             np.array([center[0], lo[1] + wall / 2.0, center[2]])),
            (np.array([max(size[0] - 2 * wall, wall), wall, size[2]]),
             np.array([center[0], hi[1] - wall / 2.0, center[2]])),
        ]
    else:
        specifications = [(size, center)]
    special = (
        sim.objectspecialproperty_collidable
        | sim.objectspecialproperty_measurable
        | sim.objectspecialproperty_detectable_all
    )
    for index, (proxy_size, local_center) in enumerate(specifications, start=1):
        proxy = int(
            sim.createPrimitiveShape(
                sim.primitiveshape_cuboid,
                [float(value) for value in proxy_size],
                0,
            )
        )
        sim.setObjectAlias(proxy, f"COL_{alias}_{index}", {"aliasIndex": 0})
        sim.setObjectParent(proxy, root, False)
        proxy_local_position = (
            root_local_center
            + frame_correction.T @ (local_center - center)
        )
        proxy_local_rotation = frame_correction.T
        proxy_local_matrix = np.hstack(
            (proxy_local_rotation, np.zeros((3, 1), dtype=float))
        )
        sim.setObjectPosition(
            proxy, root, [float(value) for value in proxy_local_position]
        )
        sim.setObjectQuaternion(
            proxy,
            root,
            sim.getQuaternionFromMatrix(proxy_local_matrix.reshape(-1).tolist()),
        )
        sim.setObjectInt32Param(proxy, sim.shapeintparam_static, 1)
        sim.setObjectInt32Param(proxy, sim.shapeintparam_respondable, 0)
        sim.setObjectInt32Param(proxy, sim.objintparam_visibility_layer, 0)
        sim.setObjectSpecialProperty(proxy, special)


def remove_placeholder_products(sim) -> dict:
    removed = {
        "targets": _remove_children(sim, TARGETS_PATH),
        "parts": _remove_children(sim, PARTS_PATH),
        "baskets": _remove_children(sim, BASKETS_PATH),
        "areas": _remove_children(sim, AREAS_PATH),
        "ground": _remove_children(sim, f"{SCENE_ROOT}/Ground_Group"),
    }
    # These scripts are created only for a live planning or validation call.
    # If an audit is interrupted, they can otherwise be saved into the .ttt
    # and initialized again on the next scene launch.
    transient_helpers = [
        int(handle)
        for handle in _tree(sim, int(sim.handle_scene))
        if str(sim.getObjectAlias(handle, 0))
        in {
            "Motion_Collision_Planner",
            "Assembly_Collision_Planner",
            "Assembly_Runtime_Batch",
        }
    ]
    if transient_helpers:
        sim.removeObjects(transient_helpers)
    removed["transient_runtime_helpers"] = len(transient_helpers)
    # An interrupted preflight can leave picked parts parented below a robot
    # tool, outside /FiveCR5A_Cell/Parts.  Remove those stale process
    # instances before importing the fresh set, otherwise aliases and TCP
    # attachments become ambiguous on the next build.
    process_aliases = {
        "Shell_1", "Rail_H1", "Rail_1", "Rail_2", "PLC_1", "PSU_1",
        "Servo_1", "DMA_1", "Contactor_1", "Breaker_1",
        "Assembly_In_Process",
    }
    stale_process_roots = [
        int(handle)
        for handle in _tree(sim, int(sim.handle_scene))
        if str(sim.getObjectAlias(handle, 0)) in process_aliases
    ]
    stale_process = list(dict.fromkeys(
        int(descendant)
        for root in stale_process_roots
        for descendant in _tree(sim, root)
    ))
    if stale_process:
        sim.removeObjects(stale_process)
    removed["stale_tool_parts"] = len(stale_process)
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
        old_output_trees = list(dict.fromkeys(
            int(descendant)
            for root in old_output
            for descendant in _tree(sim, root)
        ))
        sim.removeObjects(old_output_trees)

    report["robots_reparented"] = ensure_robots_at_root(sim)
    report["robots_positioned"] = position_robot_bases(sim)
    report["legacy_robot_bases_removed"] = remove_legacy_robot_base_discs(sim)
    report["legacy_round_tables_removed"] = remove_legacy_round_tables(sim)
    report["r2_magnetic_pad"] = configure_r2_magnetic_pad(sim)
    report["r3_slim_rail_fingers"] = configure_r3_slim_rail_fingers(sim)
    report["r8_vacuum_tool_shapes"] = configure_compact_axial_tool(sim, 'R8')
    report['r5_vacuum_tool_shapes'] = configure_r5_vacuum_tool(sim)
    report["floor_objects"] = build_floor(sim)
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
        # The internal mounting panel is part of the supplied shell
        # subassembly, not a door installed over finished components.
        panel_info = manifest['parts']['mounting_panel']
        panel = import_part(sim, 'mounting_panel', f'Mounting_Panel_{index}', handles[0], panel_info)
        place_part(sim, panel, (x,y,BELT_TOP_Z), panel_info, snap_z_min=False)

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
            (*_source_origin_xy(manifest['parts'][part_id], (-2.35 - 0.20*index, -0.65)), basket_floor),
            manifest["parts"][part_id],
            snap_z_min=True,
        )

    source_fixtures = {
        "R4": make_basket(
            sim, baskets_parent, "R4_Device_Basket", R4_BASKET_CENTER,
            (0.42, 0.22), 0.16, front_wall=False,
        ),
        "R5": make_basket(
            sim, baskets_parent, "R5_Device_Basket", R5_BASKET_CENTER,
            (0.42, 0.24), 0.15, front_wall=False,
        ),
        "R6": make_basket(
            sim, baskets_parent, "R6_Device_Basket", R6_BASKET_CENTER,
            (0.34, 0.18), 0.12, front_wall=False,
        ),
        "R8": make_basket(sim, baskets_parent, "R8_Device_Basket", (0.70,-0.50),
                          (0.18,0.16),0.025,front_wall=False),
    }
    fixture_parts = {
        "R4": ("psu", "servo", "eds"),
        "R5": ("plc", "dma"),
        "R6": ("contactor", "breaker", "com5"),
        "R8": ("filter",),
    }
    for robot, part_ids in fixture_parts.items():
        for part_id in part_ids:
            info = manifest["parts"][part_id]
            origin_x, origin_y = _source_origin_xy(
                info, DEVICE_SOURCE_CENTERS[part_id]
            )
            handles = import_part(
                sim, part_id, SOURCE_PART_ALIASES[part_id],
                source_fixtures[robot], info,
            )
            place_part(
                sim, handles, (origin_x, origin_y, basket_floor), info,
                snap_z_min=True,
            )

    # References are coordinate frames, not a duplicate rendered cabinet.
    ref_root = _group(sim, "Cabinet_Product_REF", parts_parent, (*REF_CENTER, SURFACE_Z))
    for part_id, info in manifest["parts"].items():
        reference = _dummy(sim, (*REF_CENTER, SURFACE_Z), f'REF_{part_id}', ref_root, info['color'])
        sim.setObjectInt32Param(reference, sim.objintparam_visibility_layer, 0)

    hidden_collision_shapes = 0
    for handle in sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0):
        if 'respondable' in sim.getObjectAlias(handle, 0).lower():
            sim.setObjectInt32Param(handle, sim.objintparam_visibility_layer, 0)
            hidden_collision_shapes += 1
    report['duplicate_robot_collision_shapes_hidden'] = hidden_collision_shapes

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


def _source_origin_xy(part: dict, physical_center: tuple[float, float]) -> tuple[float, float]:
    center = _center(part)
    return physical_center[0] - center[0], physical_center[1] - center[1]


def _source_tcp(
    part: dict,
    physical_center: tuple[float, float],
    floor_z: float,
    *,
    vacuum: bool = False,
    local_xy: tuple[float, float] | None = None,
    local_z: float | None = None,
) -> tuple[float, float, float]:
    """TCP on a source part placed with its bbox z-min on ``floor_z``."""
    lo, hi = part["bbox_lo"], part["bbox_hi"]
    center = _center(part)
    grasp_xy = local_xy or center[:2]
    origin_x, origin_y = _source_origin_xy(part, physical_center)
    grasp_z = (
        float(local_z)
        if local_z is not None
        else hi[2] - VACUUM_COMPRESSION_Z
        if vacuum
        else center[2]
    )
    return (
        origin_x + grasp_xy[0],
        origin_y + grasp_xy[1],
        floor_z - lo[2] + grasp_z,
    )


def _assembly_tcp(
    part: dict,
    station: tuple[float, float],
    *,
    vacuum: bool = False,
    local_xy: tuple[float, float] | None = None,
    local_z: float | None = None,
) -> tuple[float, float, float]:
    """Corresponding TCP when the CAD part is in its assembly frame."""
    center = _center(part)
    grasp_xy = local_xy or center[:2]
    grasp_z = (
        float(local_z)
        if local_z is not None
        else part["bbox_hi"][2] - VACUUM_COMPRESSION_Z
        if vacuum
        else center[2]
    )
    return (
        station[0] + grasp_xy[0],
        station[1] + grasp_xy[1],
        SURFACE_Z + grasp_z,
    )


def build_paired_targets(manifest: dict) -> dict:
    """Build one vertical APP/TCP pair for every assigned task action."""
    require_open_top_shell()
    shell = manifest["parts"]["shell"]
    rail_h = manifest["parts"]["rail_h1"]
    shell_height = shell["bbox_hi"][2] - shell["bbox_lo"][2]
    shell_edge_grip_z = BELT_TOP_Z + shell_height - SHELL_EDGE_GRIP_DEPTH_Z
    basket_floor = SURFACE_Z + 0.012
    rail_h_height = rail_h["bbox_hi"][2] - rail_h["bbox_lo"][2]

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
            (-3.35145 + WB1_MICRO_INDEX_X, 0.080, 0.29725),
            0.400 - 0.29725,
        ),
        "R7_SCREW_1": ((STAGING_CENTER[0] - 0.1615, STAGING_CENTER[1] + 0.115,
                        SURFACE_Z + shell["bbox_hi"][2]), SCREW_LIFT_Z),
        "R7_SCREW_2": ((STAGING_CENTER[0] - 0.1615, STAGING_CENTER[1] - 0.115,
                        SURFACE_Z + shell["bbox_hi"][2]), SCREW_LIFT_Z),
        "R7_SCREW_3": ((STAGING_CENTER[0] + 0.1615, STAGING_CENTER[1] + 0.115,
                        SURFACE_Z + shell["bbox_hi"][2]), SCREW_LIFT_Z),
        "R7_SCREW_4": ((STAGING_CENTER[0] + 0.1615, STAGING_CENTER[1] - 0.115,
                        SURFACE_Z + shell["bbox_hi"][2]), SCREW_LIFT_Z),
    }

    # Derive both endpoints from the SAME material grasp point. The former
    # hand-tuned placement coordinates belonged to the back-up CAD frame.
    rail_local = (R2_RAIL_GRIP_OFFSET_X, _center(rail_h)[1])
    rail_grip_z = flat_patch_height('rail_h1', rail_local, .007) - R2_MAGNET_COMPRESSION
    targets['R2_RAIL_PICK_H'] = (
        (PLATE_STAND_CENTER[0] + rail_local[0],
         PLATE_STAND_CENTER[1] - 0.105 + rail_local[1],
         SURFACE_Z - rail_h['bbox_lo'][2] + rail_grip_z), 0.19)
    targets['R2_RAIL_PLACE_H'] = (
        _assembly_tcp(rail_h, WB1_CENTER, vacuum=True, local_xy=rail_local, local_z=rail_grip_z),
        0.10,
    )
    for index, (letter, part_id) in enumerate((('A','rail_v1'), ('B','rail_v2'))):
        info = manifest['parts'][part_id]
        grip_z = info['bbox_hi'][2] - 0.010
        source_xy = (-2.35 - 0.20*index, -0.65)
        targets[f'R3_RAIL_PICK_{letter}'] = (
            _source_tcp(info, source_xy, basket_floor, local_z=grip_z), 0.12)
        targets[f'R3_RAIL_PLACE_{letter}'] = (
            _assembly_tcp(info, WB1_CENTER if index == 0 else WB1_MICRO_CENTER, local_z=grip_z), 0.09)

    assignments = {
        "R4": (("psu", False), ("servo", False), ("eds", False)),
        "R5": (("plc", True), ("dma", True)),
        "R6": (("contactor", False), ("breaker", False), ("com5", False)),
        "R8": (("filter", True),),
    }
    for robot, parts in assignments.items():
        for part_id, vacuum in parts:
            info = manifest["parts"][part_id]
            grasp = vacuum_grasp_point(part_id) if vacuum else None
            stem = part_id.upper()
            station = (
                STAGING_CENTER
                if part_id == "com5" or robot == 'R8'
                else WB2_MICRO_CENTER
                if robot == "R6"
                else WB2_CENTER
            )
            targets[f"{robot}_{stem}_PICK"] = (
                _source_tcp(
                    info, DEVICE_SOURCE_CENTERS[part_id], basket_floor,
                    vacuum=vacuum,
                    local_xy=grasp[:2] if vacuum else None,
                    local_z=grasp[2]-VACUUM_COMPRESSION_Z if vacuum else None,
                ),
                APP_LIFT_Z,
            )
            targets[f"{robot}_{stem}_PLACE"] = (
                _assembly_tcp(info, station, vacuum=vacuum,
                    local_xy=grasp[:2] if vacuum else None,
                    local_z=grasp[2]-VACUUM_COMPRESSION_Z if vacuum else None),
                APP_LIFT_Z,
            )

    return targets


def create_targets(sim, targets_parent: int, paired_targets: dict) -> int:
    count = 0
    for robot_id in range(1, 9):
        name = f"R{robot_id}"
        group = _group(sim, f"{name}_Targets", targets_parent)
        color = ROBOT_TARGET_COLORS[name]
        home = _dummy(
            sim, tuple(HOME_REF_POSITIONS[name]), f"{name}_HOME_REF", group, color
        )
        sim.setObjectQuaternion(home, -1, list(ROBOT_TCP_QUATERNIONS[name]))
        count += 1
        for target_name, (position, lift) in paired_targets.items():
            if not target_name.startswith(f"{name}_"):
                continue
            tcp = _dummy(
                sim, tuple(position), f"{target_name}_TCP", group, color
            )
            app = _dummy(
                sim, (position[0], position[1], position[2] + lift),
                f"{target_name}_APP", group, color,
            )
            quaternion = list(_target_quaternion(target_name))
            sim.setObjectQuaternion(tcp, -1, quaternion)
            sim.setObjectQuaternion(app, -1, quaternion)
            count += 2
    return count


def _target_area(name: str) -> str:
    if name.startswith("R1_SHELL_PICK"):
        return "cabinet_conveyor_area"
    if name.startswith("R2_RAIL_PICK"):
        return "r2_stand_area"
    if name.startswith("R3_RAIL_PICK"):
        return "r3_rack_area"
    if name.startswith("R4_") and name.endswith("_PICK"):
        return "r4_device_supply_area"
    if name.startswith("R5_") and name.endswith("_PICK"):
        return "r5_device_supply_area"
    if name.startswith("R6_") and name.endswith("_PICK"):
        return "r6_device_supply_area"
    if name.startswith("R8_DOOR_PICK"):
        return "door_supply_area"
    if name.startswith(("R1_", "R2_", "R3_")):
        return "public_workspace_1"
    if name.startswith(("R4_", "R5_")) or (
        name.startswith("R6_") and "COM5" not in name
    ):
        return "public_workspace_2"
    return "public_workspace_3"


def _target_quaternion(name: str) -> tuple[float, float, float, float]:
    robot = name.split("_", 1)[0]
    values = TARGET_QUATERNION_OVERRIDES.get(
        name, ROBOT_TCP_QUATERNIONS[robot]
    )
    norm = sum(float(value) ** 2 for value in values) ** 0.5
    return tuple(float(value) / norm for value in values)


def sync_points_config(paired_targets: dict) -> None:
    """Rebuild the canonical point registry from generated task targets."""
    path = REPO_ROOT / "configs" / "points.yaml"
    points: dict[str, dict] = {}
    for robot, position in HOME_REF_POSITIONS.items():
        points[f"{robot}_HOME_REF"] = {
            "robot": robot,
            "area": f"{robot.lower()}_home",
            "action": "park",
            "position": [float(v) for v in position],
            "orientation_quaternion": [
                float(v) for v in ROBOT_TCP_QUATERNIONS[robot]
            ],
            "description": f"{robot} down-facing PARK reference",
        }
    for name, (position, lift) in paired_targets.items():
        robot = name.split("_", 1)[0]
        tcp = [float(value) for value in position]
        app = [tcp[0], tcp[1], tcp[2] + float(lift)]
        quaternion = [float(v) for v in _target_quaternion(name)]
        action = name.removeprefix(f"{robot}_").lower()
        points[f"{name}_TCP"] = {
            "robot": robot,
            "area": _target_area(name),
            "action": action,
            "position": tcp,
            "orientation_quaternion": quaternion,
            "description": f"{robot} {action} contact TCP",
        }
        points[f"{name}_APP"] = {
            "robot": robot,
            "area": _target_area(name),
            "action": f"{action}_approach",
            "position": app,
            "orientation_quaternion": quaternion,
            "description": (
                f"{robot} {action} vertical APP; same XY/orientation as TCP"
            ),
        }
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
    contract["counts"]["process_targets"] = len(HOME_REF_POSITIONS) + 2 * len(
        build_paired_targets(_load_manifest())
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
    parser.add_argument('--targets-only', action='store_true', help='Regenerate APP/TCPs without reimporting the product meshes')
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = RemoteAPIClient(args.host, args.port)
    sim = client.require("sim")
    if args.targets_only:
        if sim.getSimulationState() != sim.simulation_stopped:
            raise RuntimeError('stop simulation before updating targets')
        _remove_children(sim, TARGETS_PATH)
        paired = build_paired_targets(_load_manifest())
        create_targets(sim, int(sim.getObject(TARGETS_PATH)), paired)
        sim.saveScene(str(args.output))
        sync_points_config(paired)
        sync_scene_contract(sim, args.output)
        print('updated task targets and scene contract')
        return 0
    report = build_product_scene(sim, args.output)
    for key, value in report.items():
        print(f"{key}: {value}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
