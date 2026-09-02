#!/usr/bin/env python3
"""Duplicate the complete R1/R2/R3 round-table workspace in CoppeliaSim.

The original five-arm cell is left untouched.  The copied robots are renamed
R6/R7/R8 and collected below ``/FiveCR5A_Cell/Workspace_Left_Copy`` so that
the existing R1-R5 controller continues to resolve exactly the same objects.
The copied workspace is intentionally display-only until dedicated trajectories
and collision checks are added for R6-R8.
"""

from __future__ import annotations

import argparse
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient


WORKSPACE_PATH = "/FiveCR5A_Cell/Workspace_Left_Copy"
DEFAULT_X_OFFSET = -1.95

TABLE_OBJECTS = [
    "/FiveCR5A_Cell/Tables/Damping_Table_Left",
    *[
        f"/FiveCR5A_Cell/Tables/Left_RubberPad_{index}"
        for index in range(1, 9)
    ],
]
BASE_OBJECTS = [
    f"/FiveCR5A_Cell/RobotBases/R{index}_Base" for index in range(1, 4)
]
AREA_OBJECTS = [
    "/FiveCR5A_Cell/Areas/Box_Supply_Area",
    "/FiveCR5A_Cell/Areas/Terminal_Supply_Area",
    "/FiveCR5A_Cell/Areas/PCB_Supply_Area",
    "/FiveCR5A_Cell/Areas/Module_Supply_Area",
    "/FiveCR5A_Cell/Areas/Assembly_Area",
    "/FiveCR5A_Cell/Areas/Assembly_Fixture",
]
PART_OBJECTS = [
    "/FiveCR5A_Cell/Parts/Box_Blank",
    "/FiveCR5A_Cell/Parts/PCB_Supply",
    "/FiveCR5A_Cell/Parts/Terminal_Block_Supply",
    "/FiveCR5A_Cell/Parts/Control_Module_Supply",
    "/FiveCR5A_Cell/PartsB/Box_Blank_B",
    "/FiveCR5A_Cell/PartsB/PCB_Supply_B",
    "/FiveCR5A_Cell/PartsB/Terminal_Block_Supply_B",
    "/FiveCR5A_Cell/PartsB/Control_Module_Supply_B",
]
ROBOT_OBJECTS = ["/R1", "/R2", "/R3"]
TARGET_OBJECTS = [
    f"/FiveCR5A_Cell/Targets/R{index}_Targets" for index in range(1, 4)
]


def _tree(sim, root: int) -> list[int]:
    return list(sim.getObjectsInTree(root, sim.handle_all, 0))


def _remove_tree(sim, root: int) -> None:
    sim.removeObjects(_tree(sim, root))


def _copy_tree(sim, source: int) -> int:
    """Copy a model or a regular hierarchy and return its copied root."""
    if sim.getModelProperty(source) != sim.modelproperty_not_model:
        return int(sim.copyPasteObjects([source], 1)[0])

    source_tree = _tree(sim, source)
    copies = list(sim.copyPasteObjects(source_tree, 0))
    copied_set = set(copies)
    roots = [
        handle
        for handle in copies
        if int(sim.getObjectParent(handle)) not in copied_set
    ]
    if len(roots) != 1:
        sim.removeObjects(copies)
        raise RuntimeError(
            f"expected one copied hierarchy root, received {len(roots)}"
        )
    return int(roots[0])


def _prefix_tree_aliases(sim, root: int, prefix: str) -> None:
    for handle in _tree(sim, root):
        alias = str(sim.getObjectAlias(handle))
        sim.setObjectAlias(handle, prefix + alias)


def _rename_robot_tree(sim, root: int, source: str, destination: str) -> None:
    for handle in _tree(sim, root):
        alias = str(sim.getObjectAlias(handle))
        if source in alias:
            sim.setObjectAlias(handle, alias.replace(source, destination))


def _copy_into_workspace(
    sim,
    source_path: str,
    workspace: int,
    *,
    prefix: str | None = None,
    rename: tuple[str, str] | None = None,
) -> int:
    source = int(sim.getObject(source_path))
    copied_root = _copy_tree(sim, source)
    sim.setObjectParent(copied_root, workspace, True)
    if prefix:
        _prefix_tree_aliases(sim, copied_root, prefix)
    if rename:
        _rename_robot_tree(sim, copied_root, *rename)
    return copied_root


def _set_ground_extent(sim, x_offset: float) -> None:
    """Extend the original ground leftward while preserving its right edge."""
    ground = int(sim.getObject("/FiveCR5A_Cell/Ground_Group/Ground"))
    min_x = float(sim.getObjectFloatParam(ground, sim.objfloatparam_objbbox_min_x))
    max_x = float(sim.getObjectFloatParam(ground, sim.objfloatparam_objbbox_max_x))
    current_length = max_x - min_x
    original_length = 5.6
    target_length = original_length + abs(float(x_offset))
    if current_length <= 0:
        raise RuntimeError("ground has an invalid X extent")
    sim.scaleObject(ground, target_length / current_length, 1.0, 1.0, 0)
    # Original center/right edge are -0.55/2.25 m.  Grow only to the left.
    sim.setObjectPosition(
        ground,
        -1,
        [-0.55 + float(x_offset) / 2.0, -0.30, -0.01],
    )


def duplicate_workspace(sim, x_offset: float = DEFAULT_X_OFFSET) -> dict:
    if int(sim.getSimulationState()) != int(sim.simulation_stopped):
        raise RuntimeError("stop the simulation before duplicating the workspace")

    cell = int(sim.getObject("/FiveCR5A_Cell"))
    try:
        existing = int(sim.getObject(WORKSPACE_PATH))
    except Exception:
        existing = -1
    if existing != -1:
        _remove_tree(sim, existing)

    workspace = int(sim.createDummy(0.02))
    sim.setObjectAlias(workspace, "Workspace_Left_Copy")
    sim.setObjectParent(workspace, cell, True)
    sim.setObjectPosition(workspace, -1, [0.0, 0.0, 0.0])
    sim.setObjectInt32Param(workspace, sim.objintparam_visibility_layer, 0)

    copied_roots: list[int] = []
    for path in TABLE_OBJECTS + BASE_OBJECTS + AREA_OBJECTS + PART_OBJECTS:
        copied_roots.append(
            _copy_into_workspace(sim, path, workspace, prefix="WS2_")
        )

    robot_mapping = {"R1": "R6", "R2": "R7", "R3": "R8"}
    for path in ROBOT_OBJECTS:
        source_name = path.removeprefix("/")
        copied_roots.append(
            _copy_into_workspace(
                sim,
                path,
                workspace,
                rename=(source_name, robot_mapping[source_name]),
            )
        )

    for path in TARGET_OBJECTS:
        source_name = Path(path).name.split("_", 1)[0]
        copied_roots.append(
            _copy_into_workspace(
                sim,
                path,
                workspace,
                rename=(source_name, robot_mapping[source_name]),
            )
        )

    sim.setObjectPosition(workspace, -1, [float(x_offset), 0.0, 0.0])
    _set_ground_extent(sim, x_offset)

    table = int(sim.getObject(f"{WORKSPACE_PATH}/WS2_Damping_Table_Left"))
    robots = {
        name: list(sim.getObjectPosition(sim.getObject(f"{WORKSPACE_PATH}/{name}"), -1))
        for name in ("R6", "R7", "R8")
    }
    return {
        "workspace": WORKSPACE_PATH,
        "x_offset": float(x_offset),
        "copied_root_count": len(copied_roots),
        "object_count": len(_tree(sim, workspace)),
        "table_position": list(sim.getObjectPosition(table, -1)),
        "robot_positions": robots,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", default=23000, type=int)
    parser.add_argument("--offset-x", default=DEFAULT_X_OFFSET, type=float)
    parser.add_argument(
        "--output",
        type=Path,
        help="save the modified scene to this path; omit to modify only in memory",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    client = RemoteAPIClient(args.host, args.port)
    sim = client.require("sim")
    report = duplicate_workspace(sim, args.offset_x)
    if args.output:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        sim.saveScene(str(output))
        report["saved_scene"] = str(output)
    for key, value in report.items():
        print(f"{key}: {value}")
    print("R6/R7/R8 are display-only until new motion plans are validated.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
