#!/usr/bin/env python3
"""Print the live R4 contact-pad and EDS/DIN-rail geometry."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_8arm_cabinet_assembly as planner


COLLISION_CONFIG = [
    -0.8355114444377667,
    -0.05396774778527488,
    -1.512690994439496,
    -0.004215574790670924,
    1.570803986293534,
    1.6952111216842123,
]


def world_aabb(sim, handle: int) -> list[list[float]]:
    vertices = np.asarray(sim.getShapeMesh(handle)[0], dtype=float).reshape(-1, 3)
    matrix = np.asarray(sim.getObjectMatrix(handle, -1), dtype=float).reshape(3, 4)
    world = vertices @ matrix[:, :3].T + matrix[:, 3]
    return [world.min(axis=0).round(6).tolist(), world.max(axis=0).round(6).tolist()]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=23000)
    parser.add_argument("--reproduce-collision", action="store_true")
    args = parser.parse_args()
    client = RemoteAPIClient(port=args.port)
    sim = client.require("sim")
    scene = None
    saved_matrices = {}
    saved_fingers = {}
    pallet = -1
    pallet_position = []
    if args.reproduce_collision:
        scene = planner.Scene(client)
        installed = planner.planning_product_state("R4", "EDS_PLACE")[1]
        for key in installed:
            part = scene.by_alias(planner.PARTS[key][0])
            saved_matrices[part] = list(sim.getObjectMatrix(part, -1))
            matrix = list(sim.getObjectMatrix(scene.by_alias(planner.PARTS[key][1]), -1))
            for index in range(3):
                matrix[3 + 4 * index] += (
                    planner.STATIONS["wb2"][index] - planner.REFERENCE_CENTER[index]
                )
            sim.setObjectMatrix(part, -1, matrix)
        pallet = scene.by_alias("Indexing_Pallet_1")
        pallet_position = list(sim.getObjectPosition(pallet, -1))
        sim.setObjectPosition(
            pallet, -1,
            [*planner.STATIONS["wb2"][:2], pallet_position[2]],
        )
        tool = scene.tool_roots["R4"]
        gap = planner.part_gripper_gap("R4", "eds")
        half = (gap + 0.020) / 2.0
        for side, sign in (("left", 1.0), ("right", -1.0)):
            finger = planner.unique_alias(
                sim, scene.roots["R4"], f"R4T_{side}_finger_link"
            )
            position = list(sim.getObjectPosition(finger, tool))
            saved_fingers[finger] = position
            position[1] = sign * half
            sim.setObjectPosition(finger, tool, position)
        scene.set_joints("R4", COLLISION_CONFIG)
    for path in (
        "/R4/R4T_left_inner_rubber_pad",
        "/R4/R4T_right_inner_rubber_pad",
        "/COL_Rail_2_1",
        "/EDS_1",
        "/R4_EDS_PLACE_TCP",
        "/R4_EDS_PLACE_APP",
        "/REF_rail_v1",
        "/REF_rail_v2",
        "/REF_eds",
    ):
        handle = int(sim.getObject(path))
        row = {
            "path": path,
            "handle": handle,
            "position": [round(float(v), 6) for v in sim.getObjectPosition(handle, -1)],
            "quaternion": [round(float(v), 6) for v in sim.getObjectQuaternion(handle, -1)],
        }
        if int(sim.getObjectType(handle)) == int(sim.object_shape_type):
            size, pose = sim.getShapeBB(handle)
            row["shape_bb_size"] = [round(float(v), 6) for v in size]
            row["shape_bb_pose"] = [round(float(v), 6) for v in pose]
            row["world_aabb"] = world_aabb(sim, handle)
        print(row, flush=True)
    if args.reproduce_collision:
        left = int(sim.getObject("/R4/R4T_left_inner_rubber_pad"))
        right = int(sim.getObject("/R4/R4T_right_inner_rubber_pad"))
        rail = int(sim.getObject("/COL_Rail_2_1"))
        print(
            {
                "left_pad_vs_rail": sim.checkCollision(left, rail),
                "right_pad_vs_rail": sim.checkCollision(right, rail),
            },
            flush=True,
        )
        for finger, position in saved_fingers.items():
            sim.setObjectPosition(finger, scene.tool_roots["R4"], position)
        for part, matrix in saved_matrices.items():
            sim.setObjectMatrix(part, -1, matrix)
        sim.setObjectPosition(pallet, -1, pallet_position)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
