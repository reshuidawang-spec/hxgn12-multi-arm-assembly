#!/usr/bin/env python3
"""Reproduce and inspect the R6 contactor-place fingertip collision."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

sys.path.insert(0, str(Path(__file__).resolve().parent))
import run_8arm_cabinet_assembly as planner


CONFIG = [
    -1.2047761928952896,
    0.2161302226802464,
    1.3445998563998236,
    0.010185236313877577,
    -1.5708070005104406,
    -0.9429792187566665,
]


def world_aabb(sim, handle: int) -> list[list[float]]:
    vertices = np.asarray(sim.getShapeMesh(handle)[0], dtype=float).reshape(-1, 3)
    matrix = np.asarray(sim.getObjectMatrix(handle, -1), dtype=float).reshape(3, 4)
    world = vertices @ matrix[:, :3].T + matrix[:, 3]
    return [world.min(axis=0).round(6).tolist(), world.max(axis=0).round(6).tolist()]


def main() -> int:
    client = RemoteAPIClient(port=23000)
    sim = client.require("sim")
    scene = planner.Scene(client)
    saved: dict[int, list[float]] = {}
    station, installed = planner.planning_product_state("R6", "BREAKER_PLACE")
    for key in installed:
        handle = scene.by_alias(planner.PARTS[key][0])
        saved[handle] = list(sim.getObjectMatrix(handle, -1))
        matrix = list(sim.getObjectMatrix(scene.by_alias(planner.PARTS[key][1]), -1))
        for axis in range(3):
            matrix[3 + 4 * axis] += (
                planner.STATIONS[station][axis] - planner.REFERENCE_CENTER[axis]
            )
        sim.setObjectMatrix(handle, -1, matrix)
    tool = scene.tool_roots["R6"]
    finger_saved = {}
    gap = planner.part_gripper_gap("R6", "breaker")
    half = (gap + planner.GRIPPER_FINGER_THICKNESSES["R6"]) / 2.0
    for side, sign in (("left", 1.0), ("right", -1.0)):
        finger = planner.unique_alias(sim, scene.roots["R6"], f"R6T_{side}_finger_link")
        position = list(sim.getObjectPosition(finger, tool))
        finger_saved[finger] = position
        position[1] = sign * half
        sim.setObjectPosition(finger, tool, position)
    scene.set_joints("R6", CONFIG)
    aliases = (
        "R6T_right_inner_rubber_pad",
        "R6T_right_lower_integrated_finger",
        "COL_Contactor_1_1",
        "Breaker_1",
    )
    rail = scene.by_alias("COL_Contactor_1_1")
    for alias in aliases:
        handle = scene.by_alias(alias)
        size, _ = sim.getShapeBB(handle)
        print({
            "alias": alias,
            "size": [round(float(v), 6) for v in size],
            "position": [round(float(v), 6) for v in sim.getObjectPosition(handle, -1)],
            "aabb": world_aabb(sim, handle),
            "rail_collision": sim.checkCollision(handle, rail) if handle != rail else None,
        })
    for finger, position in finger_saved.items():
        sim.setObjectPosition(finger, tool, position)
    for handle, matrix in saved.items():
        sim.setObjectMatrix(handle, -1, matrix)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
