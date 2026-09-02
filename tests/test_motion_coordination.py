from __future__ import annotations

import json
import unittest

from robot_control.motion_common import RUNTIME_BRIDGE_CODE
from scripts.run_8arm_cabinet_assembly import (
    ACTION_TARGETS,
    CONTACT_STATION,
    MIN_TRANSIT_Z,
    MAX_TRANSFER_TILT,
    PLAN_SCHEMA_VERSION,
    PREFERRED_TRANSIT_Z,
    REFERENCE_CENTER,
    R1_EDGE_PLACE_APP_JOINTS,
    R1_EDGE_STOW_JOINTS,
    R1_EDGE_STOW_POSITION,
    R1_MAX_DOWN_TILT,
    R2_MAX_DOWN_TILT,
    R2_MIN_TRANSIT_Z,
    R2_RAIL_PLACE_APP_JOINTS,
    R2_RAIL_STOW_JOINTS,
    R2_RAIL_STOW_POSITION,
    R3_FIXED_CORRIDORS,
    R3_HANDOFF_DETOUR,
    R3_MAX_DOWN_TILT,
    R3_RAIL_STOW_JOINTS,
    R3_RAIL_STOW_POSITION,
    R4_HANDOFF_CENTER,
    DEFAULT_PLAN,
    STATIONS,
    config_distance,
    compose_pose,
    inverse_pose,
    nearest_down_quaternion,
    process_stages,
    quaternion_from_z_axis,
    rotate_vector,
    unwrap_config_near,
)


class MotionCoordinationTests(unittest.TestCase):
    def test_every_action_appears_once_in_coordinated_stage_graph(self):
        expected = {
            (robot, stem)
            for robot, stems in ACTION_TARGETS.items()
            for stem in stems
        }
        scheduled = [operation for stage in process_stages() for operation in stage]
        self.assertEqual(set(scheduled), expected)
        self.assertEqual(len(scheduled), len(expected))

    def test_prefetch_overlaps_non_conflicting_work(self):
        groups = [set(stage) for stage in process_stages()]
        self.assertIn(
            {("R5", "PLC_PICK"), ("R6", "SERVO_PICK")},
            groups,
        )
        self.assertIn(
            {("R6", "SERVO_PLACE"), ("R5", "PSU_PICK")},
            groups,
        )
        self.assertIn(
            {("R5", "PSU_PLACE"), ("R6", "DMA_PICK")},
            groups,
        )

    def test_low_zone_is_reserved_for_contact_legs(self):
        self.assertGreaterEqual(PREFERRED_TRANSIT_Z, 0.50)
        self.assertGreaterEqual(MIN_TRANSIT_Z, 0.44)
        for robot, stems in ACTION_TARGETS.items():
            for stem in stems:
                if "PLACE" in stem or "SCREW" in stem or stem in {
                    "WB1_PICK",
                    "HANDOFF_PICK",
                    "WB2_PICK",
                    "STAGING_PICK",
                }:
                    self.assertIn((robot, stem), CONTACT_STATION)

    def test_kinematic_bridge_uses_direct_position_write(self):
        direct = RUNTIME_BRIDGE_CODE.index("sim.setJointPosition")
        target = RUNTIME_BRIDGE_CODE.index("sim.setJointTargetPosition")
        self.assertLess(direct, target)
        self.assertEqual(PLAN_SCHEMA_VERSION, 31)
        self.assertAlmostEqual(MAX_TRANSFER_TILT, 0.6981317008)

    def test_process_stations_follow_elevated_fixture_surface(self):
        for station in ("wb1", "wb2", "staging", "output"):
            self.assertAlmostEqual(STATIONS[station][2], 0.27)
        self.assertAlmostEqual(REFERENCE_CENTER[2], 0.27)
        self.assertAlmostEqual(STATIONS["output"][2], 0.27)
        self.assertEqual(STATIONS["output"][:2], [0.75, 0.25])

    def test_r1_uses_edge_grip_carry_and_short_turn_place(self):
        self.assertEqual(R1_EDGE_STOW_POSITION, [-3.65, 1.3625, 0.482])
        self.assertEqual(len(R1_EDGE_STOW_JOINTS), 6)
        self.assertEqual(len(R1_EDGE_PLACE_APP_JOINTS), 6)
        self.assertAlmostEqual(R1_EDGE_PLACE_APP_JOINTS[4], 1.5702, places=3)
        self.assertAlmostEqual(R1_MAX_DOWN_TILT, 0.0872664626)

    def test_cartesian_branch_unwrap_avoids_full_joint_turn(self):
        reference = [-3.56, 0.0, 0.0, 0.0, 0.0, 0.0]
        sampled = [2.72, 0.1, 0.2, 0.3, 0.4, 0.5]
        unwrapped = unwrap_config_near(reference, sampled)
        self.assertLess(abs(unwrapped[0] - reference[0]), 0.1)
        self.assertEqual(unwrapped[2], sampled[2])

    def test_r2_uses_rail_end_grip_and_high_arc_carry(self):
        self.assertEqual(R2_RAIL_STOW_POSITION, [-3.76, -0.725, 0.5])
        self.assertEqual(len(R2_RAIL_STOW_JOINTS), 6)
        self.assertEqual(len(R2_RAIL_PLACE_APP_JOINTS), 6)
        self.assertAlmostEqual(R2_MAX_DOWN_TILT, 0.0872664626)
        self.assertGreaterEqual(R2_MIN_TRANSIT_Z, 0.48)

    def test_r3_uses_open_rack_edge_grips_without_cabinet_transport(self):
        self.assertEqual(R3_RAIL_STOW_POSITION, [-2.35, -0.76625, 0.48])
        self.assertEqual(len(R3_RAIL_STOW_JOINTS), 6)
        self.assertAlmostEqual(R3_MAX_DOWN_TILT, 0.1047197551)
        self.assertEqual(len(R3_HANDOFF_DETOUR), 5)
        self.assertEqual(len(R3_FIXED_CORRIDORS["RAIL_PLACE_A"]), 4)
        self.assertEqual(len(R3_FIXED_CORRIDORS["RAIL_PLACE_B"]), 4)
        self.assertNotIn("WB1_PICK", ACTION_TARGETS["R3"])
        self.assertNotIn("HANDOFF_PLACE", ACTION_TARGETS["R3"])

    def test_complete_cabinet_transfers_are_removed_from_robot_actions(self):
        forbidden = {
            "WB1_PICK", "HANDOFF_PLACE", "HANDOFF_PICK", "WB2_PLACE",
            "WB2_PICK", "STAGING_PLACE", "STAGING_PICK", "OUTPUT_PLACE",
        }
        scheduled = {
            stem for stems in ACTION_TARGETS.values() for stem in stems
        }
        self.assertTrue(forbidden.isdisjoint(scheduled))

    def test_r4_and_r8_are_park_only_in_conveyor_flow(self):
        self.assertEqual(ACTION_TARGETS["R4"], [])
        self.assertEqual(ACTION_TARGETS["R8"], [])

    def test_tool_transform_inverse_round_trip(self):
        pose = [0.12, -0.31, 0.27, 0.2, -0.3, 0.1, 0.9273618495]
        identity = compose_pose(pose, inverse_pose(pose))
        for actual, expected in zip(identity, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]):
            self.assertAlmostEqual(actual, expected, places=7)

    def test_down_projection_preserves_yaw_family(self):
        projected = nearest_down_quaternion([-0.86, 0.51, 0.03, 0.02])
        self.assertAlmostEqual(sum(value * value for value in projected), 1.0)
        self.assertEqual(projected[2:], [0.0, 0.0])
        self.assertLess(projected[0], 0.0)
        self.assertGreater(projected[1], 0.0)

    def test_physical_tool_axis_marker_follows_flange_to_tcp_vector(self):
        direction = [0.31, -0.44, 0.77]
        quaternion = quaternion_from_z_axis(direction)
        actual = rotate_vector(quaternion, [0.0, 0.0, 1.0])
        norm = sum(value * value for value in direction) ** 0.5
        for value, expected in zip(actual, direction):
            self.assertAlmostEqual(value, expected / norm, places=7)


if __name__ == "__main__":
    unittest.main()
