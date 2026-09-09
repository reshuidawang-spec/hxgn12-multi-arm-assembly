from __future__ import annotations

import json
import unittest

from robot_control.motion_common import RUNTIME_BRIDGE_CODE
from sim_bridge.motion_policy import (
    EXPECTED_CYCLE,
    EXPECTED_FALLBACK_ORDER,
    load_motion_policy,
    safe_height_candidates,
)
from scripts.run_8arm_cabinet_assembly import (
    ACTION_TARGETS,
    CONTACT_STATION,
    MIN_TRANSIT_Z,
    MAX_TRANSFER_TILT,
    LEGACY_SPECIAL_CORRIDORS_ENABLED,
    PLAN_SCHEMA_VERSION,
    PLACE_PART,
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
    R8_COM5_TRANSIT_Z,
    R6_STOW_Z,
    R8_DOOR_TRANSIT_Z,
    R4_HANDOFF_CENTER,
    DEFAULT_PLAN,
    STATIONS,
    action_workspace,
    config_distance,
    compose_pose,
    inverse_pose,
    nearest_down_quaternion,
    process_stages,
    run_wb1_process,
    planning_action_targets,
    partial_plan_path,
    pick_stem_for_part,
    quaternion_from_z_axis,
    rotate_vector,
    reset_checkpoint_robots,
    retain_checkpoint_action_prefix,
    unwrap_config_near,
)


class MotionCoordinationTests(unittest.TestCase):
    def test_incremental_planning_selects_one_robot_and_uses_partial_path(self):
        self.assertEqual(planning_action_targets({"R1"}), {"R1": ACTION_TARGETS["R1"]})
        self.assertEqual(
            partial_plan_path(DEFAULT_PLAN),
            DEFAULT_PLAN.with_name("eight_arm_cabinet.partial.json"),
        )
        with self.assertRaises(ValueError):
            planning_action_targets({"R9"})

    def test_every_place_has_one_preceding_same_robot_pick(self):
        for (robot, place_stem), part_key in PLACE_PART.items():
            pick_stem = pick_stem_for_part(robot, part_key)
            self.assertLess(
                ACTION_TARGETS[robot].index(pick_stem),
                ACTION_TARGETS[robot].index(place_stem),
            )

    def test_selected_robot_rebuild_preserves_prior_checkpoint_actions(self):
        checkpoint = {
            "stow": {"R1": [1], "R2": [2]},
            "stow_positions": {"R1": [1], "R2": [2]},
            "actions": {"R1": {"A": {}}, "R2": {"B": {}}},
        }
        reset_checkpoint_robots(checkpoint, {"R2"})
        self.assertEqual(checkpoint["actions"], {"R1": {"A": {}}})
        self.assertEqual(checkpoint["stow"], {"R1": [1]})
        self.assertEqual(checkpoint["stow_positions"], {"R1": [1]})

    def test_scene_adoption_removes_actions_after_audited_prefix(self):
        checkpoint = {
            "actions": {
                "R1": {"A": {}},
                "R7": {"G": {}},
                "R8": {"STALE_DOOR_PICK": {}},
            }
        }
        retain_checkpoint_action_prefix(
            checkpoint, ("R1", "R2", "R3", "R4", "R5", "R6", "R7")
        )
        self.assertEqual(set(checkpoint["actions"]), {"R1", "R7"})

    def test_every_action_appears_once_in_coordinated_stage_graph(self):
        expected = {
            (robot, stem)
            for robot, stems in ACTION_TARGETS.items()
            for stem in stems
        }
        scheduled = [operation for stage in process_stages() for operation in stage]
        self.assertEqual(set(scheduled), expected)
        self.assertEqual(len(scheduled), len(expected))

    def test_assigned_device_and_door_actions_are_scheduled(self):
        scheduled = {operation for stage in process_stages() for operation in stage}
        for operation in {
            ("R4", "PSU_PLACE"), ("R4", "SERVO_PLACE"), ("R4", "EDS_PLACE"),
            ("R5", "PLC_PLACE"), ("R5", "DMA_PLACE"),
            ("R6", "CONTACTOR_PLACE"), ("R6", "BREAKER_PLACE"),
            ("R8", "COM5_PLACE"), ("R8", "FILTER_PLACE"),
        }:
            self.assertIn(operation, scheduled)

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
        self.assertEqual(PLAN_SCHEMA_VERSION, 36)
        self.assertAlmostEqual(MAX_TRANSFER_TILT, 0.1047197551)
        self.assertFalse(LEGACY_SPECIAL_CORRIDORS_ENABLED)

    def test_vertical_pi_policy_is_machine_enforced(self):
        policy = load_motion_policy()
        self.assertEqual(tuple(policy["canonical_cycle"]), EXPECTED_CYCLE)
        self.assertEqual(tuple(policy["fallback_order"]), EXPECTED_FALLBACK_ORDER)
        self.assertEqual(policy["orientation"]["yaw_change"], "safe_height_only")
        self.assertTrue(policy["contact_exclusions"]["robot_links_never_waived"])
        self.assertEqual(len(policy["fingerprint"]["sha256"]), 64)

    def test_safe_height_fallbacks_only_raise(self):
        heights = safe_height_candidates(0.50, 0.44, 0.62, 0.05)
        self.assertEqual(heights, [0.50, 0.55, 0.60, 0.62])
        self.assertTrue(all(right > left for left, right in zip(heights, heights[1:])))

    def test_coordinated_stage_has_at_most_one_workspace_entry(self):
        for stage in process_stages():
            entries = [
                action_workspace(robot, stem)
                for robot, stem in stage
                if action_workspace(robot, stem) is not None
            ]
            self.assertLessEqual(len(entries), 1, stage)

    def test_process_stations_follow_elevated_fixture_surface(self):
        for station in (
            "wb1", "wb1_micro", "wb2", "wb2_micro", "staging", "output"
        ):
            self.assertAlmostEqual(STATIONS[station][2], 0.27)
        self.assertAlmostEqual(REFERENCE_CENTER[2], 0.27)
        self.assertAlmostEqual(STATIONS["output"][2], 0.27)
        self.assertEqual(STATIONS["output"][:2], [0.75, 0.25])
        self.assertAlmostEqual(STATIONS["wb1_micro"][0] - STATIONS["wb1"][0], 0.12)
        self.assertEqual(STATIONS["wb1_micro"][1], STATIONS["wb1"][1])
        self.assertAlmostEqual(STATIONS["wb2_micro"][0] - STATIONS["wb2"][0], 0.12)
        self.assertEqual(STATIONS["wb2_micro"][1], STATIONS["wb2"][1])
        self.assertAlmostEqual(R8_DOOR_TRANSIT_Z, 0.50)

    def test_r3_b_runs_after_wb1_micro_index(self):
        class RecordingRuntime:
            def __init__(self):
                self.calls = []
                self.assembly = None
                self.events = set()

            def pick(self, robot, stem, key):
                self.calls.append(("pick", robot, stem, key))
                return object(), lambda: None

            def place(self, robot, stem, key, station):
                self.calls.append(("place", robot, stem, key, station))
                return object(), lambda: None

            def execute_pair(self, entries, label, **kwargs):
                self.calls.append(("execute", label, kwargs))

            def index_pallet(self, station, **kwargs):
                self.calls.append(("index", station, kwargs))

            def track(self, robot, stem):
                self.calls.append(("track", robot, stem))
                return type("RecordedTrack", (), {"exclusions": []})()

            def execute_screw(self, track, index, **kwargs):
                self.calls.append(("screw", index, kwargs))

        runtime = RecordingRuntime()
        run_wb1_process(runtime)
        micro = runtime.calls.index(
            ("index", "wb1_micro", {
                "wait_for": ("RAIL_A_DONE",), "emits": "WB1_MICRO_INDEXED"
            })
        )
        pick_b = next(
            index for index, call in enumerate(runtime.calls)
            if call[:3] == ("pick", "R3", "RAIL_PICK_B")
        )
        place_b = next(
            call for call in runtime.calls
            if call[:3] == ("place", "R3", "RAIL_PLACE_B")
        )
        self.assertLess(micro, pick_b)
        self.assertEqual(place_b[4], "wb1_micro")

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

    def test_r6_stow_and_r8_com5_height_remain_above_floor(self):
        self.assertEqual(R6_STOW_Z, 0.48)
        self.assertEqual(R8_COM5_TRANSIT_Z, 0.50)
        self.assertGreaterEqual(R6_STOW_Z, MIN_TRANSIT_Z)
        self.assertGreaterEqual(R8_COM5_TRANSIT_Z, MIN_TRANSIT_Z)

    def test_complete_cabinet_transfers_are_removed_from_robot_actions(self):
        forbidden = {
            "WB1_PICK", "HANDOFF_PLACE", "HANDOFF_PICK", "WB2_PLACE",
            "WB2_PICK", "STAGING_PLACE", "STAGING_PICK", "OUTPUT_PLACE",
        }
        scheduled = {
            stem for stems in ACTION_TARGETS.values() for stem in stems
        }
        self.assertTrue(forbidden.isdisjoint(scheduled))

    def test_r4_and_r8_execute_their_assigned_tasks(self):
        self.assertEqual(
            ACTION_TARGETS["R4"],
            ["PSU_PICK", "PSU_PLACE", "SERVO_PICK", "SERVO_PLACE", "EDS_PICK", "EDS_PLACE"],
        )
        self.assertEqual(
            ACTION_TARGETS["R8"],
            ["COM5_PICK", "COM5_PLACE", "FILTER_PICK", "FILTER_PLACE"],
        )

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
