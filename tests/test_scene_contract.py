from __future__ import annotations

import hashlib
import unittest
from pathlib import Path

from scheduler.config_loader import load_yaml
from scripts.build_cabinet_product_scene import (
    WB1_MICRO_INDEX_X,
    WB2_MICRO_INDEX_X,
    _load_manifest,
    build_paired_targets,
)
from sim_bridge.process_manager import CoppeliaProcessManager
from sim_bridge.scene_objects import (
    POINTS,
    PROCESS_COMMANDS,
    QUALITY_COMMANDS,
    ROBOT_IDS,
    ROBOT_TARGET_NAMES,
    ROBOT_TIPS,
    SCENE_ROOT,
    SCENE_FILE,
    get_joint_alias,
    get_point_path,
)


ROOT = Path(__file__).resolve().parents[1]


class SceneContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.contract = load_yaml(ROOT / "configs" / "scene_contract.yaml")
        cls.points = load_yaml(ROOT / "configs" / "points.yaml")
        cls.robots = load_yaml(ROOT / "configs" / "robots.yaml")["robots"]
        cls.assignment = load_yaml(
            ROOT / "configs" / "assembly_task_assignment.yaml"
        )
        cls.products = load_yaml(ROOT / "configs" / "product_types.yaml")

    def test_checked_in_scene_matches_latest_contract_fingerprint(self):
        scene = ROOT / "scenes" / self.contract["scene"]["file"]
        digest = hashlib.sha256(scene.read_bytes()).hexdigest()
        self.assertEqual(scene.stat().st_size, self.contract["scene"]["size"])
        self.assertEqual(digest, self.contract["scene"]["sha256"])
        self.assertEqual(self.contract["scene"]["root"], SCENE_ROOT)

    def test_software_launcher_uses_the_contract_scene(self):
        manager = CoppeliaProcessManager()
        expected = ROOT / "scenes" / self.contract["scene"]["file"]
        self.assertEqual(SCENE_FILE, self.contract["scene"]["file"])
        self.assertEqual(manager.scene, expected)

    def test_point_config_exactly_covers_scene_target_contract(self):
        self.assertEqual(set(self.points), set(POINTS))
        target_count = sum(len(names) for names in ROBOT_TARGET_NAMES.values())
        self.assertEqual(target_count, self.contract["counts"]["process_targets"])
        for name, path in POINTS.items():
            self.assertEqual(get_point_path(name), path)
            self.assertEqual(len(self.points[name]["position"]), 3)
            self.assertEqual(len(self.points[name]["orientation_quaternion"]), 4)

        for base_name in build_paired_targets(_load_manifest()):
            tcp = self.points[f"{base_name}_TCP"]
            app = self.points[f"{base_name}_APP"]
            self.assertEqual(app["position"][:2], tcp["position"][:2])
            self.assertGreater(app["position"][2], tcp["position"][2])
            self.assertEqual(
                app["orientation_quaternion"], tcp["orientation_quaternion"]
            )

        targets = build_paired_targets(_load_manifest())
        rail_b_x = targets["R3_RAIL_PLACE_B"][0][0]
        self.assertAlmostEqual(rail_b_x, -3.35145 + WB1_MICRO_INDEX_X)
        self.assertGreater(rail_b_x, -3.35145)
        contactor_x = targets["R6_CONTACTOR_PLACE"][0][0]
        breaker_x = targets["R6_BREAKER_PLACE"][0][0]
        self.assertAlmostEqual(contactor_x, -1.09315 + WB2_MICRO_INDEX_X)
        self.assertAlmostEqual(
            breaker_x, -1.06565 + WB2_MICRO_INDEX_X + 0.008
        )

    def test_robot_config_matches_latest_scene_tools(self):
        self.assertEqual(set(self.robots), set(ROBOT_IDS))
        for robot_id in ROBOT_IDS:
            self.assertEqual(self.robots[robot_id]["tip"], ROBOT_TIPS[robot_id])
            self.assertEqual(len(self.robots[robot_id]["position"]), 3)
            self.assertEqual(
                [get_joint_alias(robot_id, index) for index in range(1, 7)],
                [f"joint{index}" for index in range(1, 7)],
            )
        self.assertEqual(
            self.robots["R2"]["end_effector"], "magnetic_gripper"
        )
        self.assertEqual(self.robots["R2"]["tip"], "R2_vacuum_tip")
        self.assertEqual(
            self.robots["R3"]["end_effector"], "magnetic_gripper"
        )
        self.assertEqual(self.robots["R3"]["tip"], "R3_gripper_tip")
        self.assertEqual(self.robots["R8"]["end_effector"], "vacuum")
        self.assertEqual(self.robots["R8"]["tip"], "R8_vacuum_tip")

    def test_r8_filter_uses_a_surface_clear_vacuum_point(self):
        targets = build_paired_targets(_load_manifest())
        pick_tcp = targets["R8_FILTER_PICK"][0]
        place_tcp = targets["R8_FILTER_PLACE"][0]
        for actual, expected in zip(pick_tcp[:2], (0.65, -0.50)):
            self.assertAlmostEqual(actual, expected, places=4)
        self.assertLess(place_tcp[2], 0.412)
        self.assertGreater(pick_tcp[2], 0.282)
        self.assertLess(pick_tcp[2], 0.32)
        self.assertAlmostEqual(
            targets["R8_FILTER_PICK"][1], 0.120, places=6
        )

    def test_assembly_assignment_covers_every_robot_and_part_once(self):
        tasks = self.assignment["tasks"]
        workspaces = self.assignment["workspaces"]
        task_ids = [task["id"] for task in tasks]
        self.assertEqual(len(task_ids), len(set(task_ids)))
        task_index = {task_id: index for index, task_id in enumerate(task_ids)}

        assigned_robots = {task["robot"] for task in tasks}
        self.assertEqual(assigned_robots, set(ROBOT_IDS))
        assigned_actions = set()
        for task in tasks:
            robot = task["robot"]
            workspace = task["workspace"]
            self.assertIn(robot, workspaces[workspace]["members"])
            self.assertEqual(
                task["required_tool"], self.robots[robot]["end_effector"]
            )
            for action in task["actions"]:
                self.assertIn(f"{robot}_{action}_APP", POINTS)
                self.assertIn(f"{robot}_{action}_TCP", POINTS)
                self.assertNotIn((robot, action), assigned_actions)
                assigned_actions.add((robot, action))
            for dependency in task["after"]:
                self.assertIn(dependency, task_index)
                self.assertLess(task_index[dependency], task_index[task["id"]])

        manifest = load_yaml(
            ROOT / "models" / "cabinet" / "processed" / "manifest.json"
        )
        assigned_parts = [part for task in tasks for part in task["parts"]] + self.assignment['policy']['preassembled_parts']
        self.assertEqual(len(assigned_parts), len(set(assigned_parts)))
        self.assertEqual(set(assigned_parts), set(manifest["parts"]))

    @unittest.skip(
        "legacy product process model kept during the 8-robot rebuild; "
        "product_types.yaml will be rewritten with the new line process"
    )
    def test_product_tasks_reference_known_points_and_scene_commands(self):
        for product_type in ("A", "B", "C"):
            for step in self.products[product_type]["processes"]:
                self.assertIn(step["point"], POINTS)
                command = step.get("scene_done_cmd", "")
                if command:
                    self.assertIn(command, PROCESS_COMMANDS)
                if step["process"] == "inspect":
                    self.assertEqual(command, "")
        for step in self.products["post_inspection"]:
            self.assertIn(step["point"], POINTS)
            self.assertIn(step["scene_done_cmd"], PROCESS_COMMANDS)

    def test_quality_commands_are_explicit_scene_commands(self):
        self.assertEqual(
            QUALITY_COMMANDS,
            {"OK": "CAMERA_GOOD", "NG": "CAMERA_DEFECT"},
        )
        self.assertTrue(set(QUALITY_COMMANDS.values()).issubset(PROCESS_COMMANDS))

    def test_latest_builder_sources_define_the_contract(self):
        # The legacy Lua generators were retired; the scene is now produced
        # exclusively by the ZMQ builder scripts.
        rename_builder = (
            ROOT / "scripts" / "rename_scene_robots.py"
        ).read_text(encoding="utf-8")
        line_builder = (
            ROOT / "scripts" / "build_new_line_scene.py"
        ).read_text(encoding="utf-8")
        product_builder = (
            ROOT / "scripts" / "build_cabinet_product_scene.py"
        ).read_text(encoding="utf-8")
        builders = rename_builder + line_builder + product_builder

        for tip in ROBOT_TIPS.values():
            self.assertIn(tip, rename_builder)
        generated = set()
        for base_name in build_paired_targets(_load_manifest()):
            generated.update({f"{base_name}_APP", f"{base_name}_TCP"})
        generated.update(f"{robot}_HOME_REF" for robot in ROBOT_IDS)
        self.assertEqual(generated, set(POINTS))
        # HOME_REF dummies are created programmatically from
        # HOME_REF_POSITIONS in the product builder.
        self.assertIn("HOME_REF_POSITIONS", product_builder)
        for robot_id in ROBOT_IDS:
            self.assertIn(f'"{robot_id}":', product_builder)
        for key_object in (
            "Cabinet_Conveyor",
            "Workbench1_Area",
            "Workbench2_Area",
            "Handoff_Area",
            "Shell_Stack",
            "R2_Stand",
            "R3_Rail_Rack",
            "R4_Device_Basket",
            "R5_Device_Basket",
            "R6_Device_Basket",
            "R8_Device_Basket",
            "Public_Workspace_1",
            "Public_Workspace_2",
            "Public_Workspace_3",
            "Finished_Conveyor",
            "Cabinet_Product_REF",
        ):
            self.assertIn(key_object, builders)


if __name__ == "__main__":
    unittest.main()
