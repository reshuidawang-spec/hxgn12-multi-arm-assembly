from __future__ import annotations

import hashlib
import unittest
from pathlib import Path

from scheduler.config_loader import load_yaml
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

    def test_robot_config_matches_latest_scene_tools(self):
        self.assertEqual(set(self.robots), set(ROBOT_IDS))
        for robot_id in ROBOT_IDS:
            self.assertEqual(self.robots[robot_id]["tip"], ROBOT_TIPS[robot_id])
            self.assertEqual(len(self.robots[robot_id]["position"]), 3)
            self.assertEqual(
                [get_joint_alias(robot_id, index) for index in range(1, 7)],
                [f"joint{index}" for index in range(1, 7)],
            )
        self.assertEqual(self.robots["R2"]["end_effector"], "vacuum")
        self.assertEqual(self.robots["R2"]["tip"], "R2_vacuum_tip")

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
        for name in POINTS:
            if name.endswith("_HOME_REF"):
                continue
            base_name = name.rsplit("_", 1)[0]
            self.assertTrue(
                name in product_builder or base_name in product_builder,
                name,
            )
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
            "R5_Device_Basket",
            "R6_Device_Basket",
            "Staging_Area",
            "Finished_Conveyor",
            "Cabinet_Product_REF",
        ):
            self.assertIn(key_object, builders)


if __name__ == "__main__":
    unittest.main()
