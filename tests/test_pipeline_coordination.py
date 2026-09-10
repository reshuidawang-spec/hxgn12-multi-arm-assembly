from __future__ import annotations

import unittest

from scripts.run_8arm_cabinet_assembly import (
    ACTION_TARGETS,
    AssemblyRuntime,
    action_workspace,
)
from scripts.run_8arm_pipeline_assembly import (
    MODULE_DEPENDENCIES,
    MODULE_OPERATIONS,
    MODULE_ROBOTS,
    PipelineJob,
    dependencies_for_job,
    operation_id,
    operations_for_job,
    pipeline_takts,
)


class PipelineCoordinationTests(unittest.TestCase):
    def test_three_job_pipeline_fills_and_drains_in_five_takts(self):
        self.assertEqual(
            pipeline_takts(3),
            [
                {1: 1},
                {1: 2, 2: 1},
                {1: 3, 2: 2, 3: 1},
                {2: 3, 3: 2},
                {3: 3},
            ],
        )

    def test_four_job_pipeline_inserts_one_order_and_drains_in_six_takts(self):
        self.assertEqual(
            pipeline_takts(4),
            [
                {1: 1},
                {1: 2, 2: 1},
                {1: 3, 2: 2, 3: 1},
                {1: 4, 2: 3, 3: 2},
                {2: 4, 3: 3},
                {3: 4},
            ],
        )

    def test_module_operations_cover_every_accepted_action_once(self):
        expected = {
            (robot, stem)
            for robot, stems in ACTION_TARGETS.items()
            for stem in stems
        }
        scheduled = [
            (operation.robot, operation.stem)
            for operations in MODULE_OPERATIONS.values()
            for operation in operations
            if operation.robot is not None and operation.stem is not None
        ]
        self.assertEqual(set(scheduled), expected)
        self.assertEqual(len(scheduled), len(expected))

    def test_parallel_modules_never_share_a_public_workspace(self):
        max_operations = max(len(operations) for operations in MODULE_OPERATIONS.values())
        for index in range(max_operations):
            workspaces = []
            for operations in MODULE_OPERATIONS.values():
                if index >= len(operations):
                    continue
                operation = operations[index]
                if operation.robot is None or operation.stem is None:
                    continue
                workspace = action_workspace(operation.robot, operation.stem)
                if workspace is not None:
                    workspaces.append(workspace)
            self.assertEqual(len(workspaces), len(set(workspaces)))

    def test_robot_sets_are_disjoint_between_modules(self):
        robots = {
            module: {
                operation.robot
                for operation in operations
                if operation.robot is not None
            }
            for module, operations in MODULE_OPERATIONS.items()
        }
        self.assertTrue(robots[1].isdisjoint(robots[2]))
        self.assertTrue(robots[1].isdisjoint(robots[3]))
        self.assertTrue(robots[2].isdisjoint(robots[3]))

    def test_declared_interlock_sets_match_scheduled_robots(self):
        scheduled = {
            module: {
                operation.robot
                for operation in operations
                if operation.robot is not None
            }
            for module, operations in MODULE_OPERATIONS.items()
        }
        self.assertEqual(
            {module: set(robots) for module, robots in MODULE_ROBOTS.items()},
            scheduled,
        )

    def test_transport_animation_obeys_speed_multiplier(self):
        runtime = object.__new__(AssemblyRuntime)
        runtime.speed = 3.0
        self.assertEqual(runtime.scaled_transport_steps(80, minimum=24), 27)
        self.assertEqual(runtime.scaled_transport_steps(60, minimum=8), 20)
        self.assertEqual(runtime.scaled_transport_steps(100, minimum=16), 34)

    def test_dependency_graph_covers_operations_and_is_acyclic(self):
        for module, operations in MODULE_OPERATIONS.items():
            operation_ids = {operation_id(operation) for operation in operations}
            self.assertEqual(set(MODULE_DEPENDENCIES[module]), operation_ids)
            self.assertTrue(
                all(
                    dependencies.issubset(operation_ids)
                    for dependencies in MODULE_DEPENDENCIES[module].values()
                )
            )
            completed: set[str] = set()
            pending = set(operation_ids)
            while pending:
                ready = {
                    key for key in pending
                    if MODULE_DEPENDENCIES[module][key].issubset(completed)
                }
                self.assertTrue(ready, f"cyclic module {module}: {pending}")
                pending -= ready
                completed |= ready

    def test_module_one_exposes_safe_prefetch_pairs(self):
        dependencies = MODULE_DEPENDENCIES[1]
        after_shell_pick = {"R1:SHELL_PICK"}
        self.assertTrue(
            dependencies["R1:WB1_PLACE"].issubset(after_shell_pick)
        )
        self.assertTrue(
            dependencies["R2:RAIL_PICK_H"].issubset(after_shell_pick)
        )
        before_horizontal_place = {
            "R1:SHELL_PICK", "R1:WB1_PLACE", "R2:RAIL_PICK_H"
        }
        self.assertTrue(
            dependencies["R2:RAIL_PLACE_H"].issubset(
                before_horizontal_place
            )
        )
        self.assertEqual(
            dependencies["R3:RAIL_PICK_A"], {"R2:RAIL_PLACE_H"}
        )

    def test_module_two_exposes_requested_prefetch_pairs(self):
        dependencies = MODULE_DEPENDENCIES[2]
        after_psu_pick = {"R4:PSU_PICK"}
        self.assertTrue(
            dependencies["R4:PSU_PLACE"].issubset(after_psu_pick)
        )
        self.assertTrue(
            dependencies["R5:PLC_PICK"].issubset(after_psu_pick)
        )
        after_servo = after_psu_pick | {
            "R4:PSU_PLACE", "R4:SERVO_PICK", "R4:SERVO_PLACE",
            "R5:PLC_PICK",
        }
        self.assertTrue(
            dependencies["R4:EDS_PICK"].issubset(after_servo)
        )
        self.assertTrue(
            dependencies["R5:PLC_PLACE"].issubset(after_servo)
        )
        after_plc_and_eds_pick = after_servo | {
            "R4:EDS_PICK", "R5:PLC_PLACE",
        }
        self.assertTrue(
            dependencies["R4:EDS_PLACE"].issubset(after_plc_and_eds_pick)
        )
        self.assertTrue(
            dependencies["R5:DMA_PICK"].issubset(after_plc_and_eds_pick)
        )

    def test_reduced_black_recipe_reuses_paths_and_bypasses_skipped_nodes(self):
        job = PipelineJob(
            2, object(), recipe="reduced", cabinet_color="black"  # type: ignore[arg-type]
        )
        enabled = {
            operation_id(operation)
            for module in MODULE_OPERATIONS
            for operation in operations_for_job(job, module)
        }
        self.assertEqual(len(enabled), 24)
        self.assertTrue(
            {
                "R4:SERVO_PICK", "R4:SERVO_PLACE",
                "R5:DMA_PICK", "R5:DMA_PLACE",
                "R8:FILTER_PICK", "R8:FILTER_PLACE",
                "R7:SCREW_3", "R7:SCREW_4",
            }.isdisjoint(enabled)
        )
        module_two = dependencies_for_job(job, 2)
        self.assertEqual(module_two["R4:EDS_PICK"], {"R4:PSU_PLACE"})
        self.assertEqual(
            module_two["R5:PLC_PLACE"],
            {"R4:PSU_PLACE", "R5:PLC_PICK"},
        )
        module_three = dependencies_for_job(job, 3)
        self.assertEqual(
            module_three["R7:SCREW_1"], {"R8:COM5_PLACE"}
        )


if __name__ == "__main__":
    unittest.main()
