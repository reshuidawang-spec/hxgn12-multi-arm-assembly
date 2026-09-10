from __future__ import annotations

import unittest

from scripts.run_8arm_cabinet_assembly import (
    ACTION_TARGETS,
    AssemblyRuntime,
    action_workspace,
)
from scripts.run_8arm_pipeline_assembly import (
    MODULE_OPERATIONS,
    MODULE_ROBOTS,
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


if __name__ == "__main__":
    unittest.main()
