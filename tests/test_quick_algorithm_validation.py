from __future__ import annotations

import unittest

from scripts.run_quick_cbs_validation import cases, earliest_conflict, solve_cbs
from scripts.run_quick_eight_arm_validation import (
    DEFAULT_PLAN,
    Job,
    load_plan,
    recipe_durations,
    run_pipeline,
    validate_schedule,
)
from scripts.run_quick_rrt_validation import CASES, reference_prefix, rrt_connect


class QuickAlgorithmValidationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.plan = load_plan(DEFAULT_PLAN)

    def test_eight_arm_plan_has_thirty_actions(self):
        self.assertEqual(
            sum(len(actions) for actions in self.plan["actions"].values()),
            30,
        )

    def test_dynamic_pipeline_respects_precedence_and_module_capacity(self):
        durations = recipe_durations(self.plan)
        total = sum(durations["standard"])
        jobs = [
            Job("J1", "standard", 0, 1.4 * total, 1),
            Job("J2_URGENT", "reduced", 0.1 * total, total, 5),
            Job("J3", "standard", 0, 1.5 * total, 2),
        ]
        result = run_pipeline(jobs, durations, "dynamic")
        validate_schedule(result, jobs)

    def test_all_cbs_cases_finish_without_interval_conflict(self):
        for name, agents in cases().items():
            with self.subTest(name=name):
                result = solve_cbs(agents)
                self.assertTrue(result.solved)
                self.assertIsNone(earliest_conflict(result.windows))

    def test_representative_rrt_cases_find_reference_corridor(self):
        for robot, stem in CASES:
            with self.subTest(robot=robot, stem=stem):
                reference = reference_prefix(self.plan["actions"][robot][stem])
                success, _, _ = rrt_connect(reference, 1000 * int(robot[1:]))
                self.assertTrue(success)


if __name__ == "__main__":
    unittest.main()
