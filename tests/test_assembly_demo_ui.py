import importlib.util
import unittest
from pathlib import Path
from unittest.mock import patch


SERVER_PATH = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "assembly_demo_ui"
    / "server.py"
)
SPEC = importlib.util.spec_from_file_location("assembly_demo_ui_server", SERVER_PATH)
UI = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(UI)


class AssemblyDemoUiTests(unittest.TestCase):
    def test_four_cabinet_button_uses_mixed_recipe(self):
        command = UI.controller_command(4)
        self.assertEqual(command[command.index("--jobs") + 1], "4")
        self.assertEqual(command[command.index("--speed") + 1], "3.0")
        self.assertEqual(command[command.index("--black-job") + 1], "2")
        self.assertEqual(command[command.index("--reduced-job") + 1], "2")
        self.assertIn("--r7-r8-dual-entry", command)

    def test_web_controller_can_fall_back_to_stable_serial_mode(self):
        command = UI.controller_command(4, r7_r8_dual_entry=False)
        self.assertNotIn("--r7-r8-dual-entry", command)
        self.assertFalse(UI.Handler._bool_value(False, True))
        self.assertFalse(UI.Handler._bool_value("false", True))
        self.assertTrue(UI.Handler._bool_value(None, True))

        page = UI.INDEX_HTML.read_text(encoding="utf-8")
        self.assertIn('id="dual-entry-toggle" type="checkbox" checked', page)
        self.assertIn("r7_r8_dual_entry:dualEntry", page)

    def test_one_click_flow_starts_controller_after_scene_is_ready(self):
        with (
            patch.object(UI, "launch_sim", return_value=(True, "ready")),
            patch.object(UI, "_probe_sim_state", return_value="ready"),
            patch.object(
                UI, "start_controller", return_value=(True, "started")
            ) as start_controller,
        ):
            UI._launch_and_start(4, True)

        start_controller.assert_called_once_with(4, True)

    def test_report_summarizes_process_arm_time_and_finished_cabinets(self):
        UI.reset_report()
        UI.ingest_report_line(
            '[report-plan] {"jobs":2,"modules":{"1":2},'
            '"arms":{"R1":1,"R2":1}}'
        )
        UI.ingest_report_line(
            '[report] {"module":1,"job":1,"robot":"R1",'
            '"key":"R1:PICK","label":"pick","seconds":1.25}'
        )
        UI.ingest_report_line(
            '[report] {"module":1,"job":2,"robot":"R2",'
            '"key":"R2:PLACE","label":"place","seconds":2.5}'
        )
        UI.ingest_report_line('[report-job] {"job":1}')
        UI.ingest_report_line('[report-job] {"job":2}')

        report = UI.report_summary()

        self.assertEqual(report["status"], "finished")
        self.assertEqual(report["modules"][0]["pct"], 100.0)
        self.assertEqual(report["arms_completed"], 2)
        self.assertEqual(report["arms_total"], 2)
        self.assertEqual(report["jobs_completed"], 2)
        self.assertEqual(report["total_seconds"], 3.8)


if __name__ == "__main__":
    unittest.main()
