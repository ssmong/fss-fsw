import csv
import gzip
import json
import math
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch


class RecordingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        scripts = str(Path(__file__).resolve().parents[1] / "scripts")
        with patch.object(sys, "path", [scripts, *sys.path]):
            try:
                import live_plot
            except ModuleNotFoundError as exc:
                if exc.name in {"PySide6", "pyqtgraph", "numpy"}:
                    raise unittest.SkipTest(str(exc)) from exc
                raise
        cls.gui = live_plot
        with patch.dict(os.environ, QT_QPA_PLATFORM="offscreen"):
            cls.app = live_plot.QApplication.instance() or live_plot.QApplication([])

    def test_archive_uses_every_received_sample_and_unfiltered_logs(self):
        from PySide6.QtGui import QImage
        with tempfile.TemporaryDirectory() as root:
            fss = self.gui.DashboardNode()
            fss.request = lambda *args: None
            run = fss.recorder.start(root, "self-tossing", "FSS-2")
            self.assertRegex(run.relative_to(root).as_posix(), r"^self-tossing/\d{4}-\d{2}-\d{2}/\d{2}-\d{2}-\d{2}$")
            header = NS(frame_id="table", stamp=NS(sec=20, nanosec=100))
            fss.receive("vicon", NS(header=header, x=1.2, y=0.4, theta=.1))
            fss.receive("vicon", NS(header=header, x=1.3, y=0.5, theta=.2))
            fss.recorder.record("ekf", NS(header=header, x=9, y=9, theta=9))
            fss.receive("imu", NS(header=header, angular_velocity=NS(z=1.25), orientation_covariance=[math.nan]*9))
            fss.receive("valve", NS(header=header, commanded_duty_cycles=[0.3]*8, applied_values=[1200]*8,
                                    main_valve_open=True, modbus_rtt_ms=2.0))
            fss.receive("valve_output", NS(header=header, duty_cycles=[0.0]*8))
            fss.receive("log", NS(name="arm", level=10, msg="숨은 DEBUG 메시지"))
            fss.send_command("target", ["pid", .2, .3, .4])
            fss.set_command_result("accepted")
            fss.recorder.stop("disconnected")
            fss.receive("vicon", NS(header=header, x=0, y=0, theta=0))
            with gzip.open(run / "telemetry.jsonl.gz", "rt", encoding="utf-8") as fp:
                records = [json.loads(line) for line in fp]
            self.assertEqual([r["kind"] for r in records], ["session_context", "pose", "pose", "imu",
                                                          "valve", "valve_output", "log", "command", "command_result"])
            self.assertEqual(records[3]["data"]["angular_velocity"]["z"], 1.25)
            self.assertEqual(records[3]["data"]["orientation_covariance"], [None]*9)
            self.assertEqual(records[4]["data"]["header"]["stamp"]["nanosec"], 100)
            self.assertNotIn("applied_values", records[4]["data"])
            self.assertEqual(records[5]["data"], {"header": records[4]["data"]["header"],
                                                  "duty_cycles": [0.0]*8})
            self.assertEqual(records[7]["data"]["value"], ["pid", .2, .3, .4])
            self.assertTrue(all(r["elapsed_s"] >= 0 and r["received_unix_ns"] > 0 for r in records))
            self.assertIn("[DEBUG] arm: 숨은 DEBUG 메시지", (run / "logs.txt").read_text(encoding="utf8"))
            summary = json.loads((run / "session.json").read_text(encoding="utf8"))
            self.assertEqual((summary["vehicle"], summary["counts"]["pose"], summary["stop_reason"]),
                             ("FSS-2", 2, "disconnected"))
            self.assertEqual(summary["pose_source"], "vicon")
            self.assertNotIn("ekf", summary["counts"])
            with (run / "pose.csv").open(encoding="utf-8", newline="") as fp:
                poses = list(csv.DictReader(fp))
            self.assertEqual([r["x_m"] for r in poses], ["1.2", "1.3"])
            self.assertEqual(poses[-1]["yaw_rad"], "0.2")
            self.assertEqual(poses[-1]["sample_stamp_ns"], "20000000100")
            self.assertEqual(summary["status"], "finished")
            self.assertGreater(QImage(str(run / "trajectory.png")).width(), 600)

    def test_archive_finalizes_real_mp4_before_gui_shutdown(self):
        from mission_recording import ExperimentRecorder
        with tempfile.TemporaryDirectory() as root:
            recorder = ExperimentRecorder()
            run = recorder.start(root, "video", "A")
            recorder.record("vicon", NS(x=1., y=2., theta=.1))
            recorder.capture({"pose": [1., 2., .1], "ghost": [1.2, 2., .2],
                              "ghost_arm": [[0., 0.], [.2, .2]]})
            deadline = time.monotonic() + 10
            while not recorder._video.frames_written and not recorder.video_error and time.monotonic() < deadline:
                self.app.processEvents()
                time.sleep(.005)
            self.assertFalse(recorder.video_error, recorder.video_error)
            recorder.stop()
            self.assertTrue(recorder.finish_video())
            summary = json.loads((run / "session.json").read_text())
            self.assertEqual(summary["video"]["status"], "finished")
            self.assertGreaterEqual(summary["video"]["frames"], 1)
            self.assertGreater((run / "trajectory.mp4").stat().st_size, 1000)

    def test_video_completion_updates_its_own_run_and_failure_keeps_telemetry(self):
        from PySide6.QtCore import QObject, Signal
        from mission_recording import ExperimentRecorder

        class Encoder(QObject):
            finalized = Signal(str)

            def __init__(self, directory, fps):
                super().__init__()
                self.path = Path(directory) / "trajectory.mp4"
                self.finished = False
                self.error = ""
                self.frames_written = self.frames_dropped = 0

            def capture(self, snapshot, elapsed):
                self.frames_written += 1

            def stop(self):
                pass

            def complete(self, error=""):
                self.finished, self.error = True, error
                self.finalized.emit(str(self.path))

        with tempfile.TemporaryDirectory() as root, patch("mission_recording.VideoRecorder", Encoder):
            recorder = ExperimentRecorder()
            first = recorder.start(root, "self-tossing", "A")
            recorder.capture({"pose": [1, 2, 0], "ghost": [1.1, 2, 0]})
            old_video = recorder._video
            recorder.stop()
            second = recorder.start(root, "self-tossing", "A")
            recorder.capture({})
            new_video = recorder._video
            old_video.complete()
            self.assertEqual(json.loads((first / "session.json").read_text())["video"]["status"], "finished")
            self.assertEqual(json.loads((second / "session.json").read_text())["status"], "recording")
            new_video.complete("disk full")
            self.assertTrue(recorder.active)
            self.assertIn("disk full", recorder.video_error)
            recorder.record("vicon", NS(x=1, y=2, theta=.1))
            recorder.stop()
            summary = json.loads((second / "session.json").read_text())
            self.assertEqual(summary["video"]["status"], "error")
            self.assertEqual(summary["status"], "finished")
            self.assertEqual(summary["counts"]["pose"], 1)

    def test_conflicting_names_and_io_failure(self):
        from datetime import datetime, timezone
        from mission_recording import ExperimentRecorder
        with tempfile.TemporaryDirectory() as root, patch("mission_recording.datetime") as clock:
            clock.now.return_value = datetime(2026, 11, 18, tzinfo=timezone.utc)
            first, second = ExperimentRecorder(), ExperimentRecorder()
            for invalid in ("../private", "foo/bar", "CON", "abc."):
                with self.assertRaises(ValueError):
                    first.start(root, invalid, "FSS-1")
            a = first.start(root, "", "FSS-1")
            b = second.start(root, "default", "FSS-2")
            self.assertEqual(a.parent, b.parent)
            self.assertNotEqual(a, b)
            first.stop()
            with patch.object(second, "_flush", side_effect=OSError("disk full")):
                second.record("test", 1)
            self.assertFalse(second.active)
            self.assertIn("disk full", second.error)
            self.assertEqual(json.loads((b / "session.json").read_text(encoding="utf8"))["status"], "error")
            with gzip.open(a / "telemetry.jsonl.gz") as fp:
                self.assertEqual(fp.read(), b"")

    def test_tracking_export_retains_aligned_samples_and_missing_angles(self):
        with tempfile.TemporaryDirectory() as root:
            first_node, second_node = self.gui.DashboardNode(), self.gui.DashboardNode()
            a, b = first_node.recorder, second_node.recorder
            a_dir, b_dir = a.start(root, "demo", "FSS-A"), b.start(root, "demo", "FSS-B")
            point = {"run_id": "run-1", "plan_key": "trial/Nominal", "plan_id": "T1", "phase": "COAST",
                     "sample_stamp_s": 12.0, "control_stamp_s": 12.02, "feedback_age_s": 0.02, "plan_time_s": 3.5,
                     "actual_x_m": 1.3, "actual_y_m": 0.6, "planned_x_m": 1.0, "planned_y_m": 1.0,
                     "error_x_m": 0.3, "error_y_m": -0.4, "error_xy_m": 0.5, "planned_yaw_rad": math.nan,
                     "actual_elbow_rad": 0.2, "planned_elbow_rad": 0.5, "error_elbow_rad": -0.3}
            a.record("tracking_error", {**point, "sample_stamp_s": None})
            a.record("tracking_error", {**point, "actual_x_m": math.inf})
            first_node.receive("tracking_error", NS(**point))
            second_node.receive("tracking_error", {**point, "run_id": "run-B", "plan_time_s": 4})
            a.stop()
            b.stop()
            a.record("tracking_error", point)
            with (a_dir / "tracking_error.csv").open(encoding="utf-8", newline="") as fp:
                first, = csv.DictReader(fp)
            with (b_dir / "tracking_error.csv").open(encoding="utf-8", newline="") as fp:
                second, = csv.DictReader(fp)
            self.assertEqual((first["run_id"], second["run_id"]), ("run-1", "run-B"))
            self.assertEqual((first["plan_time_s"], second["plan_time_s"]), ("3.5", "4"))
            self.assertEqual(first["error_xy_m"], "0.5")
            self.assertEqual(first_node.plan_samples[-1]["actual_x_m"], 1.3)
            self.assertEqual(first_node.plan_samples[-1]["planned_x_m"], 1.0)
            self.assertEqual(first["error_y_m"], "-0.4")
            self.assertEqual(first["error_elbow_rad"], "-0.3")
            for field in ("planned_yaw_rad", "error_yaw_rad", "error_shoulder_rad", "error_wrist_rad"):
                self.assertEqual(first[field], "")
            with gzip.open(a_dir / "telemetry.jsonl.gz", "rt", encoding="utf-8") as fp:
                event, = (r for line in fp if (r := json.loads(line))["kind"] == "tracking_error")
            self.assertEqual(event["received_unix_ns"], int(first["received_unix_ns"]))
            self.assertEqual(event["elapsed_s"], float(first["elapsed_s"]))
            self.assertIsNone(event["data"]["planned_yaw_rad"])


if __name__ == "__main__":
    unittest.main()
