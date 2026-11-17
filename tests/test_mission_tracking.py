import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mission_plan import paper_run_files
from mission_tracking import PaperTracking


class PaperTrackingTest(unittest.TestCase):
    def setUp(self):
        try:
            import numpy
        except ImportError as exc:
            raise unittest.SkipTest(str(exc)) from exc
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.run = self.root / "experiment" / "run"
        self.run.mkdir(parents=True)
        self.log = self.run / "samples.jsonl"
        self.log.touch()
        self.key = "bundle/Nominal"
        self.entry = [self.key, "A", [[0., 0.], [2., 4.]], [0., 2.],
                      [math.radians(179), math.radians(183)], [[0., 1., 0.], [1., -1., -.5]]]
        self.signature = ("sha", 0., [0., 0.])
        self.meta = {"plan_id": "A", "plan_sha256": "sha",
                     "registration": {"rotation_rad": 0., "translation_m": [0., 0.]}}
        self.status = {"plan_id": "A", "phase": "LAUNCH", "frame_id": "table", "log_directory": str(self.run)}
        self.tracker = PaperTracking([self.entry], {self.key: self.signature}, 1000.)

    def poll(self, now=2000):
        config = {"paper_plan": self.meta, "experiment": {"frame_id": "table", "feedback_timeout_s": .15}}
        (self.run / "configuration.json").write_text(json.dumps(config))
        return self.tracker.poll(self.status, now, paper_run_files(self.status, self.root))

    def row(self, stamp, t, **overrides):
        return {"sample_stamp": stamp, "control_stamp": stamp + .125, "phase": "COAST",
                "plan_time_s": t + .125, "q": [1.5, 1., math.radians(-178), .25, -.1, -.4], **overrides}

    def write(self, *rows):
        with self.log.open("a") as f:
            for row in rows:
                f.write(json.dumps(row) + "\n")

    def test_measurement_time_interpolation_and_signed_errors(self):
        self.write(self.row(1100., 1.))
        sample, = self.poll()
        self.assertEqual(sample["plan_time_s"], 1.)
        self.assertEqual(sample["planned_x_m"], 1.)
        self.assertEqual(sample["planned_y_m"], 2.)
        self.assertEqual(sample["error_x_m"], .5)
        self.assertEqual(sample["error_y_m"], -1.)
        self.assertAlmostEqual(sample["error_xy_m"], math.hypot(.5, 1))
        self.assertAlmostEqual(sample["error_yaw_rad"], math.radians(1))
        self.assertAlmostEqual(sample["error_shoulder_rad"], -.25)
        self.assertAlmostEqual(sample["error_elbow_rad"], -.1)
        self.assertAlmostEqual(sample["error_wrist_rad"], -.15)
        self.assertEqual(self.poll(), [])

    def test_catalog_refresh_preserves_bound_run_and_cursor_after_plan_update_and_deletion(self):
        self.write(self.row(1100, .5))
        first, = self.poll()
        offset = self.tracker.offset
        moved = [self.key, "A", [[-.1, 0.], [1.9, 4.]], *self.entry[3:]]
        self.tracker.refresh_catalog([moved], {self.key: ("sha", 0., [-.1, 0.])})
        self.assertEqual(self.tracker.offset, offset)
        self.assertIsNotNone(self.tracker.extra_plan)
        self.assertEqual(self.tracker.extra_plan[2], self.entry[2])
        self.write(self.row(1101, 1.))
        second, = self.poll()
        self.assertEqual(second["planned_x_m"], 1.)
        self.assertEqual(second["plan_key"], first["plan_key"])
        self.assertNotEqual(self.tracker.active_key, self.key)
        self.tracker.refresh_catalog([], {})
        self.write(self.row(1102, 1.5))
        third, = self.poll()
        self.assertEqual(third["planned_x_m"], 1.5)
        self.assertEqual(third["plan_key"], second["plan_key"])
        self.assertEqual(self.poll(), [])

    def test_connection_freshness_duplicate_out_of_order_and_postplan(self):
        good = self.row(1100, .5)
        self.write(self.row(999., .25), self.row(1005., -.1), self.row(1006., .1, control_stamp=1006.5),
                   self.row(1007., .25, phase="LOCAL_BRAKE"), good, good, self.row(1099., .51),
                   self.row(1101., .45), self.row(1102., 2.01))
        self.assertEqual([s["sample_stamp_s"] for s in self.poll()], [1100])
        self.write(self.row(1103., 2., q=[1., 2., 3.]))
        self.status["phase"] = "STOPPED"
        final, = self.poll()
        self.assertIsNone(final["error_wrist_rad"])
        self.assertEqual(final["planned_wrist_rad"], -.5)

    def test_an_old_run_reprojects_same_hash_to_its_executed_registration(self):
        self.meta["registration"] = {"rotation_rad": math.pi / 2, "translation_m": [1.9, 2.]}
        self.write(self.row(1100, 1.))
        sample, = self.poll()
        self.assertAlmostEqual(sample["planned_x_m"], -.1)
        self.assertAlmostEqual(sample["planned_y_m"], 3.)
        self.assertAlmostEqual(sample["planned_yaw_rad"], math.radians(271))
        self.assertEqual(sample["plan_key"], self.tracker.extra_plan[0])
        self.assertEqual(self.entry[2][0], [0., 0.])
        self.assertEqual(self.tracker.extra_plan[2][0], [1.9, 2.])

    def test_ambiguous_or_wrong_identity_never_uses_the_selected_preview(self):
        alias = ["alternate/Nominal", *self.entry[1:]]
        self.tracker.set_catalog([self.entry, alias], {self.key: self.signature, alias[0]: self.signature})
        self.write(self.row(1100, .5))
        self.assertEqual(self.poll(), [])
        self.assertIsNone(self.tracker.active_key)
        self.tracker.set_catalog([self.entry, alias], {self.key: self.signature, alias[0]: self.signature})
        self.meta["trial_file"] = "alternate/Nominal/trial.yaml"
        sample, = self.poll()
        self.assertEqual(sample["plan_key"], alias[0])
        self.tracker.set_catalog([self.entry], {self.key: self.signature})
        self.meta["plan_sha256"] = "changed"
        self.assertEqual(self.poll(), [])
        self.assertIsNone(self.tracker.active_key)

    def test_rejects_invalid_time_grid_and_retains_legacy_xy(self):
        self.tracker.set_catalog([[self.key, "A", self.entry[2], [1, 1]]], {self.key: self.signature})
        self.write(self.row(1100, .5))
        self.assertEqual(self.poll(), [])
        self.tracker.set_catalog([self.entry[:4]], {self.key: self.signature})
        sample, = self.poll()
        self.assertIsNone(sample["planned_yaw_rad"])
        self.assertIsNone(sample["error_shoulder_rad"])
        self.assertEqual(sample["actual_shoulder_rad"], .25)

    def test_wrapped_joint_knots_and_unwrapped_turns_interpolate_continuously(self):
        self.entry[5] = [[math.radians(179), 0, -math.pi], [math.radians(-179), math.tau, -math.pi]]
        self.tracker.set_catalog([self.entry], {self.key: self.signature})
        self.write(self.row(1100, 1., q=[0., 0., 0., -math.pi, -math.pi, math.pi]))
        sample, = self.poll()
        self.assertAlmostEqual(sample["planned_shoulder_rad"], math.pi)
        self.assertAlmostEqual(sample["planned_elbow_rad"], math.pi)
        self.assertAlmostEqual(sample["error_shoulder_rad"], 0.)
        self.assertAlmostEqual(sample["error_elbow_rad"], 0.)
        self.assertAlmostEqual(sample["error_wrist_rad"], 0.)

    def test_bounded_batches_drain_forward_and_partial_row_waits(self):
        self.write(*(self.row(1100 + i * .005, i * .005) for i in range(275)))
        last = self.row(1102, 1.6)
        partial = json.dumps(last)
        with self.log.open("a") as f:
            f.write(partial[:40])
        output = self.poll() + self.poll() + self.poll()
        self.assertEqual(len(output), 275)
        with self.log.open("a") as f:
            f.write(partial[40:] + "\n")
        self.assertEqual([s["sample_stamp_s"] for s in self.poll()], [1102])

    def test_connecting_to_completed_log_replays_no_old_data(self):
        self.status["phase"] = "STOPPED"
        self.write(*(self.row(900 + i * .001, i * .001) for i in range(650)))
        self.assertEqual(self.poll(), [])
        self.assertEqual(self.poll(), [])
        self.assertEqual(self.tracker.active_key, self.key)
