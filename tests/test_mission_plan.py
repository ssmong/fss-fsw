import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mission_plan import load_table_plans, paper_plan_elapsed


class RegisteredPlanTest(unittest.TestCase):
    def setUp(self):
        try:
            import numpy as np
            import yaml
        except ImportError as exc:
            raise unittest.SkipTest(str(exc)) from exc
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        method = self.root / "trial" / "Nominal"
        method.mkdir(parents=True)
        base = np.zeros((2, 9))
        base[:, :2] = [[1, 2], [3, 4]]
        base[:, 2] = np.radians([80, 100])
        self.archive = method / "plan.npz"
        np.savez(self.archive, X_push=base, X_flight=base[::-1], frame="fss_sim_1aa9864",
                 phase_names=["push", "flight"], T=[2, 5])
        (method / "initial_setup.json").write_text(json.dumps({"base_pose": [1, 2, math.radians(80)]}))
        self.manifest = method / "trial.yaml"
        self.trial = {"plan_id": "trial-1", "plan_sha256": hashlib.sha256(self.archive.read_bytes()).hexdigest(),
                      "registration": {"rotation_rad": np.pi / 2, "translation_m": [3, 7]}}
        self.manifest.write_text(json.dumps(self.trial))

    def test_registered_body_pose(self):
        (key, plan_id, xy, times, yaw), = load_table_plans(self.root)
        self.assertEqual((key, plan_id), ("trial/Nominal", "trial-1"))
        self.assertEqual(times, [0, 2, 7])
        for actual, expected in zip(yaw, [170, 190, 170], strict=True):
            self.assertAlmostEqual(math.degrees(actual), expected)
        for actual, expected in zip(xy, [[1, 8], [-1, 10], [1, 8]], strict=True):
            for a, e in zip(actual, expected):
                self.assertAlmostEqual(a, e)

    def test_position_only_plan_retains_xy_and_time_without_synthesizing_yaw(self):
        import numpy as np
        with np.load(self.archive) as archive:
            plan = dict(archive)
        for phase in ("X_push", "X_flight"):
            plan[phase] = plan[phase][:, :2]
        np.savez(self.archive, **plan)
        self.trial["plan_sha256"] = hashlib.sha256(self.archive.read_bytes()).hexdigest()
        self.manifest.write_text(json.dumps(self.trial))
        (_, _, xy, times, yaw), = load_table_plans(self.root)
        self.assertEqual(len(xy), len(times))
        self.assertEqual(times, [0, 2, 7])
        self.assertIsNone(yaw)

    def test_changed_archive_or_missing_registration_is_excluded(self):
        with self.archive.open("ab") as f:
            f.write(b"modified")
        self.assertEqual(load_table_plans(self.root), [])
        self.trial["plan_sha256"] = hashlib.sha256(self.archive.read_bytes()).hexdigest()
        del self.trial["registration"]
        self.manifest.write_text(json.dumps(self.trial))
        self.assertEqual(load_table_plans(self.root), [])

    def test_clock_uses_last_complete_sample_of_matching_run(self):
        run = self.root / "run-1"
        run.mkdir()
        (run / "configuration.json").write_text(json.dumps({"paper_plan": {"plan_id": "A"}}))
        (run / "samples.jsonl").write_bytes(b" " * 66000 + b'\n{"plan_time_s":2.125}\n{"plan_time')
        status = {"plan_id": "A", "log_directory": str(run), "phase": "LAUNCH"}
        self.assertEqual(paper_plan_elapsed(status, self.root), 2.125)
        self.assertIsNone(paper_plan_elapsed(dict(status, phase="PREPARING"), self.root))
        self.assertIsNone(paper_plan_elapsed(dict(status, plan_id="B"), self.root))
        self.assertIsNone(paper_plan_elapsed(status, self.root / "other"))
