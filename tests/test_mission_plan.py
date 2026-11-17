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
        base[:, 3:6] = [[0.23, -1.15, -0.68], [0.42, -1.1, -0.72]]
        self.archive = method / "plan.npz"
        np.savez(self.archive, X_push=base, X_flight=base[::-1], frame="fss_sim_1aa9864",
                 phase_names=["push", "flight"], T=[2, 5], grasp_offset_m=[0, -0.1157])
        setup = {"base_pose": [1, 2, math.radians(80)], "joint_rad": base[0, 3:6].tolist()}
        self.setup_path = method / "initial_setup.json"
        self.setup_path.write_text(json.dumps(setup))
        self.manifest = method / "trial.yaml"
        self.trial = {"plan_id": "trial-1", "plan_sha256": hashlib.sha256(self.archive.read_bytes()).hexdigest(),
                      "expected_grasp_offset_m": [0, -0.1157],
                      "registration": {"rotation_rad": np.pi / 2, "translation_m": [3, 7]}}
        self.manifest.write_text(json.dumps(self.trial))
        self.offsets = [[-0.00025, -0.19025], [0, -0.199021], [0, -0.199021]]
        self.model = self.root / "planar_model.json"
        self.model.write_text(json.dumps({"schema_version": 1, "joint_offsets_m": self.offsets,
            "grasp_offset_m": [0, -0.107], "provenance": {
                "urdf_commit": "1aa9864b55c8a3032085244ab4e365785faf354c",
                "urdf_sha256": "23ad70e02d040ea482776261baaae14fbf1de2fc1da3770742b2a54bbab449f3"}}))

    def test_registered_body_pose(self):
        (key, plan_id, xy, times, yaw, joints, geometry), = load_table_plans(self.root, self.model)
        self.assertEqual((key, plan_id), ("trial/Nominal", "trial-1"))
        self.assertEqual(times, [0, 2, 7])
        for actual, expected in zip(yaw, [170, 190, 170], strict=True):
            self.assertAlmostEqual(math.degrees(actual), expected)
        for actual, expected in zip(xy, [[1, 8], [-1, 10], [1, 8]], strict=True):
            for a, e in zip(actual, expected):
                self.assertAlmostEqual(a, e)
        self.assertEqual(joints, [[0.23, -1.15, -0.68], [0.42, -1.1, -0.72], [0.23, -1.15, -0.68]])
        self.assertEqual(geometry, [self.offsets, [0, -0.1157]])

    def test_unknown_joint_initial_condition_preserves_body_pose(self):
        setup = json.loads(self.setup_path.read_text())
        setup["joint_rad"][1] = "unknown"
        self.setup_path.write_text(json.dumps(setup))
        (_, _, xy, _, yaw, joints, geometry), = load_table_plans(self.root, self.model)
        self.assertEqual(len(xy), len(yaw))
        self.assertIsNone(joints)
        self.assertIsNone(geometry)

    def test_geometry_checks_the_model_source_and_effective_grasp(self):
        self.trial["expected_grasp_offset_m"][1] = -0.067
        self.manifest.write_text(json.dumps(self.trial))
        plan, = load_table_plans(self.root, self.model)
        self.assertEqual(plan[-1], [self.offsets, None])
        data = json.loads(self.model.read_text())
        data["provenance"]["urdf_commit"] = "other"
        self.model.write_text(json.dumps(data))
        plan, = load_table_plans(self.root, self.model)
        self.assertIsNotNone(plan[-2])
        self.assertIsNone(plan[-1])

    def test_position_only_plan_retains_xy_and_time_without_synthesizing_yaw(self):
        import numpy as np
        with np.load(self.archive) as archive:
            plan = dict(archive)
        for phase in ("X_push", "X_flight"):
            plan[phase] = plan[phase][:, :2]
        np.savez(self.archive, **plan)
        self.trial["plan_sha256"] = hashlib.sha256(self.archive.read_bytes()).hexdigest()
        self.manifest.write_text(json.dumps(self.trial))
        (_, _, xy, times, yaw, joints, geometry), = load_table_plans(self.root, self.model)
        self.assertEqual(len(xy), len(times))
        self.assertEqual(times, [0, 2, 7])
        self.assertIsNone(yaw)
        self.assertIsNone(joints)
        self.assertIsNone(geometry)

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
