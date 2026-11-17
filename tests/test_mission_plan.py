import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mission_plan import load_table_plans, paper_plan_elapsed, PlanCatalog


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

    def test_catalog_detects_registration_and_added_deleted_plans_with_cached_reads(self):
        import shutil
        catalog = PlanCatalog(self.root, self.model)
        self.assertTrue(catalog.refresh())
        with patch("mission_plan.load_table_plans", wraps=load_table_plans) as reader:
            self.assertFalse(catalog.refresh())
            reader.assert_not_called()
            self.trial["registration"]["translation_m"][0] -= .1
            self.manifest.write_text(json.dumps(self.trial))
            self.assertTrue(catalog.refresh())
            self.assertEqual(reader.call_count, 1)
            self.assertAlmostEqual(catalog.plans[0][2][0][0], .9)
            alternate = self.manifest.parent.with_name("Proposed")
            shutil.copytree(self.manifest.parent, alternate)
            self.assertTrue(catalog.refresh())
            self.assertEqual(reader.call_count, 2)
            self.assertEqual(len(catalog.plans), 2)
            (alternate / "trial.yaml").unlink()
            self.assertTrue(catalog.refresh())
            self.assertEqual(reader.call_count, 2)
            self.assertEqual(len(catalog.plans), 1)

    def test_catalog_recovers_partial_update_and_reloads_setup_archive_and_model(self):
        catalog = PlanCatalog(self.root, self.model)
        catalog.refresh()
        original = self.archive.read_bytes()
        self.archive.write_bytes(b"partial")
        self.assertTrue(catalog.refresh())
        self.assertEqual(catalog.plans, [])
        self.archive.write_bytes(original)
        self.assertTrue(catalog.refresh())
        setup = json.loads(self.setup_path.read_text())
        del setup["joint_rad"]
        self.setup_path.write_text(json.dumps(setup))
        self.assertTrue(catalog.refresh())
        self.assertIsNone(catalog.plans[0][5])
        self.setup_path.write_text(json.dumps(dict(setup, joint_rad=[.23, -1.15, -.68])))
        self.assertTrue(catalog.refresh())
        model = json.loads(self.model.read_text())
        model["joint_offsets_m"][0][0] += .001
        self.model.write_text(json.dumps(model))
        self.assertTrue(catalog.refresh())
        self.assertAlmostEqual(catalog.plans[0][6][0][0][0], .00075)

    def test_catalog_reads_static_launch_default_and_detects_default_only_changes(self):
        import shutil
        experiments = self.root / "experiments"
        experiments.mkdir()
        shutil.copytree(self.manifest.parent.parent, experiments / "trial")
        shutil.copytree(self.manifest.parent.parent, experiments / "alternate")
        launch = self.root / "src/self_tossing_hils/launch/paper_toss.launch.py"
        launch.parent.mkdir(parents=True)
        code = (
            "def generate_launch_description():\n"
            "    root = Path(os.environ.get('SELF_TOSSING_HARDWARE_ROOT', Path(__file__).resolve().parents[3]))\n"
            "    bundle = root / 'experiments/trial'\n"
            "    return LaunchDescription([DeclareLaunchArgument('trial_file', "
            "default_value=PathJoinSubstitution([str(bundle), LaunchConfiguration('method'), 'trial.yaml']))])\n"
        )
        launch.write_text(code)
        catalog = PlanCatalog(experiments, self.model)
        self.assertTrue(catalog.refresh())
        self.assertEqual(catalog.defaults, {"Nominal": "trial/Nominal"})
        with patch("mission_plan.load_table_plans", wraps=load_table_plans) as reader:
            launch.write_text(code.replace("experiments/trial", "experiments/alternate"))
            self.assertTrue(catalog.refresh())
            reader.assert_not_called()
            self.assertEqual(catalog.defaults, {"Nominal": "alternate/Nominal"})
            launch.write_text(code.replace("root / 'experiments/trial'", "compute_bundle()"))
            self.assertTrue(catalog.refresh())
            self.assertEqual(catalog.defaults, {})
            self.assertFalse(catalog.refresh())
            launch.write_text(code.replace("experiments/trial", "../outside"))
            catalog.refresh()
            self.assertEqual(catalog.defaults, {})
            launch.write_text(code.replace("LaunchConfiguration('method')", "choose_method()"))
            catalog.refresh()
            self.assertEqual(catalog.defaults, {})
            launch.write_text(code.replace("Path(__file__).resolve().parents[3]", "different_root()"))
            catalog.refresh()
            self.assertEqual(catalog.defaults, {})

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
