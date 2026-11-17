"""Exercise the hardware lifetime contract using real Linux flock on temporary files."""

import os
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import Mock, patch

COMMON = Path(__file__).resolve().parents[1] / "src/infra/fss_common"
sys.path.insert(0, str(COMMON))
from fss_common.hardware_ownership import HARDWARE_FD, claim_hardware


@unittest.skipUnless(sys.platform.startswith("linux"), "Jetson lifetime locks use Linux flock")
class HardwareOwnership(unittest.TestCase):
    def setUp(self):
        folder = tempfile.TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        self.lock = Path(folder.name) / "hardware.lock"

    def test_foreign_resource_prevents_claim_and_next_attempt_can_retry(self):
        with self.assertRaisesRegex(RuntimeError, "Vicon"):
            claim_hardware(self.lock, lambda: "Vicon UDP :9870 (PID 42)")
        with claim_hardware(self.lock, lambda: None):
            with self.assertRaisesRegex(RuntimeError, "다른 FSS"):
                claim_hardware(self.lock, Mock(side_effect=AssertionError))

    def test_unrelated_descriptor_on_same_inode_cannot_authorize_a_second_launch(self):
        with claim_hardware(self.lock, lambda: None), self.lock.open("rb") as unrelated:
            with patch.dict(os.environ, {HARDWARE_FD: str(unrelated.fileno())}):
                with self.assertRaisesRegex(RuntimeError, "다른 FSS"):
                    claim_hardware(self.lock, Mock(side_effect=AssertionError))

    def test_detached_child_retains_parent_lease_after_ssh_disconnect(self):
        code = ("import sys; from pathlib import Path; from fss_common.hardware_ownership import claim_hardware; "
                "lease = claim_hardware(Path(sys.argv[1]), lambda: 'already-open Xsens'); "
                "print('ready', flush=True); sys.stdin.readline()")
        with claim_hardware(self.lock, lambda: None) as parent:
            fd = parent.fileno()
            child = subprocess.Popen([sys.executable, "-c", code, str(self.lock)], pass_fds=(fd,),
                                     env={**os.environ, "PYTHONPATH": str(COMMON), HARDWARE_FD: str(fd)},
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True,
                                     start_new_session=True)
        with child:
            try:
                self.assertEqual(child.stdout.readline().strip(), "ready")
                with self.assertRaisesRegex(RuntimeError, "다른 FSS"):
                    claim_hardware(self.lock, Mock(side_effect=AssertionError))
            finally:
                child.stdin.write("\n")
                child.stdin.flush()
        self.assertEqual(child.returncode, 0)
        with claim_hardware(self.lock, lambda: None):
            pass

    def test_full_launch_checks_physical_lease_before_children_and_skips_simulation(self):
        try:
            from launch import LaunchContext
            from launch.actions import DeclareLaunchArgument, OpaqueFunction
        except ModuleNotFoundError:
            self.skipTest("ROS Humble unavailable")
        file = COMMON.parent / "fss_bringup/launch/fss_full.launch.py"
        spec = importlib.util.spec_from_file_location("tested_full_launch", file)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        first_action = next(action for action in module.generate_launch_description().entities
                            if not isinstance(action, DeclareLaunchArgument))
        self.assertIsInstance(first_action, OpaqueFunction)
        context = LaunchContext()
        with patch("fss_common.hardware_ownership.claim_hardware", side_effect=RuntimeError("fixture occupied")) as claim:
            context.launch_configurations["use_sim"] = "true"
            first_action.visit(context)
            claim.assert_not_called()
            context.launch_configurations["use_sim"] = "false"
            with self.assertRaisesRegex(RuntimeError, "fixture occupied"):
                first_action.visit(context)
            claim.assert_called_once()


if __name__ == "__main__":
    unittest.main()
