"""Mission Control startup selection and high-level mode sequencing, without devices."""

import sys
import os
import tempfile
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mission_ros import MissionActions
from mission_runtime import HardwareBusy, local_service_owner, local_stacks, prepare_hardware, prepare_session, process_role


class FlightCommands(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.replies = []
        self.actions = MissionActions(self.call, self.replies.append)
        self.actions.observe("mode", SimpleNamespace(mode=0))

    def call(self, name, value):
        self.calls.append((name, value))
        self.f = Future()
        return self.f

    def accept(self):
        op, value = self.calls[-1]
        self.f.set_result(SimpleNamespace(success=True, actual_mode=value, message="ok"))
        self.actions.tick()

    def test_air_on_waits_for_manager_confirmation_before_ready(self):
        self.actions.request("valve", True)
        self.assertEqual(self.calls, [("mode", 1)])
        self.accept()
        self.assertEqual(self.calls[-1], ("valve", True))
        self.accept()
        self.actions.tick()
        self.assertEqual(len(self.calls), 2)
        self.actions.observe("health", SimpleNamespace(main_valve_open=True))
        self.actions.tick()
        self.assertEqual(self.calls[-1], ("mode", 2))
        self.accept()
        self.assertEqual(self.replies, ["에어베어링 ON"])

    def test_rejected_target_never_arms(self):
        rejected = []
        self.actions = MissionActions(self.call, self.replies.append, rejected.append)
        self.actions.observe("mode", SimpleNamespace(mode=2))
        self.actions.request("target", ["pid", 0, 0, 0])
        self.f.set_result(SimpleNamespace(success=False, message="bad target"))
        self.actions.tick()
        self.assertEqual(rejected, ["bad target"])
        self.assertEqual(self.replies, [])
        self.assertEqual(len(self.calls), 1)

    def test_target_arms_only_after_success_and_off_first_holds(self):
        self.actions.observe("mode", SimpleNamespace(mode=2))
        self.actions.request("target", ["lqr", 1., 2., .5])
        self.accept()
        self.assertEqual(self.calls[-1], ("mode", 3))
        self.accept()
        self.actions.request("valve", False)
        self.assertEqual(self.calls[-1], ("mode", 4))
        self.accept()
        self.assertEqual(self.calls[-1], ("valve", False))
        self.accept()
        self.assertEqual(self.calls[-1], ("mode", 0))

    def test_estop_cancels_pending_arm(self):
        self.actions.observe("mode", SimpleNamespace(mode=2))
        self.actions.request("target", ["pid", 0., 0., 0.])
        target_response = self.f
        self.actions.request("mode", 5)
        target_response.set_result(SimpleNamespace(success=True, message="ok"))
        self.accept()
        self.assertEqual([op for op in self.calls if op == ("mode", 3)], [])


class Discovery(unittest.TestCase):
    def test_portable_overlay_selection_from_installs_and_live_processes(self):
        from mission_runtime import PRIVATE, PUBLIC, select_workspace

        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            proc = home / "proc"
            proc.mkdir()
            (home / PRIVATE / "install").mkdir(parents=True)
            (home / PRIVATE / "install/setup.bash").touch()
            self.assertEqual(select_workspace(PUBLIC, home, proc), PRIVATE)
            (home / PUBLIC / "install").mkdir(parents=True)
            (home / PUBLIC / "install/setup.bash").touch()
            self.assertEqual(select_workspace(PUBLIC, home, proc), PUBLIC)
            self.assertEqual(select_workspace(PRIVATE, home, proc), PRIVATE)
            child = proc / "99"
            child.mkdir()
            (child / "cmdline").write_bytes(b"/home/smrl/FSS_FSW/install/fss_gnc/lib/fss_gnc/gnc_node\x00")
            (child / "environ").write_bytes(b"ROS_DOMAIN_ID=31\x00")
            self.assertEqual(select_workspace(PUBLIC, home, proc), PRIVATE)
            with patch.dict(os.environ, {"FSS_MISSION_PREFERRED": PRIVATE}):
                self.assertEqual(local_stacks(proc)["flight"], [{"ROS_DOMAIN_ID": "31"}])
            with patch.dict(os.environ, {"FSS_MISSION_PREFERRED": PUBLIC, "FSS_MISSION_WORKSPACE": PRIVATE}):
                with patch("mission_runtime.local_stacks", return_value={"flight": [], "imu": []}), patch("mission_runtime.prepare_hardware") as start:
                    self.assertTrue(prepare_session()[1])
                    start.assert_not_called()

    def test_private_checkout_keeps_foreign_experiment_and_public_stack_guards(self):
        with patch.dict(os.environ, {"FSS_MISSION_PREFERRED": "FSS_FSW"}):
            self.assertEqual(process_role(["/home/smrl/FSS_FSW/install/hardware_runtime/lib/hardware_runtime/arm_driver"]), "flight")
            self.assertEqual(process_role(["/home/smrl/fss-fsw/install/fss_gnc/lib/fss_gnc/gnc_node"]), "competing")
            self.assertEqual(process_role(["ros2", "launch", "self_tossing_hils", "paper_toss.launch.py"]), "competing")
            self.assertEqual(process_role(["python3", "/home/smrl/self_tossing_hardware/scripts/run_arm_reference_ros.py"]), "competing")
            self.assertIsNone(process_role(["python3", "/home/smrl/self_tossing_hardware/scripts/plot_arm_tracking.py"]))

    def test_private_bringup_owns_vendor_imu(self):
        for existing_imu in ([], [{}]):
            with self.subTest(existing_imu=existing_imu), tempfile.TemporaryDirectory() as tmp:
                home = Path(tmp)
                (home / "FSS_FSW").mkdir()
                with (patch.object(Path, "home", return_value=home),
                      patch.dict(os.environ, {"FSS_MISSION_PREFERRED": "FSS_FSW", "FSS_MISSION_WORKSPACE": "FSS_FSW"}),
                      patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                      patch("mission_runtime.local_stacks", return_value={"flight": [], "imu": existing_imu}),
                      patch("mission_runtime.time.sleep"), patch("mission_runtime.subprocess.Popen") as spawn):
                    spawn.return_value.poll.return_value = None
                    prepare_hardware()
                    self.assertEqual(spawn.call_count, 1)
                    command = spawn.call_args.args[0]
                    self.assertEqual(command[:4], ["ros2", "launch", "fss_bringup", "fss_full.launch.py"])
                    self.assertEqual(command[4:], ["start_imu_driver:=false"] if existing_imu else [])
                    self.assertEqual(spawn.call_args.kwargs["cwd"], home / "FSS_FSW")

    def test_explicit_monitor_on_an_unused_host_never_starts_flight_or_imu(self):
        with (patch("mission_runtime.local_stacks", return_value={"flight": [], "imu": []}),
              patch("mission_runtime.prepare_hardware") as hardware,
              patch("mission_runtime.subprocess.Popen") as spawn):
            status, reason = prepare_session(monitor_only=True)
            self.assertIn("모니터링", status)
            self.assertTrue(reason)
            hardware.assert_not_called()
            spawn.assert_not_called()

    def test_legacy_stack_blocks_hardware_and_commands_without_capture_executor(self):
        with tempfile.TemporaryDirectory() as tmp:
            proc = Path(tmp)
            for pid, path in enumerate((
                    "/home/smrl/FSS_FSW/install/hardware_runtime/lib/hardware_runtime/arm_driver",
                    "/home/smrl/FSS_FSW/install/fss_actuators/lib/fss_actuators/valve_driver_node",
                    "/home/smrl/FSS_FSW/install/fss_manager/lib/fss_manager/manager_node"), 10):
                child = proc / str(pid)
                child.mkdir()
                argv = (["/usr/bin/python3"] if pid == 10 else []) + [path, "--ros-args"]
                (child / "cmdline").write_bytes("\0".join(argv).encode() + b"\0")
                (child / "environ").write_bytes(b"ROS_DOMAIN_ID=30\0")
                self.assertEqual(process_role(argv), "competing")
            with self.assertRaisesRegex(HardwareBusy, "FSS_FSW") as ctx:
                local_stacks(proc)
            self.assertEqual(ctx.exception.environment["ROS_DOMAIN_ID"], "30")
            with patch.dict(os.environ, {"ROS_DOMAIN_ID": "30"}):
                self.assertFalse(local_service_owner("manager_node", proc))
            self.assertIsNone(process_role(["python3", "-c", 'print("/home/smrl/FSS_FSW/install/hardware_runtime")']))

    def test_busy_host_attaches_to_existing_domain_without_launch(self):
        busy = HardwareBusy({"ROS_DOMAIN_ID": "17"})
        with (patch.dict(os.environ, {"ROS_LOCALHOST_ONLY": "1"}),
              patch("mission_runtime.prepare_hardware", side_effect=busy),
              patch("mission_runtime.subprocess.Popen") as popen):
            _, reason = prepare_session()
            self.assertIn("모니터링", reason)
            self.assertEqual(os.environ["ROS_DOMAIN_ID"], "17")
            self.assertNotIn("ROS_LOCALHOST_ONLY", os.environ)
            popen.assert_not_called()

    def test_new_stack_starts_once_and_reconnect_attaches(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            (home / "fss-fsw").mkdir()
            imu = home / "xsens_mti_ros2_driver/install/setup.bash"
            imu.parent.mkdir(parents=True)
            imu.touch()
            isolated = {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1"}
            with (patch.object(Path, "home", return_value=home),
                  patch.dict(os.environ, {}),
                  patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                  patch("mission_runtime.local_stacks", side_effect=[
                      {"flight": [], "imu": []}, {"flight": [isolated], "imu": [isolated]}]),
                  patch("mission_runtime.time.sleep"),
                  patch("mission_runtime.subprocess.Popen") as popen):
                popen.return_value.poll.return_value = None
                self.assertEqual(prepare_hardware(), "기체 소프트웨어 기동")
                self.assertEqual(prepare_hardware(), "기존 기체 소프트웨어 연결")
                self.assertEqual([args.args[0] for args in popen.call_args_list],
                                 [["bash", "scripts/launch/start_imu.sh"],
                                  ["ros2", "launch", "fss_bringup", "fss_full.launch.py"]])
                self.assertEqual(os.environ["ROS_LOCALHOST_ONLY"], "1")

    def test_match_launch_argv_not_python_source_or_shell_text(self):
        self.assertIsNone(process_role(["python3", "-c", "ros2 launch self_tossing_hils"]))
        self.assertEqual(process_role(["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
                                       "self_tossing_hils", "paper_toss.launch.py"]), "competing")
        self.assertEqual(process_role(["/work/install/fss_gnc/lib/fss_gnc/gnc_node"]), "flight")

    def test_local_environment_and_competing_experiment(self):
        with tempfile.TemporaryDirectory() as tmp:
            process = Path(tmp) / "140"
            process.mkdir()
            (process / "cmdline").write_bytes(b"/bin/manager_node\0--ros-args\0")
            (process / "environ").write_bytes(b"ROS_DOMAIN_ID=7\0IGNORED=secret\0")
            self.assertEqual(local_stacks(Path(tmp))["flight"], [{"ROS_DOMAIN_ID": "7"}])
            with patch.dict(os.environ, {"ROS_DOMAIN_ID": "7", "ROS_LOCALHOST_ONLY": "0"}):
                self.assertTrue(local_service_owner("manager_node", Path(tmp)))
                self.assertFalse(local_service_owner("gnc_node", Path(tmp)))
            (process / "cmdline").write_bytes(b"/bin/ros2\0launch\0self_tossing_hils\0paper_toss.launch.py\0")
            with self.assertRaisesRegex(RuntimeError, "self_tossing_hils"):
                local_stacks(Path(tmp))


if __name__ == "__main__":
    unittest.main()
