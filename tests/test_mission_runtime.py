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
from mission_runtime import (
    PUBLIC, PRIVATE, PRIVATE_CPP, HardwareBusy, checkout_for, local_service_owner,
    local_stacks, prepare_hardware, prepare_session, process_role, select_workspace,
)


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
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            proc = home / "proc"
            proc.mkdir()
            (home / PRIVATE / "install").mkdir(parents=True)
            (home / PRIVATE / "install/setup.bash").touch()
            with self.assertRaisesRegex(RuntimeError, "scripts/cpp_fsw.sh build"):
                select_workspace(PRIVATE, home, proc)
            self.assertEqual(select_workspace(PRIVATE, home, proc, monitor_only=True), PRIVATE)
            for workspace in (PUBLIC, PRIVATE_CPP):
                install = home / workspace / "install"
                libdir = install / ("lib" if workspace == PRIVATE_CPP else "fss_gnc/lib") / "fss_gnc"
                libdir.mkdir(parents=True)
                (install / "local_setup.bash").touch()
                gnc = libdir / "gnc_node"
                gnc.write_bytes(b"#!/usr/bin/python3\n")
                if workspace == PUBLIC:
                    with self.assertRaisesRegex(RuntimeError, "colcon build"):
                        select_workspace(PUBLIC, home, proc)
                gnc.write_bytes(b"\x7fELF")
                self.assertEqual(select_workspace(PRIVATE, home, proc), workspace)
            self.assertEqual(select_workspace(PUBLIC, home, proc), PUBLIC)
            self.assertEqual(select_workspace(PRIVATE, home, proc), PRIVATE_CPP)
            child = proc / "99"
            child.mkdir()
            (child / "environ").write_bytes(b"ROS_DOMAIN_ID=31\x00")
            for active in (PUBLIC, PRIVATE_CPP, PRIVATE):
                (child / "cmdline").write_bytes(f"/home/smrl/{active}/install/fss_gnc/lib/fss_gnc/gnc_node\0".encode())
                self.assertEqual(select_workspace(PUBLIC, home, proc), active)
                self.assertEqual(select_workspace(PRIVATE, home, proc), active)
            with patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PRIVATE}):
                with patch("mission_runtime.local_stacks", side_effect=lambda: local_stacks(proc)), patch("mission_runtime.prepare_hardware") as start:
                    self.assertTrue(prepare_session()[1])
                    start.assert_not_called()

    def test_process_owners_and_overlay_launchers(self):
        self.assertEqual(process_role(["/home/smrl/FSS_FSW/install/hardware_runtime/lib/hardware_runtime/arm_driver"]), "competing")
        node = "/home/smrl/FSS_FSW/runtime/cpp/install/fss_gnc/lib/fss_gnc/gnc_node"
        self.assertEqual(process_role([node], PRIVATE_CPP), "flight")
        self.assertEqual(process_role([node], PUBLIC), "competing")
        self.assertEqual(process_role(["python3", node + ".py"], PRIVATE_CPP), "competing")
        self.assertEqual(process_role(["ros2", "launch", "self_tossing_hils", "paper_toss.launch.py"]), "competing")
        self.assertEqual(process_role(["python3", "/home/smrl/self_tossing_hardware/scripts/run_arm_reference_ros.py"]), "competing")
        self.assertIsNone(process_role(["python3", "/home/smrl/self_tossing_hardware/scripts/plot_arm_tracking.py"]))
        launch = ["python3", "/opt/ros/humble/bin/ros2", "launch", "fss_bringup", "fss_full.launch.py"]
        env = {"AMENT_PREFIX_PATH": "/home/smrl/FSS_FSW/runtime/cpp/install/fss_bringup:/opt/ros/humble"}
        self.assertEqual(checkout_for(launch, env), PRIVATE_CPP)
        self.assertEqual(process_role(launch, PRIVATE_CPP, env), "flight")
        arm = ["ros2", "launch", "/home/smrl/FSS_FSW/runtime/cpp/arm.launch.py", "use_sim:=false"]
        self.assertEqual(checkout_for(arm, {}), PRIVATE_CPP)
        self.assertEqual(process_role(arm, PRIVATE_CPP, {}), "competing")
        base = ["ros2", "launch", "/home/smrl/FSS_FSW/runtime/cpp/base.launch.py", "use_sim:=false"]
        self.assertEqual(process_role(base, PRIVATE_CPP, env), "flight")
        self.assertEqual(checkout_for(base, {"AMENT_PREFIX_PATH": "/home/smrl/fss-fsw/install/fss_gnc"}), PUBLIC)

    def test_nested_native_bringup_uses_pinned_source_and_reuses_isolated_vendor_imu(self):
        for existing_imu in ([], [{"ROS_LOCALHOST_ONLY": "1"}], [{}]):
            with self.subTest(existing_imu=existing_imu), tempfile.TemporaryDirectory() as tmp:
                home = Path(tmp)
                work = home / PRIVATE_CPP / "source"
                work.mkdir(parents=True)
                (work.parent / "base.launch.py").touch()
                vendor = home / "xsens_mti_ros2_driver/install/setup.bash"
                vendor.parent.mkdir(parents=True)
                vendor.touch()
                with (patch.object(Path, "home", return_value=home),
                      patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PRIVATE_CPP}),
                      patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                      patch("mission_runtime.local_stacks", return_value={"flight": [], "imu": existing_imu}),
                      patch("mission_runtime.time.sleep"), patch("mission_runtime.subprocess.Popen") as spawn):
                    spawn.return_value.poll.return_value = None
                    _, reason = prepare_session()
                    if existing_imu == [{}]:
                        self.assertTrue(reason)
                        spawn.assert_not_called()
                    else:
                        self.assertFalse(reason)
                        commands = [call.args[0] for call in spawn.call_args_list]
                        self.assertEqual(commands[-1], ["ros2", "launch", str(work.parent / "base.launch.py"), "use_sim:=false"])
                        self.assertEqual(commands[:-1], [] if existing_imu else [["bash", "scripts/launch/start_imu.sh"]])
                        self.assertTrue(all(call.kwargs["cwd"] == work for call in spawn.call_args_list))

    def test_explicit_monitor_on_an_unused_host_never_starts_flight_or_imu(self):
        with (patch("mission_runtime.local_stacks", return_value={"flight": [], "imu": []}),
              patch("mission_runtime.prepare_hardware") as hardware,
              patch("mission_runtime.subprocess.Popen") as spawn):
            status, reason = prepare_session(monitor_only=True)
            self.assertIn("모니터링", status)
            self.assertTrue(reason)
            hardware.assert_not_called()
            spawn.assert_not_called()

    def test_mismatched_discovery_profiles_cannot_start_or_attach_for_commands(self):
        peers = [{"ROS_DOMAIN_ID": "8", "CYCLONEDDS_URI": uri} for uri in ("file:///a", "file:///b")]
        with tempfile.TemporaryDirectory() as tmp, patch.object(Path, "home", return_value=Path(tmp)):
            with (patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                  patch("mission_runtime.local_stacks", return_value={"flight": peers, "imu": []}),
                  patch("mission_runtime.subprocess.Popen") as spawn):
                with self.assertRaisesRegex(RuntimeError, "서로 다른 ROS 환경"):
                    prepare_session()
                spawn.assert_not_called()

    def test_standalone_native_on_private_host_requires_lab_profile_for_startup_and_commands(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(Path, "home", return_value=Path(tmp)):
            home = Path(tmp)
            proc = home / "proc"
            proc.mkdir()
            with (patch.dict(os.environ, {"FSS_MISSION_PREFERRED": PRIVATE, "FSS_MISSION_WORKSPACE": PUBLIC}),
                  patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                  patch("mission_runtime.local_stacks", side_effect=lambda: local_stacks(proc)),
                  patch("mission_runtime.time.sleep"), patch("mission_runtime.subprocess.Popen") as spawn):
                with self.assertRaisesRegex(RuntimeError, "scripts/cpp_fsw.sh build"):
                    prepare_session()
                spawn.assert_not_called()
                child = proc / "41"
                child.mkdir()
                (child / "cmdline").write_bytes(b"/home/smrl/fss-fsw/install/fss_manager/lib/fss_manager/manager_node\0")
                (child / "environ").write_bytes(b"ROS_DOMAIN_ID=0\0")
                self.assertTrue(prepare_session()[1])
                self.assertFalse(local_service_owner("manager_node", proc))
                spawn.assert_not_called()
                profile = home / PRIVATE_CPP / "base.launch.py"
                profile.parent.mkdir(parents=True)
                profile.touch()
                launch = proc / "42"
                launch.mkdir()
                (launch / "cmdline").write_bytes(f"ros2\0launch\0{profile.as_posix()}\0use_sim:=false\0".encode())
                (launch / "environ").write_bytes(b"AMENT_PREFIX_PATH=/home/smrl/fss-fsw/install\0")
                self.assertTrue(local_service_owner("manager_node", proc))
                self.assertFalse(prepare_session()[1])
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
                                  ["ros2", "launch", "fss_bringup", "fss_full.launch.py", "gnc_backend:=cpp"]])
                self.assertEqual(os.environ["ROS_LOCALHOST_ONLY"], "1")

    def test_match_launch_argv_not_python_source_or_shell_text(self):
        self.assertIsNone(process_role(["python3", "-c", "ros2 launch self_tossing_hils"]))
        self.assertEqual(process_role(["/usr/bin/python3", "/opt/ros/humble/bin/ros2", "launch",
                                       "self_tossing_hils", "paper_toss.launch.py"]), "competing")
        self.assertEqual(process_role(["/work/install/fss_gnc/lib/fss_gnc/gnc_node"]), "competing")

    def test_local_environment_and_competing_experiment(self):
        with tempfile.TemporaryDirectory() as tmp:
            process = Path(tmp) / "140"
            process.mkdir()
            (process / "cmdline").write_bytes(b"/home/smrl/fss-fsw/install/fss_manager/lib/fss_manager/manager_node\0--ros-args\0")
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
