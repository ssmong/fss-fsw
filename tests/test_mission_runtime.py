"""Mission Control startup selection and high-level mode sequencing, without devices."""

import sys
import os
import json
import shlex
import tempfile
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mission_ros import MissionActions, ValveOutput
from mission_runtime import (
    PUBLIC, PRIVATE, PRIVATE_CPP, HardwareBusy, checkout_for, local_service_owner,
    local_stacks, native_overlay_changed, prepare_hardware, prepare_session, process_role, select_workspace, startup_failure,
)


class BodyAvailability(unittest.TestCase):
    def test_heartbeat_hides_local_or_other_launch_and_offers_retry_when_idle(self):
        import mission_ros

        ros, node = Mock(), Mock()
        events = []
        clock = iter([0, 1, 2, 3, 4, 5, 6, 7])
        now = [next(clock)]
        node.get_name.return_value = "fss_mission_test"
        node.get_node_names.side_effect = [[], [], ["gnc_node"], [], [], [], [], []]
        ros.ok.side_effect = [True] * 8 + [False]
        def advance(*_args, **_kwargs):
            mission_ros._starting_flight = None
            now[0] = next(clock, now[0])

        ros.spin_once.side_effect = advance
        processes = Mock(side_effect=[{"flight": ["external"]}, {"flight": []}])
        runtime = dict(_starting_flight=object(), prepare_hardware=Mock(), PUBLIC=PUBLIC,
                       PRIVATE_CPP=PRIVATE_CPP, PRIVATE=PRIVATE, FSS_NODES={"gnc_node"}, local_stacks=processes)
        modules = {"rclpy": ros, "rosidl_runtime_py.convert": Mock()}
        env = {"ROS_LOCALHOST_ONLY": "1", "ROS_DOMAIN_ID": "0", "FSS_MISSION_WORKSPACE": PUBLIC}
        with (patch.dict(mission_ros.__dict__, runtime), patch.dict(sys.modules, modules),
              patch.dict(os.environ, env), patch.object(mission_ros, "open_node", return_value=(node, None)),
              patch("mission_ros.time.monotonic", side_effect=lambda: now[0]),
              patch("mission_ros.select.select", return_value=([], [], [])),
              patch("builtins.print", side_effect=lambda msg, **kw: events.append(json.loads(msg)))):
            mission_ros.serve()
        self.assertEqual([value["can_start_body"] for name, value in events if name == "connected"],
                         [False, False, False, True, True, True, True, True])
        self.assertEqual(processes.call_count, 2)
        runtime["prepare_hardware"].assert_not_called()


class CalibratedImuPolicy(unittest.TestCase):
    def test_profile_uses_config_and_rejects_invalid_values(self):
        from mission_runtime import profile_uses_imu
        loader = Mock()
        with (patch.dict(sys.modules, {"fss_common.hydra_bridge": SimpleNamespace(load_hydra_config=loader)}),
              patch.dict(os.environ, {"FSS_HYDRA_OVERRIDES": "system.use_imu=false"})):
            self.assertTrue(profile_uses_imu(None))
            loader.assert_not_called()
            for value in (False, True):
                loader.return_value = SimpleNamespace(system=SimpleNamespace(use_imu=value))
                self.assertIs(profile_uses_imu(Path("base.launch.py")), value)
                self.assertEqual(loader.call_args.kwargs["overrides"], ["system.use_imu=false"])
            loader.return_value = SimpleNamespace(system=SimpleNamespace(use_imu="false"))
            with self.assertRaisesRegex(ValueError, "system.use_imu"):
                profile_uses_imu(Path("base.launch.py"))


class ValveTelemetry(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.sink = Mock()
        self.driver = SimpleNamespace(node_name="valve_driver_node", node_namespace="/", endpoint_gid=[1])
        self.gnc = SimpleNamespace(node_name="gnc_node", node_namespace="/", endpoint_gid=[2])
        self.valve = ValveOutput(self.node, self.sink)

    def test_native_requires_only_driver_as_publisher(self):
        msg = SimpleNamespace(header=None, duty_cycles=[.2] * 8)
        for pubs in ([], [self.gnc], [self.driver, self.gnc], [self.driver, self.driver]):
            self.node.get_publishers_info_by_topic.return_value = pubs
            self.valve.native(msg, "/actuators/valve_outputs")
        self.sink.assert_not_called()
        self.node.get_publishers_info_by_topic.return_value = [self.driver]
        self.valve.native(msg, "/actuators/valve_outputs")
        self.assertEqual(self.sink.call_args.args[1].duty_cycles, [.2] * 8)


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

    def test_air_on_preserves_safe_hold_estop_or_unknown_mode(self):
        for mode in (None, self.actions.SAFE_HOLD, self.actions.ESTOP):
            self.calls.clear()
            self.actions.mode = mode
            self.actions.request("valve", True)
            self.assertEqual(self.calls, [("valve", True)])
            self.accept()
            self.assertEqual(self.actions.mode, mode)

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
    def setUp(self):
        ticks = patch("mission_runtime.process_start_ticks", return_value=313)
        ticks.start()
        self.addCleanup(ticks.stop)
        guard = patch("mission_runtime.claim_installed_hardware")
        self.lease = guard.start()
        self.addCleanup(guard.stop)
        self.lease.return_value.__enter__.return_value.fileno.return_value = 43
        imu = patch("mission_runtime.profile_uses_imu", return_value=True)
        imu.start()
        self.addCleanup(imu.stop)

    @staticmethod
    def install_launch_fixture(prefix):
        launch = prefix / "share/fss_bringup/launch"
        launch.mkdir(parents=True, exist_ok=True)
        repo = Path(__file__).resolve().parents[1]
        if (repo / "runtime/cpp/base.launch.py").is_file():
            repo = repo / "runtime/cpp/source"
        source = repo / "src/infra/fss_bringup/launch"
        for name in ("fss_full.launch.py", "gnc.launch.py"):
            if source.is_dir():
                (launch / name).write_bytes((source / name).read_bytes())
            else:  # Private clone before its native submodule is initialized.
                (launch / name).write_text('LaunchConfiguration("require_explicit_target")\n'
                                           'OpaqueFunction(function=claim_launch, condition=UnlessCondition(use_sim))')

    def test_portable_overlay_selection_from_installs_and_live_processes(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            proc = home / "proc"
            proc.mkdir()
            (home / PRIVATE / "install").mkdir(parents=True)
            (home / PRIVATE / "install/setup.bash").touch()
            with self.assertRaisesRegex(RuntimeError, "scripts/cpp_fsw.sh build"):
                select_workspace(PRIVATE, home, proc)
            with self.assertRaisesRegex(RuntimeError, r"C\+\+ FSW"):
                select_workspace(PRIVATE, home, proc, monitor_only=True)
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
                self.assertEqual(select_workspace(PUBLIC, home, proc), PUBLIC if active == PRIVATE else active)
                self.assertEqual(select_workspace(PRIVATE, home, proc), PRIVATE_CPP if active == PRIVATE else active)
            with patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PRIVATE}):
                with patch("mission_runtime.local_stacks", side_effect=lambda **options: local_stacks(proc, **options)), patch("mission_runtime.prepare_hardware") as start:
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

    def test_native_private_root_is_preferred_but_toss_launch_still_blocks_body_control(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(Path, "home", return_value=Path(tmp)):
            home = Path(tmp)
            proc = home / "proc"
            proc.mkdir()
            for workspace in (PRIVATE, PRIVATE_CPP, PUBLIC):
                install = home / workspace / "install"
                binary = install / "fss_gnc/lib/fss_gnc/gnc_node"
                binary.parent.mkdir(parents=True)
                binary.write_bytes(b"\x7fELF")
                (install / "local_setup.bash").touch()
            self.assertEqual(select_workspace(PRIVATE, home, proc), PRIVATE)
            self.assertEqual(select_workspace(PUBLIC, home, proc), PUBLIC)
            node = [(home / PRIVATE / "install/fss_gnc/lib/fss_gnc/gnc_node").as_posix()]
            self.assertEqual(process_role(node, PRIVATE), "flight")
            self.assertEqual(process_role(node, PUBLIC), "competing")
            env = {"AMENT_PREFIX_PATH": (home / PRIVATE / "install/fss_bringup").as_posix()}
            launch = ["ros2", "launch", "fss_bringup", "fss_full.launch.py",
                      "use_arm:=true", "use_thrusters:=false", "use_imu:=true"]
            self.assertEqual(process_role(launch, PRIVATE, env), "competing")
            with patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PRIVATE}), patch(
                    "mission_runtime.device_processes", return_value=[(launch, env)]):
                self.assertIn("모니터링", prepare_session()[1])
                self.assertFalse(local_service_owner("manager_node"))
            with patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PUBLIC}), patch(
                    "mission_runtime.select_workspace", return_value=PRIVATE):
                self.assertTrue(native_overlay_changed(str(HardwareBusy({}, PRIVATE))))

    def test_native_private_root_vicon_only_start_skips_vendor_imu(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(Path, "home", return_value=Path(tmp)):
            home = Path(tmp)
            install = home / PRIVATE / "install"
            binary = install / "fss_gnc/lib/fss_gnc/gnc_node"
            binary.parent.mkdir(parents=True)
            binary.write_bytes(b"\x7fELF")
            self.install_launch_fixture(install)
            marker = install / "share/ament_index/resource_index/fss_mission_control/fss_gnc"
            marker.parent.mkdir(parents=True)
            marker.write_text("require_explicit_target")
            profile = home / PRIVATE_CPP / "base.launch.py"
            profile.parent.mkdir(parents=True)
            profile.write_text('LaunchConfiguration("require_explicit_target")')
            vendor = home / "xsens_mti_ros2_driver/install/setup.bash"
            vendor.parent.mkdir(parents=True)
            vendor.touch()
            with (patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PRIVATE}),
                  patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                  patch("mission_runtime.local_stacks", return_value={"flight": [], "imu": [], "monitor": {}}),
                  patch("mission_runtime.profile_uses_imu", return_value=False),
                  patch("mission_runtime.time.sleep"), patch("mission_runtime.subprocess.Popen") as spawn):
                spawn.return_value.poll.return_value = None
                self.assertEqual(prepare_session(start_if_idle=True), ("기체 소프트웨어 기동", ""))
                command = spawn.call_args.args[0]
                self.assertNotIn("--imu-script", command)
                self.assertEqual(spawn.call_args.kwargs["cwd"], home / PRIVATE)
                self.assertIn(str(profile), command)

    def test_nested_native_bringup_uses_pinned_source_and_reuses_isolated_vendor_imu(self):
        for existing_imu in ([], [{"ROS_LOCALHOST_ONLY": "1"}], [{}]):
            with self.subTest(existing_imu=existing_imu), tempfile.TemporaryDirectory() as tmp:
                home = Path(tmp)
                work = home / PRIVATE_CPP / "source"
                work.mkdir(parents=True)
                (work.parent / "base.launch.py").write_text('LaunchConfiguration("require_explicit_target")')
                self.install_launch_fixture(work.parent / "install")
                marker = work.parent / "install/share/ament_index/resource_index/fss_mission_control/fss_gnc"
                marker.parent.mkdir(parents=True)
                marker.write_text("require_explicit_target")
                vendor = home / "xsens_mti_ros2_driver/install/setup.bash"
                vendor.parent.mkdir(parents=True)
                vendor.touch()
                with (patch.object(Path, "home", return_value=home),
                      patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PRIVATE_CPP}),
                      patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                      patch("mission_runtime.local_stacks", return_value={"flight": [], "imu": existing_imu}),
                      patch("mission_runtime.time.sleep"), patch("mission_runtime.subprocess.Popen") as spawn):
                    spawn.return_value.poll.return_value = None
                    if existing_imu == [{}]:
                        with self.assertRaisesRegex(HardwareBusy, "공유 ROS 네트워크의 IMU"):
                            prepare_hardware()
                        spawn.assert_not_called()
                    else:
                        prepare_hardware()
                        commands = [call.args[0] for call in spawn.call_args_list]
                        self.assertEqual(len(commands), 1)
                        command = commands[0]
                        self.assertEqual(command[:3], ["python3", "-m", "fss_common.flight_runner"])
                        self.assertEqual(command[command.index("--") + 1:],
                                         ["ros2", "launch", str(work.parent / "base.launch.py"),
                                          "use_sim:=false", "require_explicit_target:=true"])
                        self.assertEqual("--imu-script" in command, not existing_imu)
                        self.assertTrue(all(call.kwargs["cwd"] == work for call in spawn.call_args_list))

    def test_only_normal_auto_domain_native_connection_starts_on_idle_device(self):
        for workspace in (PUBLIC, PRIVATE_CPP):
            for monitor in (False, True):
                for domain in (None, 23):
                    with (self.subTest(workspace=workspace, monitor=monitor, domain=domain),
                          patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": workspace, "ROS_DOMAIN_ID": "4",
                                                  "ROS_LOCALHOST_ONLY": "0"}),
                          patch("mission_runtime.device_processes", return_value=[]),
                          patch("mission_runtime.prepare_hardware") as hardware,
                          patch("mission_runtime.subprocess.Popen") as spawn):
                        _, reason = prepare_session(monitor_only=monitor, domain=domain, start_if_idle=True)
                        self.assertEqual(bool(reason), monitor)
                        expected_domain = str(domain) if domain is not None else ("4" if monitor else "0")
                        self.assertEqual(os.environ["ROS_DOMAIN_ID"], expected_domain)
                        self.assertEqual(os.environ["ROS_LOCALHOST_ONLY"], "0" if monitor else "1")
                        if not monitor and domain is None:
                            hardware.assert_called_once_with(only_if_idle=True)
                        else:
                            hardware.assert_not_called()
                        spawn.assert_not_called()

    def test_auto_start_requires_known_idle_owner_and_preserves_retry_after_error(self):
        with (patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PUBLIC}),
              patch("mission_runtime.private_base", return_value=None), patch("mission_runtime.device_processes") as scan,
              patch("mission_runtime.prepare_hardware") as start):
            scan.return_value = []
            start.side_effect = OSError("fixture launch failure")
            message, reason = prepare_session(start_if_idle=True)
            self.assertIn("FSW 자동 시작 실패", message)
            self.assertIn("fixture launch failure", message)
            self.assertFalse(reason)
            start.assert_called_once()
            start.reset_mock()
            for name in ("manager_node", "xsens_mti_node", "capture_executor"):
                scan.return_value = [([f"/home/smrl/fss-fsw/install/test/lib/{name}"],
                                      {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1"})]
                status, reason = prepare_session(start_if_idle=True)
                self.assertEqual(bool(reason), name == "capture_executor")
                start.assert_not_called()

    def test_switch_overlay_only_for_single_active_alternate_native(self):
        for selected, alternate in ((PUBLIC, PRIVATE_CPP), (PRIVATE_CPP, PUBLIC)):
            with patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": selected}), patch(
                    "mission_runtime.select_workspace", return_value=alternate) as select:
                reason = str(HardwareBusy({}, alternate))
                self.assertTrue(native_overlay_changed(reason))
                select.return_value = selected
                self.assertFalse(native_overlay_changed(reason))
                for owner in ("self_tossing_hils", PRIVATE, "FSS_FSW/hardware_runtime", alternate + " · self_tossing_hils"):
                    self.assertFalse(native_overlay_changed(str(HardwareBusy({}, owner))))

    def test_ssh_workflow_selection_and_explicit_shared_bench_start(self):
        import mission_bootstrap as boot

        for shared in (True, False):
            for profile in ("auto", "monitor"):
                with patch.object(boot, "detect_toss_workflow", return_value=shared), patch.object(
                        boot, "select_workspace", return_value=PUBLIC):
                    self.assertEqual(boot.bridge_selection(profile, None, "first", PUBLIC),
                                     ("toss", PRIVATE) if shared else ("control", PUBLIC))
                    self.assertEqual(boot.bridge_selection(profile, None, "body", PUBLIC), ("control", PUBLIC))
                for transition in ("first", "passive", "body"):
                    for domain in (None, 3):
                        with patch.object(boot, "toss_bench_installed", return_value=shared), patch.object(
                                boot, "prepare_session", return_value=("ready", "")) as prepare, patch.object(
                                boot, "private_base", return_value=None):
                            ctx = boot.bridge_context(profile, domain, transition, "control")
                            expected = profile == "auto" and domain is None and transition == "body"
                            prepare.assert_called_once_with(profile == "monitor", domain, start_if_idle=expected)
                            self.assertTrue(ctx["follow_workflow"])
                with patch.object(boot, "prepare_session") as prepare, patch.object(boot, "attach_toss") as attach:
                    ctx = boot.bridge_context(profile, None, "first", "toss")
                    attach.assert_called_once_with(None)
                    prepare.assert_not_called()
                    self.assertEqual(ctx["toss_monitor"], profile == "monitor")

    def test_ssh_bootstrap_fits_exec_argument_limit(self):
        try:
            from mission_link import JetsonLink
        except ModuleNotFoundError as exc:
            if exc.name == "PySide6":
                self.skipTest("PySide6 unavailable")
            raise
        link = JetsonLink(Mock())
        link.stopped.set()
        client = MagicMock()
        link._telemetry(client, None)
        channel = client.get_transport.return_value.open_session.return_value.__enter__.return_value
        command = channel.exec_command.call_args.args[0]
        self.assertLess(len(command.encode()), 128 * 1024)
        args = shlex.split(command)
        self.assertEqual(args[:2], ["bash", "-c"])
        self.assertEqual(len(args), 3)

    def test_shared_preflight_never_starts_hardware_before_singleton_lock(self):
        import mission_bootstrap as boot
        with patch.object(boot, "toss_bench_installed", return_value=False), patch.object(
                boot, "prepare_session", return_value=("ready", "")) as prepare, patch.object(
                boot, "private_base", return_value=None):
            for transition in ("first", "body", "passive"):
                boot.bridge_context("auto", None, transition, "control", allow_start=False)
                self.assertFalse(prepare.call_args.kwargs["start_if_idle"])

    def test_mismatched_discovery_profiles_cannot_start_or_attach_for_commands(self):
        peers = [{"ROS_DOMAIN_ID": "8", "CYCLONEDDS_URI": uri} for uri in ("file:///a", "file:///b")]
        with tempfile.TemporaryDirectory() as tmp, patch.object(Path, "home", return_value=Path(tmp)):
            with (patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                  patch("mission_runtime.private_base", return_value=None),
                  patch("mission_runtime.device_processes", return_value=[
                      (["/home/smrl/fss-fsw/install/fss_manager/lib/fss_manager/manager_node"], env)
                      for env in peers]),
                  patch("mission_runtime.subprocess.Popen") as spawn):
                with self.assertRaisesRegex(RuntimeError, "ROS 환경을 구분할 수 없습니다"):
                    prepare_session()
                spawn.assert_not_called()

    def test_standalone_native_on_private_host_requires_lab_profile_for_startup_and_commands(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(Path, "home", return_value=Path(tmp)):
            home = Path(tmp)
            proc = home / "proc"
            proc.mkdir()
            with (patch.dict(os.environ, {"FSS_MISSION_PREFERRED": PRIVATE, "FSS_MISSION_WORKSPACE": PUBLIC}),
                  patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                  patch("mission_runtime.local_stacks", side_effect=lambda **options: local_stacks(proc, **options)),
                  patch("mission_runtime.time.sleep"), patch("mission_runtime.subprocess.Popen") as spawn):
                self.assertFalse(prepare_session()[1])
                with self.assertRaisesRegex(RuntimeError, "scripts/cpp_fsw.sh build"):
                    prepare_hardware()
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
              patch("mission_runtime.local_stacks", side_effect=busy),
              patch("mission_runtime.subprocess.Popen") as popen):
            _, reason = prepare_session()
            self.assertIn("모니터링", reason)
            self.assertEqual(os.environ["ROS_DOMAIN_ID"], "17")
            self.assertNotIn("ROS_LOCALHOST_ONLY", os.environ)
            popen.assert_not_called()

    def test_monitor_prefers_device_publishers_and_honors_domain_despite_competing_client(self):
        client_env = {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "0"}
        device_env = {"ROS_DOMAIN_ID": "17", "ROS_LOCALHOST_ONLY": "1"}
        processes = [
            (["ros2", "launch", "self_tossing_hils", "paper_toss.launch.py"], client_env),
            (["/home/smrl/FSS_FSW/install/fss_actuators/lib/fss_actuators/valve_driver_node"], device_env),
            (["/home/smrl/FSS_FSW/install/fss_manager/lib/fss_manager/manager_node"], device_env),
        ]
        for order in (processes, list(reversed(processes))):
            for domain, expected in ((None, device_env), (17, device_env), (0, client_env)):
                with (self.subTest(domain=domain, first=order[0][0]),
                      patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PRIVATE}),
                      patch("mission_runtime.device_processes", return_value=order),
                      patch("mission_runtime.subprocess.Popen") as spawn):
                    _, reason = prepare_session(monitor_only=True, domain=domain)
                    self.assertIn("모니터링", reason)
                    self.assertIn("self_tossing_hils", reason)
                    for key, value in expected.items():
                        self.assertEqual(os.environ[key], value)
                    spawn.assert_not_called()

    def test_monitor_refuses_equally_plausible_device_networks_until_domain_is_explicit(self):
        processes = [(["/home/smrl/fss-fsw/install/fss_manager/lib/fss_manager/manager_node"],
                      {"ROS_DOMAIN_ID": domain}) for domain in ("4", "7")]
        with (patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PUBLIC}, clear=True),
              patch("mission_runtime.private_base", return_value=None),
              patch("mission_runtime.device_processes", return_value=processes),
              patch("mission_runtime.subprocess.Popen") as spawn):
            with self.assertRaisesRegex(RuntimeError, "4, 7"):
                prepare_session(monitor_only=True)
            _, reason = prepare_session(monitor_only=True, domain=7)
            self.assertTrue(reason)
            self.assertEqual(os.environ["ROS_DOMAIN_ID"], "7")
            spawn.assert_not_called()

    def test_new_stack_starts_once_and_reconnect_attaches(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            (home / "fss-fsw").mkdir()
            self.install_launch_fixture(home / PUBLIC / "install/fss_bringup")
            marker = home / PUBLIC / "install/fss_gnc/share/ament_index/resource_index/fss_mission_control/fss_gnc"
            marker.parent.mkdir(parents=True)
            marker.write_text("require_explicit_target")
            imu = home / "xsens_mti_ros2_driver/install/setup.bash"
            imu.parent.mkdir(parents=True)
            imu.touch()
            isolated = {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1"}
            with (patch.object(Path, "home", return_value=home),
                  patch.dict(os.environ, {}),
                  patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                  patch("mission_runtime.local_stacks", side_effect=[
                      {"flight": [], "imu": [], "monitor": {}}, {"flight": [], "imu": []},
                      {"flight": [isolated], "imu": [isolated], "monitor": isolated}]),
                  patch("mission_runtime.time.sleep"),
                  patch("mission_runtime.subprocess.Popen") as popen):
                popen.return_value.poll.return_value = None
                self.assertEqual(prepare_session(start_if_idle=True), ("기체 소프트웨어 기동", ""))
                self.assertEqual(prepare_session(start_if_idle=True), ("기체 연결", ""))
                self.assertEqual([args.args[0] for args in popen.call_args_list],
                                 [["python3", "-m", "fss_common.flight_runner",
                                   "--imu-script", str(home / PUBLIC / "scripts/launch/start_imu.sh"),
                                   "--imu-log", str(home / ".ros/mission-control/imu.log"),
                                   "--", "ros2", "launch", "fss_bringup", "fss_full.launch.py",
                                   "gnc_backend:=cpp", "use_arm:=false", "require_explicit_target:=true"]])
                self.assertEqual(os.environ["ROS_LOCALHOST_ONLY"], "1")
                self.lease.assert_called_once()
                for call in popen.call_args_list:
                    self.assertEqual(call.kwargs["pass_fds"], (43,))
                    self.assertEqual(call.kwargs["env"]["FSS_HARDWARE_LEASE_FD"], "43")

    def test_failed_auto_launch_retains_manual_retry_and_rejects_old_launch_files_before_spawning(self):
        with tempfile.TemporaryDirectory() as tmp:
            home = Path(tmp)
            install = home / PUBLIC / "install"
            marker = install / "share/ament_index/resource_index/fss_mission_control/fss_gnc"
            marker.parent.mkdir(parents=True)
            marker.write_text("require_explicit_target")
            with (patch.object(Path, "home", return_value=home),
                  patch.dict(os.environ, {"FSS_MISSION_WORKSPACE": PUBLIC}),
                  patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, flock=lambda *_: None)),
                  patch("mission_runtime.device_processes", return_value=[]),
                  patch("mission_runtime.time.sleep"), patch("mission_runtime.subprocess.Popen") as spawn):
                status, reason = prepare_session(start_if_idle=True)
                self.assertIn("기체 런처에 목표 대기 설정", status)
                self.assertFalse(reason)
                spawn.assert_not_called()
                self.install_launch_fixture(install)
                legacy = home / PRIVATE / "install"
                legacy.mkdir(parents=True)
                (legacy / "setup.bash").touch()
                profile = home / PRIVATE_CPP / "base.launch.py"
                profile.parent.mkdir(parents=True)
                profile.write_text('LaunchConfiguration("require_explicit_target")')
                status, reason = prepare_session(start_if_idle=True)
                self.assertIn("하드웨어 잠금 런처", status)
                self.assertFalse(reason)
                spawn.assert_not_called()
                self.lease.assert_not_called()
                self.install_launch_fixture(legacy)
                spawn.return_value.poll.return_value = 2
                status, reason = prepare_session(start_if_idle=True)
                self.assertIn("flight.log", status)
                self.assertFalse(reason)
                spawn.assert_called_once()
                status, reason = prepare_session(start_if_idle=True)
                self.assertIn("최근 기동 요청", status)
                self.assertFalse(reason)
                spawn.assert_called_once()
                spawn.return_value.poll.return_value = None
                self.assertEqual(prepare_hardware(), "기체 소프트웨어 기동")
                self.assertEqual(spawn.call_count, 2)
                self.assertEqual(startup_failure([]), "")
                spawn.return_value.poll.return_value = 3
                self.assertIn("시작 중 종료 (3)", startup_failure([]))
                self.assertEqual(startup_failure([]), "")

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
