"""Self-tossing adapter exercised with an isolated process tree and stub launches."""

import json
import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
import mission_toss as toss


class TossTests(unittest.TestCase):
    def setUp(self):
        files = tempfile.TemporaryDirectory()
        self.addCleanup(files.cleanup)
        self.home = Path(files.name)
        self.proc = self.home / "proc"
        self.proc.mkdir()
        self.b = toss.TossBench(self.home, self.proc)
        self.b.state.mkdir(parents=True)
        (self.home / ".bash_aliases").touch()
        self.procs = []
        self.names = []
        for item in (patch.object(Path, "home", return_value=self.home),
                     patch.dict(os.environ, {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "0"}),
                     patch.object(toss, "device_processes", side_effect=lambda *a, **kw: list(self.procs)),
                     patch.object(toss, "guarded_full_launch", return_value=True)):
            item.start()
            self.addCleanup(item.stop)

    def add(self, pid, parent, sid, args, ticks=313, env=None):
        proc = self.proc / str(pid)
        proc.mkdir()
        (proc / "stat").write_text(f"{pid} (some ) cmd) S {parent} {sid} {sid} " + "0 " * 15 + str(ticks))
        self.procs.append((pid, args, env if env is not None else dict(os.environ,
                           AMENT_PREFIX_PATH="/lab/FSS_FSW/install/fss_bringup")))

    def flight(self):
        self.add(10, 1, 10, ["ros2", "launch", "fss_bringup", "fss_full.launch.py", "use_arm:=true",
                             "use_thrusters:=false", "use_imu:=true", "use_realsense:=false", "auto_activate:=true",
                             "toss_nav_uncertainty_grace:=true",
                             f"arm_config:={self.home / 'FSS_FSW'}/src/hardware/hardware_runtime/config/arm.yaml"])
        self.add(11, 10, 10, ["/lab/FSS_FSW/install/hardware_runtime/lib/hardware_runtime/arm_driver"])
        self.add(12, 10, 10, ["/lab/FSS_FSW/install/fss_manager/lib/fss_manager/manager_node"])
        self.names.extend(("arm_driver_node", "manager_node"))

    def paper(self, owned=False, args=None):
        method, *options = ["Nominal", "tracking_mode:=velocity"] if args is None else args
        self.add(20, 1, 20, ["ros2", "launch", "self_tossing_hils", "paper_toss.launch.py", "method:=" + method,
                             "execute:=true", *options])
        self.add(21, 20, 20, ["/lab/self_tossing_hardware/install_client/self_tossing_hils/lib/self_tossing_hils/capture_executor"])
        self.names.append("paper_toss_executor")
        self.b.observe({"phase": "IDLE"})
        if owned:
            (self.b.state / "toss_run.json").write_text(json.dumps([20, 313, method if args is None else args]))

    def test_method_only_and_future_launch_arguments(self):
        self.assertEqual(toss.toss_run_args("Nominal"), ["Nominal"])
        args = ["Nominal", "tracking_mode:=position", "trial_file:=/lab/plan A/trial.yaml", "future_flag:=a;b"]
        self.assertEqual(toss.toss_run_args(args), args)
        for option in ("method:=Nominal", "execute:=true", "missing-value", "name:=", "--flag:=v",
                       "bad\x00name:=x", "session:=a\nb", 42, "tracking_mode:=velocity"):
            with self.subTest(option=option), self.assertRaises(ValueError):
                toss.toss_run_args([*args, option])
        for value in (["Unknown"], [], {}, None, "Nominal mode:=velocity"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                toss.toss_run_args(value)

    @unittest.skipUnless(importlib.util.find_spec("omegaconf"), "Jetson tuning dependency")
    def test_gain_snapshot_preserves_source_and_other_tuning(self):
        from omegaconf import OmegaConf
        root = self.home / "self_tossing_hardware"
        root.mkdir()
        source = root / "capture.yaml"
        original = "tuning_file: tuning.yaml\nmodel_file: model.json\nlog_directory: outputs\nrelease_open_delay_s: 0\n"
        tuning_text = ("launch_position_gain_change_s: 3\nlaunch_position_gain_before_per_s: 1\n"
                       "launch_position_gain_after_per_s: 0.25\njoint_position_gain_per_s: 0.5\n")
        source.write_text(original)
        (root / "tuning.yaml").write_text(tuning_text)
        args = ["Nominal", f"config_file:={source}", "tracking_mode:=velocity",
                "launch_position_gain_change_s:=2.125", "launch_position_gain_before_per_s:=0",
                "launch_velocity_feedforward_scale:=0.8", "release_open_delay_s:=-0.1"]
        with patch.object(toss, "_toss_capture_config", return_value=source) as resolve:
            actual = toss.snapshot_toss_tuning(args, self.b.state, root)
        self.assertEqual(resolve.call_args.args[0], args[:3])
        snapshot = Path(actual[-1].split(":=", 1)[1])
        cfg = OmegaConf.load(snapshot)
        tuned = OmegaConf.load(snapshot.parent / cfg.tuning_file)
        self.assertEqual(tuned.launch_position_gain_change_s, 2.125)
        self.assertEqual(tuned.launch_position_gain_before_per_s, 0)
        self.assertEqual(tuned.launch_velocity_feedforward_scale, .8)
        self.assertEqual(cfg.release_open_delay_s, -.1)
        self.assertNotIn("release_open_delay_s", tuned)
        self.assertEqual(tuned.launch_position_gain_after_per_s, .25)
        self.assertEqual(tuned.joint_position_gain_per_s, .5)
        self.assertEqual(Path(cfg.model_file), (root / "model.json").resolve())
        self.assertEqual(source.read_text(), original)
        self.assertEqual((root / "tuning.yaml").read_text(), tuning_text)
        with patch.object(toss, "_toss_capture_config", return_value=source):
            defaults = toss.toss_tuning_defaults(root)
        self.assertEqual(defaults["Nominal"]["launch_position_gain_change_s"], 3)
        self.assertEqual(defaults["Proposed"]["launch_position_gain_before_per_s"], 1)
        self.assertEqual(source.read_text(), original)
        self.assertEqual((root / "tuning.yaml").read_text(), tuning_text)
        self.assertTrue(toss.matches_toss_run([20, 313, actual],
                        ["method:=Nominal", "execute:=true", *actual[1:]]))
        self.assertEqual(toss.snapshot_toss_tuning(args[:3], self.b.state, root), args[:3])
        for value in ("nan", "inf", "-1", "true"):
            with self.assertRaises(ValueError):
                toss.toss_run_args(["Nominal", f"launch_position_gain_change_s:={value}"])

    def test_new_owner_record_matches_exact_runtime_args(self):
        self.flight()
        args = ["Nominal", "trial_file:=/lab/test 2.yaml", "tracking_mode:=position"]
        self.paper(owned=True, args=args)
        self.assertEqual(self.b.available(self.names)[0], ("toss_start",))
        self.procs[-2][1].pop()
        self.assertEqual(self.b.available(self.names), ((), "실행기 방법 또는 설정 불일치"))
        self.procs[-2][1][-1] = "tracking_mode:=velocity"
        self.assertEqual(self.b.available(self.names)[0], ())

    def test_external_executor_status_is_observe_only(self):
        self.flight()
        self.assertEqual(self.b.available(self.names)[0], ("toss_run",))
        self.paper()
        self.assertEqual(self.b.available(self.names), ((), "기존 실험 · IDLE"))
        (self.b.state / "toss_run.json").write_text("[20, 999]")
        self.assertEqual(self.b.available(self.names)[0], ())

    def test_cpp_toss_flight_with_driver_children_enables_run(self):
        self.flight()
        for pid, name in enumerate(("valve_driver_node", "vicon_bridge_node", "imu_bridge_node", "ekf_node"), 30):
            self.add(pid, 10, 10, [f"/home/smrl/FSS_FSW/.native-install/unified-20261118/lib/{name}"])
            self.names.append(name)
        self.assertEqual(self.b.available(self.names)[0], ("toss_run",))
        args = ["Proposed", "tracking_mode:=velocity", "instant_close:=true"]
        self.assertEqual(toss.toss_run_args(args), args)
        self.paper(owned=True, args=args)
        self.assertEqual(self.b.available(self.names)[0], ("toss_start",))

    def test_own_executor_uses_fresh_status_and_unique_local_owner(self):
        self.flight()
        self.paper(owned=True)
        self.assertEqual(self.b.available(self.names)[0], ("toss_start",))
        self.b.observe({"phase": "LAUNCH"})
        self.assertEqual(self.b.available(self.names)[0], ("toss_stop",))
        self.b.phase_at -= 4
        self.assertEqual(self.b.available(self.names)[0], ())
        self.b.observe({"phase": "A_HELD"})
        (self.b.state / "toss_run.json").write_text(json.dumps([20, 313, "Proposed"]))
        self.assertEqual(self.b.available(self.names), ((), "실행기 방법 또는 설정 불일치"))
        (self.b.state / "toss_run.json").write_text(json.dumps([20, 313, "Nominal"]))
        self.names.append("paper_toss_executor")
        self.assertEqual(self.b.available(self.names)[0], ())

    def test_owned_fault_allows_executor_guarded_restart(self):
        self.flight()
        self.paper(owned=True)
        (self.home / ".bash_aliases").write_text("toss_close() { :; }\n")
        self.b.observe({"phase": "FAULT", "stop_confirmed": True})
        self.assertEqual(self.b.available(self.names)[0], ("toss_close", "toss_start", "toss_stop"))
        self.b.phase_at -= 4
        self.assertEqual(self.b.available(self.names)[0], ())
        self.b.observe({"phase": "FAULT"})
        (self.b.state / "toss_run.json").unlink()
        self.assertEqual(self.b.available(self.names)[0], ())

    def test_position_switch_rejects_reused_pid_and_unrelated_hardware(self):
        self.add(40, 1, 40, [])
        self.procs.clear()
        runner = self.proc / "40"
        (runner / "cmdline").write_bytes(b"python3\0-m\0fss_common.flight_runner\0require_explicit_target:=true\0")
        (self.b.state / "position-owner.json").write_text('[40,313]')
        self.add(41, 40, 41, ["ros2", "launch", "fss_bringup", "fss_full.launch.py"])
        self.add(42, 41, 41, ["/lab/fss-fsw/install/fss_manager/lib/fss_manager/manager_node"])
        self.assertEqual(self.b.position_owner(), (40,313))
        (self.b.state / "position-owner.json").write_text('[40,999]')
        self.assertIsNone(self.b.position_owner())
        (self.b.state / "position-owner.json").write_text('[40,313]')
        self.add(50, 1, 50, ["ros2", "launch", "self_tossing_hils", "paper_toss.launch.py"])
        self.assertIsNone(self.b.position_owner())

    def test_position_switch_waits_for_exit_and_ros_removal_before_launch(self):
        fake_fcntl = SimpleNamespace(LOCK_EX=2, LOCK_NB=4, flock=Mock())
        for timeout in (False, True):
            with (patch.dict(sys.modules, fcntl=fake_fcntl),
                  patch.object(self.b, "can_switch_position", return_value=True),
                  patch.object(self.b, "position_owner", return_value=(40,313)),
                  patch.object(self.b.cancelled, "wait", return_value=False),
                  patch.object(toss.os, "pidfd_open", return_value=7, create=True),
                  patch.object(toss.os, "close") as close,
                  patch("signal.pidfd_send_signal", create=True) as signal,
                  patch("select.select", return_value=([7], [], [])),
                  patch.object(toss.time, "monotonic", side_effect=[0,16] if timeout else [0,1,2]),
                  patch.object(self.b, "request", return_value="started") as launch):
                names = Mock(side_effect=[["gnc_node"], []])
                if timeout:
                    with self.assertRaisesRegex(RuntimeError, "시간 초과"):
                        self.b.switch_position(names, Mock())
                    launch.assert_not_called()
                else:
                    self.assertEqual(self.b.switch_position(names, Mock()), "started")
                    self.assertEqual(names.call_count, 2)
                    self.assertEqual(launch.call_args.args[:2], ("toss_fss", True))
                signal.assert_called_once()
                close.assert_called_once_with(7)

    def test_air_control_requires_recognized_bench_but_not_experiment_phase(self):
        self.assertFalse(self.b.can_control_air([]))
        self.flight()
        self.assertFalse(self.b.can_control_air(self.names))
        self.names.append("valve_driver_node")
        self.assertTrue(self.b.can_control_air(self.names))
        self.paper(owned=False)
        self.assertFalse(self.b.can_control_air(self.names))
        (self.b.state / "toss_run.json").write_text('[20, 313, "Nominal"]')
        self.assertTrue(self.b.can_control_air(self.names))
        self.b.observe({"phase": "LAUNCH"})
        self.assertTrue(self.b.can_control_air(self.names))
        self.b.observe({"phase": "STOPPED"})
        self.assertTrue(self.b.can_control_air(self.names))
        self.b.phase_at -= 4
        self.assertTrue(self.b.can_control_air(self.names))

    def test_explicit_stop_targets_existing_fss_snapshot_after_reconnect(self):
        self.flight()
        self.paper(owned=False)
        reconnected = toss.TossBench(self.home, self.proc)
        fake_fcntl = SimpleNamespace(LOCK_EX=2, flock=Mock())
        with patch.dict(sys.modules, fcntl=fake_fcntl), patch.object(toss, "interrupt_fss_processes") as interrupt:
            self.assertTrue(reconnected.can_stop_body())
            self.assertEqual(reconnected.stop_body(), "FSS 프로세스 종료")
            self.assertEqual([pid for pid, _, _ in interrupt.call_args.args[0]], [p[0] for p in self.procs])
            self.assertTrue(reconnected.cancelled.is_set())

    def test_reset_owned_run_after_reconnect_keeps_fss_and_rejects_stale_owner(self):
        self.flight()
        wrapper = ["bash", "-c", 'source "$HOME/.bash_aliases" && "$@"', "mission-control", "toss_run", "Nominal"]
        self.add(20, 1, 20, wrapper)
        self.procs.pop()
        (self.proc / "20/cmdline").write_bytes("\0".join(wrapper).encode() + b"\0")
        self.add(21, 20, 20, ["ros2", "launch", "self_tossing_hils", "paper_toss.launch.py", "method:=Nominal", "execute:=true"])
        self.add(22, 21, 20, ["/lab/self_tossing_hardware/install_client/self_tossing_hils/lib/self_tossing_hils/capture_executor"])
        (self.b.state / "toss_run.json").write_text('[20,313,["Nominal"]]')
        reconnect = toss.TossBench(self.home, self.proc)
        self.assertIn("toss_reset", reconnect.available(self.names)[0])
        with (patch.dict(sys.modules, fcntl=SimpleNamespace(LOCK_EX=2, LOCK_NB=4, flock=Mock())),
              patch.object(toss, "interrupt_fss_processes") as interrupt):
            self.assertEqual(reconnect.reset_run(), "실험 대기 · toss_fss 유지")
            self.assertEqual([pid for pid, _, _ in interrupt.call_args.args[0]], [20,21,22])
            (self.b.state / "toss_run.json").write_text('[20,999,["Nominal"]]')
            self.assertNotIn("toss_reset", reconnect.available(self.names)[0])
            with self.assertRaises(RuntimeError):
                reconnect.reset_run()
            interrupt.assert_called_once()

    def test_changed_pid_identity_is_never_signalled(self):
        with (patch.object(toss.os, "pidfd_open", return_value=7, create=True),
              patch.object(toss.os, "close") as close,
              patch.object(toss, "toss_process", return_value=(1,40,999)),
              patch("signal.pidfd_send_signal", create=True) as send):
            with self.assertRaisesRegex(RuntimeError, "변경"):
                toss.interrupt_fss_processes([(40,313,["manager_node"])], self.proc)
            send.assert_not_called()
            close.assert_called_once_with(7)

    @unittest.skipUnless(hasattr(os, "pidfd_open"), "Linux pidfd shutdown")
    def test_pidfd_shutdown_leaves_unrelated_process_running(self):
        import subprocess
        import time
        children = [subprocess.Popen([sys.executable, "-c", "import time; time.sleep(30)"],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) for _ in range(2)]
        try:
            time.sleep(.1)
            pid = children[0].pid
            ticks = toss.toss_process(pid, Path("/proc"))[2]
            toss.interrupt_fss_processes([(pid,ticks,[sys.executable])],Path("/proc"))
            children[0].wait(timeout=2)
            self.assertIsNone(children[1].poll())
        finally:
            for child in children:
                if child.poll() is None: child.terminate()
                child.wait(timeout=3)

    def test_idle_upgrade_and_competing_owners(self):
        with patch.object(toss, "guarded_full_launch", return_value=False):
            self.assertEqual(self.b.available([])[0], ())
        self.assertEqual(self.b.available([])[0], ("toss_fss",))
        self.assertEqual(self.b.available(["gnc_node"])[0], ())
        self.flight()
        self.procs[0][1].append("use_thrusters:=true")
        self.assertEqual(self.b.available(self.names)[0], ())
        self.procs[0][1].pop()
        self.add(30, 1, 30, ["ros2", "launch", "fss_bringup", "arm.launch.py"])
        self.assertEqual(self.b.available(self.names)[0], ())

    def test_prepare_chains_verified_flight_and_never_starts_motion(self):
        fake_fcntl = SimpleNamespace(LOCK_EX=2, LOCK_NB=4, flock=Mock())
        def launch(action, value, names, verify, *args):
            if action == "toss_fss":
                self.flight()
                (self.b.state / "toss_fss.json").write_text("[10, 313]")
            else:
                self.assertEqual((action, value), ("toss_run", ["Nominal"]))
                self.paper(owned=True, args=value)
        with patch.dict(sys.modules, fcntl=fake_fcntl), patch.object(self.b, "_invoke", side_effect=launch) as invoke:
            self.assertIn("준비 완료", self.b.request("toss_prepare", "Nominal", lambda: self.names, Mock()))
            self.assertEqual([x.args[0] for x in invoke.call_args_list], ["toss_fss", "toss_run"])
            for action, value in (("toss_fss", "yes"), ("toss_run", "Unknown"),
                                  ("toss_prepare", "Nominal;echo foo"), ("toss_start", "yes")):
                with self.assertRaises(ValueError):
                    self.b.request(action, value, lambda: self.names, Mock())
            self.assertEqual(invoke.call_count, 2)

    def test_raw_launch_buttons_invoke_only_the_selected_command(self):
        fake_fcntl = SimpleNamespace(LOCK_EX=2, LOCK_NB=4, flock=Mock())
        with patch.dict(sys.modules, fcntl=fake_fcntl), patch.object(self.b, "_invoke") as invoke:
            self.b.request("toss_fss", True, list, Mock())
            self.assertEqual(invoke.call_count, 1)
            self.assertEqual(invoke.call_args.args[:2], ("toss_fss", True))
            self.b.request("toss_run", ["Nominal", "tracking_mode:=current"], list, Mock())
            self.assertEqual(invoke.call_count, 2)
            self.assertEqual(invoke.call_args.args[:2], ("toss_run", ["Nominal", "tracking_mode:=current"]))

    def test_close_requires_existing_alias_and_owned_fresh_executor(self):
        self.flight()
        self.paper(owned=True)
        self.assertNotIn("toss_close", self.b.available(self.names)[0])
        (self.home / ".bash_aliases").write_text("toss_close() { :; }\n")
        self.assertEqual(self.b.available(self.names)[0], ("toss_close", "toss_start"))
        self.b.observe({"phase": "WAITING_FOR_A"})
        self.assertEqual(self.b.available(self.names)[0], ("toss_close", "toss_stop"))
        self.b.observe({"phase": "LAUNCH"})
        self.assertEqual(self.b.available(self.names)[0], ("toss_stop",))
        self.b.phase_at -= 4
        self.assertEqual(self.b.available(self.names)[0], ())

    @unittest.skipUnless(os.name == "posix", "Linux alias service forwarding")
    def test_close_checks_manual_close_start_service_and_preserves_refusal(self):
        (self.home / ".bash_aliases").write_text(
            "toss_close() { echo \"std_srvs.srv._trigger.Trigger_Response(success=False, message='fixture refused')\"; }\n")
        verify = Mock()
        with (patch.dict(os.environ, HOME=str(self.home)),
              patch.object(self.b, "_stage", return_value=(("toss_close",), "WAITING_FOR_A")),
              self.assertRaisesRegex(RuntimeError, "fixture refused")):
            self.b.request("toss_close", True, lambda: self.names, verify)
        verify.assert_called_once_with("/paper_toss/manual_close_start")

    def test_trigger_refusal_and_cli_errors_are_not_success(self):
        line = "response:\nstd_srvs.srv._trigger.Trigger_Response(success={}, message='device busy')"
        self.assertEqual(toss.toss_trigger_reply(line.format("True")), "device busy")
        with self.assertRaisesRegex(RuntimeError, "device busy"):
            toss.toss_trigger_reply(line.format("False"))
        with self.assertRaises(RuntimeError):
            toss.toss_trigger_reply("Waiting for service ...")

    @unittest.skipUnless(os.name == "posix", "Linux launch locks and bash aliases")
    def test_two_bridges_cannot_prepare_concurrently_and_trigger_uses_existing_alias(self):
        from fcntl import LOCK_EX, flock

        with (self.b.state / "toss-prepare.lock").open("a") as other_pc:
            flock(other_pc, LOCK_EX)
            with patch.object(self.b, "_invoke") as invoke, self.assertRaisesRegex(RuntimeError, "다른 관제"):
                self.b.request("toss_prepare", "Proposed", lambda: [], Mock())
            invoke.assert_not_called()
        (self.home / ".bash_aliases").write_text(
            """toss_start() { echo "std_srvs.srv._trigger.Trigger_Response(success=False, message='fixture refused')"; }\n""")
        verify = Mock()
        with (patch.dict(os.environ, HOME=str(self.home)),
              patch.object(self.b, "_stage", return_value=(("toss_start",), "IDLE")),
              self.assertRaisesRegex(RuntimeError, "fixture refused")):
            self.b.request("toss_start", True, lambda: self.names, verify)
        verify.assert_called_once_with("/paper_toss/start")

    @unittest.skipUnless(os.name == "posix", "Linux bash and launch locks")
    def test_toss_run_preserves_each_option_as_one_argument(self):
        args = ["JF_only", "future_option:=$HOME/*.yaml", "trial_file:=/lab/trial 1/a;b.yaml"]
        (self.home / ".bash_aliases").write_text(
            """toss_run() { printf '%s\\0' "$@" > "$HOME/argv"; }\n""")
        with (patch.dict(os.environ, HOME=str(self.home)),
              patch.object(self.b, "_stage", return_value=(("toss_run",), "ready")),
              patch.object(toss, "toss_process", return_value=(1, 2, 313))):
            self.b._invoke("toss_run", args, list, Mock())
        self.assertEqual(self.b.children[0].wait(timeout=5), 0)
        self.assertEqual((self.home / "argv").read_bytes().split(b"\0"), [*(a.encode() for a in args), b""])
        self.assertEqual(json.loads((self.b.state / "toss_run.json").read_text())[2], args)

    def test_foreign_flight_winning_preparation_never_launches_executor(self):
        fake_fcntl = SimpleNamespace(LOCK_EX=2, LOCK_NB=4, flock=Mock())
        def launch(*_args):
            self.flight()  # A different session won during startup.
        with (patch.dict(sys.modules, fcntl=fake_fcntl),
              patch.object(self.b, "_owner", side_effect=lambda action: 99 if self.procs and action == "toss_fss" else None),
              patch.object(self.b, "_invoke", side_effect=launch) as invoke,
              self.assertRaisesRegex(RuntimeError, "소유자 변경")):
            self.b.request("toss_prepare", "Nominal", lambda: self.names, Mock())
        self.assertEqual(invoke.call_count, 1)

    def test_unified_attach_detects_bench_vs_existing_runtime_without_starting(self):
        self.assertFalse(toss.toss_bench_installed(self.home))
        for name in ("self_tossing_hardware/scripts/env.sh", "self_tossing_hardware/install_client/local_setup.bash",
                     "FSS_FSW/install/local_setup.bash"):
            file = self.home / name
            file.parent.mkdir(parents=True, exist_ok=True)
            file.touch()
        (self.home / ".bash_aliases").write_text("\n".join(
            name + "() { :; }" for name in ("toss_fss", "toss_run", "toss_start", "toss_stop")))
        with patch.object(toss, "device_processes", side_effect=lambda *_: [(argv, env) for _, argv, env in self.procs]):
            self.assertTrue(toss.detect_toss_workflow())
            self.add(80, 1, 80, ["/lab/fss-fsw/install/fss_manager/lib/fss_manager/manager_node"])
            self.assertFalse(toss.detect_toss_workflow())
            self.paper()
            self.assertTrue(toss.detect_toss_workflow())

    def test_monitor_heartbeat_and_stdin_cannot_start_or_prepare(self):
        import mission_ros
        bench, ros, node = Mock(), Mock(), Mock()
        bench.capabilities.return_value = (("toss_prepare",), "ready", True)
        node.get_node_names.return_value = []
        node.get_name.return_value = "fss_mission_test"
        ros.ok.side_effect = [True, False]
        commands = b'["toss_prepare","Nominal"]\n["start_body",true]\n'
        out = []
        with (patch.dict(sys.modules, {"rclpy": ros, "rosidl_runtime_py.convert": Mock()}),
              patch.dict(mission_ros.__dict__, {"toss_bench": bench, "toss_monitor": True, "TOSS_COMMANDS": toss.TOSS_COMMANDS}),
              patch.object(mission_ros, "open_node", return_value=(node, Mock())),
              patch.object(mission_ros.select, "select", return_value=([sys.stdin], [], [])),
              patch.object(mission_ros.os, "read", return_value=commands),
              patch("builtins.print", side_effect=lambda x, **kwargs: out.append(json.loads(x)))):
            mission_ros.serve()
        beat = next(data for event, data in out if event == "connected")
        self.assertEqual((beat["workflow"], beat["can_start_body"], beat["toss_actions"]), ("toss", False, []))
        self.assertTrue(beat["read_only"])
        self.assertEqual([event for event, _ in out][-2:], ["toss_error", "body_error"])
        bench.request.assert_not_called()

    def test_private_native_root_offers_body_start_only_when_bench_idle(self):
        with patch.object(toss, "select_workspace", return_value=toss.PRIVATE):
            self.assertFalse(self.b.capabilities([], None)[2])
            binary = self.home / "FSS_FSW/install/fss_gnc/lib/fss_gnc/gnc_node"
            binary.parent.mkdir(parents=True)
            binary.write_bytes(b"\x7fELF")
            self.assertTrue(self.b.capabilities([], None)[2])
            self.assertFalse(self.b.capabilities([], 0)[2])
            self.flight()
            self.assertFalse(self.b.capabilities(self.names, None)[2])


if __name__ == "__main__":
    unittest.main()
