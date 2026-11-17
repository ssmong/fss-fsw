"""Self-tossing adapter exercised with an isolated process tree and stub launches."""

import json
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

    def paper(self, owned=False):
        self.add(20, 1, 20, ["ros2", "launch", "self_tossing_hils", "paper_toss.launch.py", "method:=Nominal",
                             "execute:=true", "tracking_mode:=velocity"])
        self.add(21, 20, 20, ["/lab/self_tossing_hardware/install_client/self_tossing_hils/lib/self_tossing_hils/capture_executor"])
        self.names.append("paper_toss_executor")
        self.b.observe({"phase": "IDLE"})
        if owned:
            (self.b.state / "toss_run.json").write_text(json.dumps([20, 313, "Nominal"]))

    def test_external_executor_status_is_observe_only(self):
        self.flight()
        self.assertEqual(self.b.available(self.names)[0], ("toss_prepare",))
        self.paper()
        self.assertEqual(self.b.available(self.names), ((), "기존 실험 · IDLE"))
        (self.b.state / "toss_run.json").write_text("[20, 999]")
        self.assertEqual(self.b.available(self.names)[0], ())

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

    def test_idle_upgrade_and_competing_owners(self):
        with patch.object(toss, "guarded_full_launch", return_value=False):
            self.assertEqual(self.b.available([])[0], ())
        self.assertEqual(self.b.available([])[0], ("toss_prepare",))
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
                self.assertEqual((action, value), ("toss_run", "Nominal"))
                self.paper(owned=True)
        with patch.dict(sys.modules, fcntl=fake_fcntl), patch.object(self.b, "_invoke", side_effect=launch) as invoke:
            self.assertIn("준비 완료", self.b.request("toss_prepare", "Nominal", lambda: self.names, Mock()))
            self.assertEqual([x.args[0] for x in invoke.call_args_list], ["toss_fss", "toss_run"])
            for action, value in (("toss_fss", True), ("toss_run", "Nominal"),
                                  ("toss_prepare", "Nominal;echo foo"), ("toss_start", "yes")):
                with self.assertRaises(ValueError):
                    self.b.request(action, value, lambda: self.names, Mock())
            self.assertEqual(invoke.call_count, 2)

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


if __name__ == "__main__":
    unittest.main()
