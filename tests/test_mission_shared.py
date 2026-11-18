"""Shared transport tests use real sockets and fake ROS services, never hardware."""

import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from mission_shared import SharedHub, SHARED_PROTOCOL, shared_key, _shared_relay


class SharedTransportTest(unittest.TestCase):
    def setUp(self):
        listener = socket.socket()
        listener.bind(("127.0.0.1", 0))
        listener.listen()
        self.address = listener.getsockname()
        self.hub = SharedHub(listener)
        self.sockets = []
        self.buffers = {}

    def tearDown(self):
        for sock in self.sockets:
            sock.close()
        self.hub.close()

    def until(self, predicate):
        deadline = time.monotonic() + 3
        while not predicate() and time.monotonic() < deadline:
            time.sleep(.01)
        self.assertTrue(predicate())

    def client(self, profile="auto"):
        sock = socket.create_connection(self.address)
        sock.settimeout(2)
        self.sockets.append(sock)
        previous = self.hub.sequence
        sock.sendall(SharedHub._packet("hello", dict(protocol=SHARED_PROTOCOL, profile=profile)))
        self.until(lambda: self.hub.sequence > previous and self.hub.clients[self.hub.sequence]["hello"])
        return sock, self.hub.sequence

    def receive(self, sock):
        data = self.buffers.pop(sock, b"")
        while b"\n" not in data:
            block = sock.recv(65536)
            self.assertTrue(block, "unexpected EOF")
            data += block
        line, self.buffers[sock] = data.split(b"\n", 1)
        return json.loads(line)

    def test_cache_monitor_permissions_and_targeted_reply(self):
        self.hub.publish("plans", [["bundle/Proposed", "p"]])
        self.hub.publish("plan_defaults", {"Proposed": "bundle/Proposed"})
        self.hub.publish("arm_registration", [0, 0, 0, 0, 40, 150])
        self.hub.publish("connected", dict(observer="one", can_start_body=True, toss_actions=["toss_start"]))
        self.hub.publish("reply", "old result")
        self.hub.publish("paper_status", {"phase": "FAULT"})
        monitor, mid = self.client("monitor")
        control, cid = self.client()
        for sock in (monitor, control):
            events = [self.receive(sock) for _ in range(4)]
            self.assertEqual([e[0] for e in events], ["plans", "plan_defaults", "arm_registration", "connected"])
            if sock is monitor:
                self.assertFalse(events[-1][1]["can_start_body"])
                self.assertEqual(events[-1][1]["toss_actions"], [])
        self.hub.poll()  # Writable attachment is internal, never accepted from the socket.
        monitor.sendall(SharedHub._packet("mode", 5))
        self.assertEqual(self.receive(monitor)[0], "reply_error")
        self.assertEqual(self.hub.poll(), [])
        control.sendall((json.dumps(["target", ["PID", 1, 2, 0], "one"]) + "\n").encode())
        self.until(lambda: not self.hub.commands.empty())
        self.assertEqual(self.hub.poll(), [(cid, "target", ["PID", 1, 2, 0])])
        self.hub.publish("reply", "done", target=cid)
        self.assertEqual(self.receive(control), ["reply", "done"])
        monitor.settimeout(.1)
        with self.assertRaises(socket.timeout):
            monitor.recv(1)

    def test_recording_is_per_client_and_disconnect_does_not_stop_other_client(self):
        a, aid = self.client()
        b, bid = self.client("monitor")
        a.sendall(SharedHub._packet("record_stream", True))
        b.sendall(SharedHub._packet("record_stream", True))
        self.until(lambda: all(c["recording"] for c in self.hub.clients.values()))
        a.sendall(SharedHub._packet("record_stream", False))
        self.until(lambda: not self.hub.clients[aid]["recording"])
        self.assertTrue(self.hub.recording)
        a.close()
        self.until(lambda: aid not in self.hub.clients)
        self.hub.publish("vicon", {"x": 1})
        self.assertEqual(self.receive(b), ["vicon", {"x": 1}])
        self.assertTrue(self.hub.recording)
        b.close()
        self.until(lambda: not self.hub.clients)
        self.assertFalse(self.hub.recording)
        self.assertTrue(self.hub.thread.is_alive())

    def test_slow_client_and_cache_overflow_leave_io_and_other_clients_alive(self):
        fast, fid = self.client()
        slow, sid = self.client("monitor")
        self.hub.max_pending = 256
        self.hub.publish("log", "x" * 512, target=sid)
        self.until(lambda: sid not in self.hub.clients)
        self.hub.publish("reply", "alive", target=fid)
        self.assertEqual(self.receive(fast), ["reply", "alive"])
        self.hub.publish("plans", "x" * 512)  # Replay alone exceeds the bound.
        newcomer = socket.create_connection(self.address)
        self.sockets.append(newcomer)
        newcomer.sendall(SharedHub._packet("hello", dict(protocol=SHARED_PROTOCOL, profile="monitor"))
                          + SharedHub._packet("record_stream", True))
        time.sleep(.15)
        self.assertTrue(self.hub.thread.is_alive())

    def test_shutdown_broadcast_and_distinct_ros_environments(self):
        a, _ = self.client()
        b, _ = self.client("monitor")
        self.hub.close(77)
        self.assertEqual(self.receive(a), ["__restart__", 77])
        self.assertEqual(self.receive(b), ["__restart__", 77])
        env = {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1", "FSS_MISSION_WORKSPACE": "fss-fsw"}
        self.assertEqual(shared_key(env, None), shared_key(dict(env), None))
        for name, value in (("ROS_DOMAIN_ID", "26"), ("CYCLONEDDS_URI", "file:///other.xml"),
                            ("FSS_MISSION_WORKSPACE", "FSS_FSW")):
            self.assertNotEqual(shared_key(env, None), shared_key(dict(env, **{name: value}), None))
        self.assertNotEqual(shared_key(env, None), shared_key(env, 0))

    def test_expired_command_is_not_executed(self):
        client, key = self.client()
        self.hub.poll()
        self.hub.commands.put((key, "target", ["PID", 1, 2, 0], time.monotonic()-2))
        self.assertEqual(self.hub.poll(), [])
        self.assertEqual(self.receive(client), ["reply_error", "중계 명령 대기 시간 초과"])

    def test_command_from_previous_observer_is_rejected(self):
        self.hub.publish("connected", dict(observer="new"))
        client, _ = self.client()
        self.receive(client)
        self.hub.poll()
        client.sendall((json.dumps(["target", ["PID", 1, 2, 0], "old"]) + "\n").encode())
        self.assertEqual(self.receive(client)[0], "reply_error")
        self.assertEqual(self.hub.poll(), [])

    def test_handshake_retirement_retries_but_established_connection_failure_does_not(self):
        sock = Mock()
        sock.sendall.side_effect = BrokenPipeError()
        self.assertEqual(_shared_relay(sock, "monitor"), 77)
        sock.sendall.side_effect = None
        sock.recv.side_effect = ConnectionResetError()
        with patch("mission_shared.select.select", return_value=([sock], [], [])):
            self.assertEqual(_shared_relay(sock, "monitor"), 77)
            sock.recv.side_effect = [b'["log","started"]\n', ConnectionResetError()]
            with patch("mission_shared._shared_write"), self.assertRaises(ConnectionResetError):
                _shared_relay(sock, "monitor")


class SharedCommandTest(unittest.TestCase):
    def test_busy_rejection_and_startup_diagnostic_preserve_request_owner(self):
        import mission_ros as ros
        outputs, batches = [], iter([[(1, "target", ["PID", 1, 2, 0])],
                                     [(2, "target", ["PID", 2, 2, 0])], [], []])
        hub = NS(poll=lambda: next(batches), publish=lambda kind, data, target=None: outputs.append((kind, data, target)),
                 expired=lambda: False, recording=False)
        clock = [100.]
        future = Mock()
        future.done.side_effect = lambda: clock[0] >= 103
        future.result.return_value = NS(success=True, actual_mode=3)
        action = None
        def opened(receive, reply, reject):
            nonlocal action
            action = ros.MissionActions(lambda *_: future, reply, reject)
            action.mode, action.mode_at = 3, clock[0]
            return Mock(get_node_names=lambda: []), action.request
        def spin(*args, **kwargs):
            clock[0] += 1
            action.tick()
        rclpy = NS(ok=Mock(side_effect=[True]*4+[False]), spin_once=spin, shutdown=Mock())
        with (patch.dict(sys.modules, rclpy=rclpy, rosidl_runtime_py=NS(),
                         **{"rosidl_runtime_py.convert": NS(message_to_ordereddict=lambda x:x)}),
              patch.object(ros, "open_node", side_effect=opened),
              patch.object(ros.time, "monotonic", side_effect=lambda: clock[0]),
              patch.object(ros, "startup_failure", lambda names: "startup diagnostic", create=True)):
            ros.serve(hub)
        self.assertIn(("reply_error", "다른 명령 처리 중", 2), outputs)
        self.assertIn(("reply", "목표 설정", 1), outputs)
        self.assertFalse(any(kind == "reply_error" and data == "startup diagnostic" for kind,data,_ in outputs))

    def test_off_owner_survives_failure_of_previous_service(self):
        import mission_ros as ros
        outputs, batches = [], iter([[(1, "target", ["PID", 1, 2, 0])], [(2, "valve", False)], [], [], [], [], []])
        hub = NS(poll=lambda: next(batches), publish=lambda kind, data, target=None: outputs.append((kind, data, target)),
                 expired=lambda: False, recording=False)
        clock, action, calls = [100.], None, []
        previous = Mock()
        previous.done.side_effect = lambda: clock[0] >= 102
        previous.result.return_value = NS(success=False, message="previous service failed")
        complete = Mock(done=lambda: True, result=lambda: NS(success=True, actual_mode=0))
        def call(name, value):
            calls.append((name,value))
            return previous if len(calls)==1 else complete
        def opened(receive, reply, reject):
            nonlocal action
            action = ros.MissionActions(call, reply, reject)
            action.mode, action.mode_at = 2, clock[0]
            return Mock(get_node_names=lambda: []), action.request
        def spin(*args, **kwargs):
            clock[0] += .5
            action.tick()
        rclpy = NS(ok=Mock(side_effect=[True]*7+[False]), spin_once=spin, shutdown=Mock())
        with (patch.dict(sys.modules, rclpy=rclpy, rosidl_runtime_py=NS(),
                         **{"rosidl_runtime_py.convert": NS(message_to_ordereddict=lambda x:x)}),
              patch.object(ros, "open_node", side_effect=opened),
              patch.object(ros.time, "monotonic", side_effect=lambda: clock[0])):
            ros.serve(hub)
        self.assertIn(("valve", False), calls)
        self.assertIn(("reply", "에어베어링 OFF", 2), outputs)
        self.assertFalse(any(kind=="reply_error" and target==2 for kind,_,target in outputs))


@unittest.skipUnless(hasattr(os, "fork"), "Jetson/Linux daemon lifecycle")
class SharedDaemonTest(unittest.TestCase):
    def test_environment_change_before_worker_requests_rediscovery(self):
        module = Path(__file__).resolve().parents[1] / "scripts/mission_shared.py"
        with tempfile.TemporaryDirectory() as directory:
            source = f"""
import os
from pathlib import Path
exec(compile(Path({str(module)!r}).read_text(), 'mission_shared.py', 'exec'))
shared_options=dict(profile='monitor',domain=None,transition='first',workflow='control')
def bridge_context(*args,**kwargs):
    os.environ['ROS_DOMAIN_ID']='987'
    return {{}}
def worker(hub):
    (Path.home()/'unexpected-worker').touch()
run_shared(worker)
"""
            child = subprocess.Popen([sys.executable, "-u", "-c", source],
                env=dict(os.environ, HOME=directory, ROS_DOMAIN_ID="0"),
                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            try:
                self.assertEqual(child.wait(timeout=5), 77)
                self.assertFalse((Path(directory)/"unexpected-worker").exists())
            finally:
                if child.poll() is None:
                    child.kill()
                child.wait(timeout=3)
                child.stdin.close()
                child.stdout.close()
                child.stderr.close()

    def test_simultaneous_clients_share_one_worker_and_survive_first_disconnect(self):
        module = Path(__file__).resolve().parents[1] / "scripts/mission_shared.py"
        with tempfile.TemporaryDirectory() as directory:
            source = f"""
import os,time
from pathlib import Path
exec(compile(Path({str(module)!r}).read_text(), 'mission_shared.py', 'exec'))
shared_options=dict(profile='monitor',domain=None,transition='first',workflow='control')
def bridge_context(*args,**kwargs): return {{}}
def worker(hub):
    hub.idle_seconds=.2
    while not hub.expired():
        hub.publish('connected',dict(observer=str(os.getpid()),peers=[],can_start_body=False))
        marker=Path.home()/'restart'
        if marker.exists():
            marker.unlink()
            raise SystemExit(77)
        time.sleep(.05)
run_shared(worker)
"""
            env = dict(os.environ, HOME=directory)
            children = [subprocess.Popen([sys.executable, "-u", "-c", source], env=env,
                        stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE) for _ in range(2)]
            try:
                first = [json.loads(p.stdout.readline()) for p in children]
                self.assertEqual(first[0][1]["observer"], first[1][1]["observer"])
                children[0].stdin.close()
                children[0].wait(timeout=3)
                self.assertEqual(json.loads(children[1].stdout.readline())[1]["observer"], first[0][1]["observer"])
                (Path(directory)/"restart").touch()
                children[1].wait(timeout=3)
                self.assertEqual(children[1].returncode, 77)
                children[1].stdin.close()
                replacements = [subprocess.Popen([sys.executable, "-u", "-c", source], env=env,
                                stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE) for _ in range(2)]
                children.extend(replacements)
                restarted = [json.loads(p.stdout.readline()) for p in replacements]
                self.assertEqual(restarted[0][1]["observer"], restarted[1][1]["observer"])
                self.assertNotEqual(restarted[0][1]["observer"], first[0][1]["observer"])
                for child in replacements:
                    child.stdin.close()
                    child.wait(timeout=3)
                until = time.monotonic() + 4
                while list(Path(directory).rglob("*.sock")) and time.monotonic() < until:
                    time.sleep(.05)
                self.assertFalse(list(Path(directory).rglob("*.sock")))
            finally:
                for child in children:
                    if child.poll() is None:
                        child.kill()
                    child.wait(timeout=3)
                    child.stdout.close()
                    child.stderr.close()


if __name__ == "__main__":
    unittest.main()
