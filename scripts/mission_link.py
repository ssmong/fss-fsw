"""SSH transport for the existing Mission Control dashboard."""

import base64
import hashlib
import json
import queue
import shlex
import socket
import threading
import time
import zlib
from pathlib import Path
from types import SimpleNamespace

from PySide6.QtCore import QObject, Signal

from mission_terminal import ShellSessions
from mission_toss import toss_run_args

SOURCE_TIMEOUT_SEC = 2.5
PREFERRED_WORKSPACE = ("FSS_FSW" if (Path(__file__).resolve().parents[1]
                                     / "src/hardware/hardware_runtime/package.xml").is_file()
                       else "fss-fsw")


class JetsonLink(QObject):
    trust = Signal(object, str, object)
    changed = Signal(str)

    def __init__(self, data, monitor_only=False, profile=None):
        super().__init__()
        self.data = data
        self.inbox = queue.Queue(maxsize=256)
        self.commands = queue.Queue()
        self.stopped = threading.Event()
        self.decision = threading.Event()
        self.accepted = False
        self.ready = False
        self.shells = ShellSessions()
        self.bridge_error = ""
        self.profile = "monitor" if monitor_only else (profile or "auto")
        if self.profile not in ("auto", "control", "monitor", "toss"):
            raise ValueError("잘못된 FSS 연결 방식")
        self.workflow = "toss" if self.profile == "toss" else "control"
        self.monitor_only = self.profile == "monitor"
        self.toss_actions = ()
        self.toss_state = ""
        self.toss_pending = False
        self.workspace = None
        self.private_arm = False
        self.can_start_body = False
        self.body_start_pending = False
        self.last_body_status_at = float("-inf")
        self.read_only = "모니터링 전용" if self.monitor_only else ""
        self.observer = None
        self.peers = ()

    def start(self, host, user, domain, password):
        threading.Thread(target=self._run, args=(host, user, domain, password), daemon=True).start()

    def can_toss(self, name):
        return (self.profile in ("auto", "toss") and self.workflow == "toss"
                and self.ready and not self.read_only and not self.bridge_error
                and not self.stopped.is_set() and not self.toss_pending and not self.body_start_pending
                and time.monotonic() - self.last_body_status_at <= SOURCE_TIMEOUT_SEC
                and name in ("toss_prepare", "toss_start", "toss_stop") and name in self.toss_actions)

    def request(self, name, value):
        if not self.ready or self.monitor_only or self.read_only or self.stopped.is_set() or self.bridge_error:
            return
        if name.startswith("toss_"):
            if not self.can_toss(name):
                return
            if name == "toss_prepare":
                try:
                    args = toss_run_args(value)
                except ValueError:
                    return
                value = args[0] if isinstance(value, str) else args
            elif value is not True:
                return
            self.toss_pending = True
        elif name == "start_body":
            if (self.profile not in ("auto", "control") or not self.can_start_body or self.body_start_pending
                    or self.toss_pending or time.monotonic() - self.last_body_status_at > SOURCE_TIMEOUT_SEC):
                return
            self.body_start_pending = True
        elif self.workflow != "control" or self.profile == "toss":
            return
        self.commands.put((time.monotonic(), name, value))

    def close(self):
        self.ready = False
        self.stopped.set()
        self.decision.set()
        self.shells.close()

    def drain(self):
        for shell in self.shells.sessions.values():
            shell.drain()
        if self.stopped.is_set():
            return
        while True:
            try:
                messages = self.inbox.get_nowait()
            except queue.Empty:
                return
            for name, value in messages:
                if name == "connected":
                    self.ready = True
                    if value is not None:
                        self.observer = value.observer
                        self.peers = value.peers
                        self.workspace = getattr(value, "workspace", None)
                        self.private_arm = getattr(value, "private_arm", False)
                        workflow = getattr(value, "workflow", self.workflow)
                        if workflow == "toss" and self.workflow != "toss":
                            self.shells.selected = "log"
                        self.workflow = workflow
                        self.can_start_body = getattr(value, "can_start_body", False)
                        self.last_body_status_at = getattr(value, "received_at", float("-inf"))
                        self.toss_actions = tuple(getattr(value, "toss_actions", ()))
                        self.toss_state = getattr(value, "toss_state", "")
                        self.read_only = getattr(value, "read_only", "") or (
                            "모니터링 전용" if self.monitor_only else "")
                elif name == "bridge_error":
                    self.ready = False
                    self.toss_pending = False
                    self.toss_actions = ()
                    self.bridge_error = value
                    self.data.set_command_result(value, 40)
                elif name == "reply":
                    self.data.set_command_result(value)
                elif name == "reply_error":
                    self.data.set_command_result(value, 40)
                elif name in ("body_reply", "body_error"):
                    self.body_start_pending = False
                    self.can_start_body = False
                    self.data.set_command_result(value, 20 if name == "body_reply" else 40)
                elif name in ("toss_reply", "toss_error"):
                    self.toss_pending = False
                    self.toss_actions = ()
                    self.data.set_command_result(value, 20 if name == "toss_reply" else 40)
                else:
                    self.data.receive(name, value)

    def _run(self, host, user, domain, password):
        try:
            import paramiko

            link = self

            class ConfirmKey(paramiko.MissingHostKeyPolicy):
                def missing_host_key(self, client, hostname, key):
                    fingerprint = base64.b64encode(hashlib.sha256(key.asbytes()).digest())
                    link.trust.emit(link, hostname, fingerprint.decode().rstrip("="))
                    link.decision.wait()
                    if not link.accepted or link.stopped.is_set():
                        raise ValueError("호스트 키 확인 취소")
                    client.get_host_keys().add(hostname, key.get_name(), key)
                    known.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                    with known.open("a", encoding="utf-8") as out:
                        out.write(f"{hostname} {key.get_name()} {key.get_base64()}\n")

            known = Path.home() / ".ssh" / "fss_ground_station_known_hosts"
            with paramiko.SSHClient() as client:
                client.load_system_host_keys()
                if known.exists():
                    client.load_host_keys(str(known))
                client.set_missing_host_key_policy(ConfirmKey())
                client.connect(host, username=user, password=password or None,
                               timeout=7, banner_timeout=8, auth_timeout=12)
                del password
                if self.stopped.is_set():
                    return
                self.shells.attach(client)
                try:
                    self._telemetry(client, domain)
                except Exception as exc:
                    if not self.stopped.is_set() and client.get_transport().is_active():
                        while not self.stopped.is_set():
                            try:
                                self.inbox.put([("bridge_error", str(exc) or type(exc).__name__)], timeout=0.2)
                                break
                            except queue.Full:
                                continue
                        # A missing ROS package must not drop the logged-in operator's shell.
                        while client.get_transport().is_active() and not self.stopped.wait(0.2):
                            pass
                    if not self.stopped.is_set():
                        raise
        except Exception as exc:
            if not self.stopped.is_set():
                self.changed.emit(str(exc) or type(exc).__name__)
        finally:
            self.shells.close()
            self.shells.join()

    def _telemetry(self, client, domain):
        directory = Path(__file__).parent
        domain = None if domain is None else int(domain)
        names = ("mission_runtime", "mission_toss", "mission_bootstrap")
        routing = "\n".join((directory / (name + ".py")).read_text(encoding="utf-8") for name in names)
        selection = (routing + f'\nprint(":".join(bridge_selection({self.profile!r}, {domain!r}, '
                     'os.environ["FSS_MISSION_TRANSITION"], os.environ["FSS_MISSION_PREFERRED"])))')
        context = (f'shared_options=dict(profile={self.profile!r}, domain={domain!r}, '
                   'transition=os.environ["FSS_MISSION_TRANSITION"], workflow=os.environ["FSS_MISSION_WORKFLOW"]); '
                   f'globals().update(bridge_context("auto", {domain!r}, '
                   'shared_options["transition"], shared_options["workflow"], allow_start=False))')
        bridge = "\n".join([routing, context, *((directory / (name + ".py")).read_text(encoding="utf-8")
                                             for name in ("mission_plan", "mission_tracking", "mission_shared", "mission_ros"))])

        def python_argument(source):
            # Keep the SSH command below Linux's per-argument exec limit, including nested shell quoting.
            data = base64.b64encode(zlib.compress(source.encode("utf-8"))).decode("ascii")
            return shlex.quote(f"import base64,zlib;exec(compile(zlib.decompress(base64.b64decode({data!r})),"
                               "'<mission-control>','exec'))")

        # Each pass sources exactly one ROS overlay in its own subshell. The outer shell keeps the SSH
        # channel open while changing from an idle bench to an operator-selected native stack.
        cmd = (f'export FSS_MISSION_PREFERRED={shlex.quote(PREFERRED_WORKSPACE)}; '
               'connect_bridge() (export FSS_MISSION_TRANSITION="$1"; '
               f'route=$(python3 -c {python_argument(selection)}) || exit $?; '
               'export FSS_MISSION_WORKFLOW="${route%%:*}" FSS_MISSION_WORKSPACE="${route#*:}"; '
               'if [[ "$FSS_MISSION_WORKFLOW" == toss ]]; then '
               'source "$HOME/self_tossing_hardware/scripts/env.sh"; '
               'else if [[ "$FSS_MISSION_WORKSPACE" == FSS_FSW ]]; then setup=setup.bash; '
               'else unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH FSS_CONFIG_DIR; '
               'setup=local_setup.bash; fi; '
               'source /opt/ros/humble/setup.bash && source "$HOME/$FSS_MISSION_WORKSPACE/install/$setup"; fi '
               f'&& exec python3 -u -c {python_argument(bridge)}); '
               'transition=first; while true; do connect_bridge "$transition"; rc=$?; case "$rc" in '
               '75|77) transition=passive;; 76) transition=body;; *) exit "$rc";; esac; done')
        with client.get_transport().open_session(timeout=8) as ch:
            ch.exec_command("bash -c " + shlex.quote(cmd))
            ch.settimeout(0.1)
            pending = b""
            error = ""
            last_rx = time.monotonic()
            full_stream = False
            observer = None
            while not self.stopped.is_set():
                recording = self.data.recorder.active
                if recording != full_stream:
                    ch.sendall((json.dumps(["record_stream", recording]) + "\n").encode())
                    full_stream = recording
                while not self.commands.empty():
                    stamp, name, value = self.commands.get_nowait()
                    if time.monotonic() - stamp < 1:
                        ch.sendall((json.dumps([name, value, observer]) + "\n").encode())
                    else:
                        error_event = "body_error" if name == "start_body" else "reply_error"
                        if name.startswith("toss_"):
                            error_event = "toss_error"
                        self.inbox.put([(error_event, "전송 시간 초과")], timeout=2)
                if ch.recv_stderr_ready():
                    error = ch.recv_stderr(8192).decode(errors="replace").strip()[-350:]
                if time.monotonic() - last_rx > 12:
                    raise ValueError(error or "FSS 응답 시간 초과")
                try:
                    chunk = ch.recv(65536)
                except socket.timeout:
                    continue
                if not chunk:
                    if ch.recv_stderr_ready():
                        error = ch.recv_stderr(16384).decode(errors="replace").strip()[-350:]
                    raise ValueError(error or "FSS 연결 종료")
                last_rx = time.monotonic()
                lines = (pending + chunk).split(b"\n")
                pending = lines.pop()
                batch = []
                for line in lines:
                    try:
                        event = json.loads(line, object_hook=lambda d: SimpleNamespace(**d))
                    except (ValueError, UnicodeError):
                        error = line.decode(errors="replace")[-350:]
                        continue
                    if isinstance(event, list) and len(event) == 2:
                        if event[0] == "connected" and isinstance(event[1], SimpleNamespace):
                            event[1].received_at = last_rx
                            if event[1].observer != observer:
                                observer = event[1].observer
                                full_stream = False
                        batch.append(event)
                if batch:
                    self.inbox.put(batch, timeout=2)
