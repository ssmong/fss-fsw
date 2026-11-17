"""SSH transport for the existing Mission Control dashboard."""

import base64
import hashlib
import json
import queue
import shlex
import socket
import threading
import time
from pathlib import Path
from types import SimpleNamespace

from PySide6.QtCore import QObject, Signal

from mission_terminal import ShellSessions

PREFERRED_WORKSPACE = ("FSS_FSW" if (Path(__file__).resolve().parents[1]
                                     / "src/hardware/hardware_runtime/package.xml").is_file()
                       else "fss-fsw")


class JetsonLink(QObject):
    trust = Signal(object, str, object)
    changed = Signal(str)

    def __init__(self, data, monitor_only=False):
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
        self.monitor_only = monitor_only
        self.workspace = None
        self.private_arm = False
        self.can_start_body = False
        self.read_only = "모니터링 전용" if monitor_only else ""
        self.observer = None
        self.peers = ()

    def start(self, host, user, domain, password):
        threading.Thread(target=self._run, args=(host, user, domain, password), daemon=True).start()

    def request(self, name, value):
        if self.ready and not self.read_only:
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
                        self.can_start_body = getattr(value, "can_start_body", False)
                        self.read_only = getattr(value, "read_only", "") or (
                            "모니터링 전용" if self.monitor_only else "")
                elif name == "bridge_error":
                    self.ready = False
                    self.bridge_error = value
                    self.data.set_command_result(value, 40)
                elif name == "reply":
                    self.data.set_command_result(value)
                elif name == "reply_error":
                    self.data.set_command_result(value, 40)
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
        code = Path(__file__).with_name("mission_ros.py").read_text(encoding="utf-8")
        code = Path(__file__).with_name("mission_plan.py").read_text(encoding="utf-8") + "\n" + code
        runtime = Path(__file__).with_name("mission_runtime.py").read_text(encoding="utf-8")
        domain = None if domain is None else int(domain)
        code = (runtime + f"\nstartup_status, read_only = prepare_session({bool(self.monitor_only)!r}, {domain!r})\n"
                + "private_arm = private_base() is not None\n"
                + code)
        preferred = PREFERRED_WORKSPACE
        select = runtime + f"\nprint(select_workspace({preferred!r}, monitor_only={bool(self.monitor_only)!r}))\n"
        # Native processes must not resolve a missing package from a legacy underlay.
        cmd = (f"export FSS_MISSION_PREFERRED={shlex.quote(preferred)} && "
               f"FSS_MISSION_WORKSPACE=$(python3 -c {shlex.quote(select)}) && export FSS_MISSION_WORKSPACE && "
               'if [[ "$FSS_MISSION_WORKSPACE" == FSS_FSW ]]; then setup=setup.bash; '
               'else unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH ROS_PACKAGE_PATH PYTHONPATH LD_LIBRARY_PATH FSS_CONFIG_DIR; '
               'setup=local_setup.bash; fi && '
               'source /opt/ros/humble/setup.bash && source "$HOME/$FSS_MISSION_WORKSPACE/install/$setup" '
               f"&& exec python3 -u -c {shlex.quote(code)}")
        with client.get_transport().open_session(timeout=8) as ch:
            ch.exec_command("bash -c " + shlex.quote(cmd))
            ch.settimeout(0.1)
            pending = b""
            error = ""
            last_rx = time.monotonic()
            full_stream = False
            while not self.stopped.is_set():
                recording = self.data.recorder.active
                if recording != full_stream:
                    ch.sendall((json.dumps(["record_stream", recording]) + "\n").encode())
                    full_stream = recording
                while not self.commands.empty():
                    stamp, name, value = self.commands.get_nowait()
                    if time.monotonic() - stamp < 1:
                        ch.sendall((json.dumps([name, value]) + "\n").encode())
                    else:
                        self.inbox.put([("reply_error", "전송 시간 초과")], timeout=2)
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
                        batch.append(event)
                if batch:
                    self.inbox.put(batch, timeout=2)
