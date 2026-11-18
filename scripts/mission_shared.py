"""One ROS observer per Jetson environment, with private SSH/Unix-socket clients."""

import collections
import hashlib
import json
import os
from pathlib import Path
import queue
import select
import socket
import threading
import time


SHARED_PROTOCOL = 4
SHARED_CACHE = {"plans", "plan_defaults", "arm_registration", "connected"}
SHARED_DISPLAY = {"vicon", "ekf", "ref", "arm_joints", "arm_driver", "arm_table", "arm_base", "arm_tcp"}


def shared_key(environment, domain):
    # Explicit domains must not follow an automatic connection into another network.
    keys = ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION", "CYCLONEDDS_URI",
            "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE", "FSS_MISSION_WORKSPACE")
    identity = [SHARED_PROTOCOL, domain, [(key, environment.get(key, "")) for key in keys]]
    return hashlib.sha256(json.dumps(identity).encode()).hexdigest()[:24]


class SharedHub:
    """Socket I/O never runs on the ROS thread; slow clients lose their own connection."""

    def __init__(self, listener, *, max_pending=8 * 1024 * 1024, idle_seconds=60):
        self.listener = listener
        self.listener.setblocking(False)
        self.max_pending, self.idle_seconds = max_pending, idle_seconds
        self.clients, self.cache = {}, {}
        self.commands = queue.Queue(maxsize=128)
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.closing = False
        self.empty_since = time.monotonic()
        self.sequence = 0
        self.thread = threading.Thread(target=self._io, name="mission-sockets", daemon=True)
        self.thread.start()

    @property
    def recording(self):
        with self.lock:
            return any(c["recording"] for c in self.clients.values())

    def _drop(self, key):
        client = self.clients.pop(key, None)
        if client:
            client["socket"].close()
        if not self.clients:
            self.empty_since = time.monotonic()

    def _enqueue(self, key, packet):
        client = self.clients.get(key)
        if client is None:
            return
        if client["size"] + len(packet) > self.max_pending:
            # EOF makes the PC close its recording with an explicit connection error.
            self._drop(key)
            return
        client["out"].append(packet)
        client["size"] += len(packet)

    @staticmethod
    def _packet(kind, data):
        return (json.dumps([kind, data], separators=(",", ":")) + "\n").encode()

    def publish(self, kind, data, target=None):
        packet = self._packet(kind, data)
        with self.lock:
            if target is None and kind in SHARED_CACHE:
                self.cache[kind] = (data, packet)
            for key, client in list(self.clients.items()):
                if not client["hello"] or (target is not None and key != target):
                    continue
                outgoing = packet
                if kind == "connected":
                    status = dict(data, shared_bridge=True, shared_clients=len(self.clients))
                    if client["profile"] == "monitor":
                        status.update(read_only="모니터링 전용", can_start_body=False, toss_actions=[])
                    elif client["profile"] == "control":
                        status["toss_actions"] = []
                    elif client["profile"] == "toss":
                        status["can_start_body"] = False
                    outgoing = self._packet(kind, status)
                elif kind in SHARED_DISPLAY and not client["recording"]:
                    stamp = data.get("header", {}).get("stamp", {}) if isinstance(data, dict) else {}
                    ns = stamp.get("sec", 0) * 10**9 + stamp.get("nanosec", 0)
                    bucket = (ns or time.monotonic_ns()) // 50_000_000
                    if client["buckets"].get(kind) == bucket:
                        continue
                    client["buckets"][kind] = bucket
                self._enqueue(key, outgoing)

    def poll(self):
        items = []
        for _ in range(32):
            try:
                key, name, value, received = self.commands.get_nowait()
            except queue.Empty:
                break
            with self.lock:
                if key in self.clients:
                    if name != "__attach__" and time.monotonic() - received > 1:
                        error = "toss_error" if name.startswith("toss_") else (
                            "body_error" if name == "start_body" else "reply_error")
                        self.publish(error, "중계 명령 대기 시간 초과", target=key)
                    else:
                        items.append((key, name, value))
        return items

    def expired(self):
        with self.lock:
            return not self.clients and time.monotonic() - self.empty_since >= self.idle_seconds

    def _receive(self, key, line):
        client = self.clients.get(key)
        if client is None:
            return
        event = json.loads(line)
        if not isinstance(event, list) or len(event) not in (2, 3):
            raise ValueError("invalid command")
        name, value = event[:2]
        if not client["hello"]:
            if (name != "hello" or not isinstance(value, dict) or value.get("protocol") != SHARED_PROTOCOL
                    or value.get("profile") not in ("auto", "control", "monitor", "toss")):
                raise ValueError("incompatible shared bridge")
            client["hello"], client["profile"] = True, value["profile"]
            for kind, (data, _) in list(self.cache.items()):
                self.publish(kind, data, target=key)
            if value["profile"] in ("auto", "control"):
                self.commands.put_nowait((key, "__attach__", True, time.monotonic()))
            return
        if name == "record_stream":
            client["recording"] = value is True
            return
        error = "toss_error" if isinstance(name, str) and name.startswith("toss_") else (
            "body_error" if name == "start_body" else "reply_error")
        profile = client["profile"]
        if (profile == "monitor" or (profile == "control" and isinstance(name, str) and name.startswith("toss_"))
                or (profile == "toss" and name not in ("valve", "toss_fss", "toss_run", "toss_close", "toss_prepare", "toss_start", "toss_stop"))
                or name not in ("mode", "valve", "target", "controller", "start_body",
                                "toss_fss", "toss_run", "toss_close", "toss_prepare", "toss_start", "toss_stop")):
            self.publish(error, "이 연결에서는 명령을 보낼 수 없습니다.", target=key)
            return
        status = self.cache.get("connected", ({}, None))[0]
        if name == "valve" and ((profile == "toss" and status.get("workflow") != "toss")
                                or (profile == "control" and status.get("workflow") == "toss")):
            self.publish(error, "이 연결에서는 명령을 보낼 수 없습니다.", target=key)
            return
        if len(event) != 3 or not event[2] or event[2] != status.get("observer"):
            self.publish(error, "중계 전환 중 · 연결 상태 갱신 후 다시 보내세요.", target=key)
            return
        self.commands.put_nowait((key, name, value, time.monotonic()))

    def _io(self):
        while not self.closed.wait(.02):
            with self.lock:
                try:
                    while True:
                        if self.closing:
                            break
                        sock, _ = self.listener.accept()
                        sock.setblocking(False)
                        if len(self.clients) >= 16:
                            sock.close()
                            continue
                        self.sequence += 1
                        self.clients[self.sequence] = dict(socket=sock, hello=False, profile="monitor",
                            recording=False, incoming=b"", out=collections.deque(), size=0, buckets={},
                            accepted=time.monotonic())
                except BlockingIOError:
                    pass
                except OSError:
                    return
                for key, client in list(self.clients.items()):
                    try:
                        if not client["hello"] and time.monotonic() - client["accepted"] > 5:
                            raise ValueError("handshake timeout")
                        try:
                            incoming = client["socket"].recv(8192)
                            if not incoming:
                                raise ValueError("client closed")
                            client["incoming"] += incoming
                            if len(client["incoming"]) > 65536:
                                raise ValueError("command too large")
                            lines = client["incoming"].split(b"\n")
                            client["incoming"] = lines.pop()
                            for line in lines:
                                if key not in self.clients:
                                    break
                                self._receive(key, line)
                        except BlockingIOError:
                            pass
                        if key not in self.clients:
                            continue
                        if client["out"]:
                            # Coalesce small topic messages into one write every 20 ms.
                            batch = bytearray()
                            while client["out"] and len(batch) < 65536:
                                batch.extend(client["out"].popleft())
                            client["out"].appendleft(bytes(batch))
                            try:
                                sent = client["socket"].send(client["out"][0])
                                client["size"] -= sent
                                remainder = client["out"].popleft()[sent:]
                                if remainder:
                                    client["out"].appendleft(remainder)
                            except BlockingIOError:
                                pass
                    except (OSError, ValueError, TypeError, queue.Full):
                        self._drop(key)

    def close(self, code=0):
        with self.lock:
            self.closing = True
            for key in list(self.clients):
                self._enqueue(key, self._packet("__restart__", code))
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            with self.lock:
                if all(not c["out"] for c in self.clients.values()):
                    break
            time.sleep(.01)
        self.closed.set()
        self.thread.join(timeout=2)
        with self.lock:
            for key in list(self.clients):
                self._drop(key)
            self.listener.close()


def _shared_relay(sock, profile):
    received = [False]
    try:
        return _shared_relay_stream(sock, profile, received)
    except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
        if not received[0]:
            return 77
        raise


def _shared_relay_stream(sock, profile, received):
    sock.sendall(SharedHub._packet("hello", {"profile": profile, "protocol": SHARED_PROTOCOL}))
    pending = b""
    while True:
        ready, _, _ = select.select([sock, 0], [], [], 1)
        if 0 in ready:
            data = os.read(0, 8192)
            if not data:
                return 0
            sock.sendall(data)
        if sock in ready:
            data = sock.recv(65536)
            if not data:
                if not received[0]:
                    return 77  # A connection racing daemon retirement retries discovery.
                raise RuntimeError("공용 중계 연결 종료 · 수신 지연 또는 중계 상태를 확인하세요.")
            received[0] = True
            lines = (pending + data).split(b"\n")
            pending = lines.pop()
            output = bytearray()
            for line in lines:
                if line.startswith(b'["__restart__",'):
                    if output:
                        _shared_write(output)
                    return int(json.loads(line)[1])
                output.extend(line + b"\n")
            if output:
                _shared_write(output)


def _shared_write(data):
    view = memoryview(data)
    while view:
        view = view[os.write(1, view):]


def run_shared(worker):
    """Called before rclpy.init; only the lock winner starts a ROS worker."""
    import fcntl
    import stat
    options = globals()["shared_options"]
    directory = Path.home() / ".ros/mission-control/shared"
    directory.mkdir(parents=True, mode=0o700, exist_ok=True)
    info = directory.lstat()
    if not stat.S_ISDIR(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
        raise RuntimeError("공용 중계 디렉터리 소유권/권한 확인 필요")
    key = shared_key(os.environ, options["domain"])
    path = directory / (key + ".sock")
    def connect():
        sock = socket.socket(socket.AF_UNIX)
        try:
            sock.connect(str(path))
            return sock
        except OSError:
            sock.close()
            return None
    with (directory / (key + ".lock")).open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        sock = connect()
        if sock is None:
            path.unlink(missing_ok=True)
            listener = socket.socket(socket.AF_UNIX)
            listener.bind(str(path))
            os.chmod(path, 0o600)
            listener.listen(16)
            pid = os.fork()
            if pid == 0:
                os.setsid()
                log = os.open(str(directory / (key + ".log")), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
                null = os.open(os.devnull, os.O_RDONLY)
                os.dup2(null, 0)
                os.dup2(log, 1)
                os.dup2(log, 2)
                fcntl.flock(lock, fcntl.LOCK_UN)
                for fd in os.listdir("/proc/self/fd"):
                    if int(fd) > 2 and int(fd) != listener.fileno():
                        try:
                            os.close(int(fd))
                        except OSError:
                            pass
                hub = SharedHub(listener)
                code = 0
                try:
                    profile = "auto"
                    transition = options["transition"]
                    if options["profile"] == "monitor" and transition == "first":
                        transition = "passive"
                    globals().update(bridge_context(profile, options["domain"], transition, options["workflow"]))
                    if shared_key(os.environ, options["domain"]) != key:
                        raise SystemExit(77)
                    worker(hub)
                except SystemExit as exc:
                    code = int(exc.code or 0)
                except Exception as exc:
                    hub.publish("bridge_error", str(exc) or type(exc).__name__)
                    code = 1
                finally:
                    with (directory / (key + ".lock")).open("a") as retirement:
                        fcntl.flock(retirement, fcntl.LOCK_EX)
                        hub.close(code)
                        path.unlink(missing_ok=True)
                os._exit(code)
            listener.close()
            sock = connect()
            if sock is None:
                raise RuntimeError("공용 중계 시작 실패")
    with sock:
        raise SystemExit(_shared_relay(sock, options["profile"]))
