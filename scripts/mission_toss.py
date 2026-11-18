"""Operator-triggered adapter for the bench's ~/.bash_aliases self-tossing interface."""

import ast
import json
import math
import os
import re
import subprocess
import threading
import time
from pathlib import Path

if "device_processes" not in globals():  # SSH bridge concatenates these modules.
    from mission_runtime import (FSS_NODES, PRIVATE, ROS_ENV_KEYS, checkout_for, device_processes,
                                 executable_name, guarded_full_launch, launch_package, process_role,
                                 native_workspace, ros_network, select_workspace)


TOSS_METHODS = ("Proposed", "Nominal", "JF_only")
TOSS_COMMANDS = ("toss_fss", "toss_run", "toss_close", "toss_start", "toss_stop", "toss_prepare", "toss_reset")
TOSS_START_PHASES = ("IDLE", "STOPPED", "MISSED", "A_HELD", "FAULT")
TOSS_UPGRADE = "Self-tossing FSW 런처 업데이트 필요"
TOSS_TUNING_KEYS = ("launch_position_gain_change_s", "launch_position_gain_before_per_s",
                    "launch_velocity_feedforward_scale", "release_open_delay_s")


def toss_run_args(value):
    """Normalize the old method-only request and the GUI's ROS launch argv."""
    args = [value] if isinstance(value, str) else value
    if not isinstance(args, list) or not args or args[0] not in TOSS_METHODS:
        raise ValueError("실험 방법을 선택하세요.")
    keys = set()
    for option in args[1:]:
        if not isinstance(option, str) or not option.isprintable():
            raise ValueError("실험 옵션은 이름:=값 형식으로 입력하세요.")
        key, separator, argument = option.partition(":=")
        if not separator or not argument or not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_./-]*", key):
            raise ValueError("실험 옵션은 이름:=값 형식으로 입력하세요.")
        if key in ("method", "execute"):
            raise ValueError(f"{key}는 실험 준비에서 지정합니다.")
        if key in keys:
            raise ValueError(f"중복 옵션 · {key}")
        if key in TOSS_TUNING_KEYS:
            try:
                number = float(argument)
            except ValueError as exc:
                raise ValueError(f"{key}: 0 이상의 유한한 값을 입력하세요.") from exc
            if not math.isfinite(number) or (number < 0 and key != "release_open_delay_s"):
                raise ValueError(f"{key}: 0 이상의 유한한 값을 입력하세요.")
        keys.add(key)
    return list(args)


def _toss_capture_config(args, root):
    """Ask the existing launcher's config selector; do not execute any launch actions."""
    import importlib.util
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch.utilities import perform_substitutions
    if "_paper_launch_path" in globals():
        launch_path = globals()["_paper_launch_path"]
    else:
        from mission_plan import _paper_launch_path as launch_path

    launch = launch_path(root)
    spec = importlib.util.spec_from_file_location("_mission_toss_launch", launch)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(dict(option.split(":=", 1) for option in args[1:]))
    context.launch_configurations.update(method=args[0], execute="true")
    # toss_run changes directory to its workspace before launching.
    for key in ("config_file", "trial_file"):
        if key in context.launch_configurations:
            path = Path(context.launch_configurations[key]).expanduser()
            context.launch_configurations[key] = str(path if path.is_absolute() else root / path)
    for entity in module.generate_launch_description().entities:
        if isinstance(entity, DeclareLaunchArgument) and entity.name not in context.launch_configurations:
            context.launch_configurations[entity.name] = perform_substitutions(context, entity.default_value)
    # Capture the Node description instead of constructing or starting a ROS node.
    module.Node = lambda **kwargs: kwargs
    nodes = module._executor(context)
    if len(nodes) != 1 or nodes[0].get("executable") != "capture_executor":
        raise RuntimeError("Self-tossing 설정 선택기를 확인할 수 없습니다.")
    configs = [p["config_file"] for p in nodes[0]["parameters"] if isinstance(p, dict) and "config_file" in p]
    if len(configs) != 1 or not isinstance(configs[0], str):
        raise RuntimeError("Self-tossing 설정 파일을 확인할 수 없습니다.")
    return Path(configs[0])


def toss_tuning_defaults(root=None):
    """Read each method's current YAML values without creating a run or snapshot."""
    root = Path(root or os.environ.get("SELF_TOSSING_HARDWARE_ROOT", Path.home() / "self_tossing_hardware"))
    if not root.is_dir():
        return {}
    try:
        from omegaconf import OmegaConf
        from omegaconf.errors import OmegaConfBaseException
        from yaml import YAMLError
    except ImportError:
        return {}
    defaults = {}
    for method in TOSS_METHODS:
        try:
            source = _toss_capture_config([method], root)
            config = OmegaConf.to_container(OmegaConf.load(source), resolve=True)
            if not isinstance(config, dict):
                continue
            tuning_file = config.get("tuning_file")
            if tuning_file is not None:
                tuning = OmegaConf.to_container(OmegaConf.load(source.parent / tuning_file), resolve=True)
                if not isinstance(tuning, dict) or config.keys() & tuning.keys():
                    continue
                config.update(tuning)
            defaults[method] = {key: config[key] for key in TOSS_TUNING_KEYS
                                if type(config.get(key)) in (int, float) and math.isfinite(config[key])}
        except (ImportError, OSError, ValueError, TypeError, KeyError, AttributeError, RuntimeError,
                SyntaxError, OmegaConfBaseException, YAMLError):
            continue
    return defaults


def snapshot_toss_tuning(args, state, root=None):
    """Replace GUI-only gain options with an isolated config_file launch argument."""
    import tempfile

    args = toss_run_args(args)
    overrides = {key: float(value) for option in args[1:] for key, value in [option.split(":=", 1)]
                 if key in TOSS_TUNING_KEYS}
    if not overrides:
        return args
    from omegaconf import OmegaConf
    root = Path(root or os.environ.get("SELF_TOSSING_HARDWARE_ROOT", Path.home() / "self_tossing_hardware"))
    actual = [args[0], *(option for option in args[1:] if option.split(":=", 1)[0] not in TOSS_TUNING_KEYS)]
    source = _toss_capture_config(actual, root)
    config = OmegaConf.to_container(OmegaConf.load(source), resolve=True)
    if not isinstance(config, dict):
        raise ValueError("Self-tossing 설정은 mapping이어야 합니다.")
    tuning_file = config.get("tuning_file")
    tuning = None
    if tuning_file is not None:
        tuning = OmegaConf.to_container(OmegaConf.load(source.parent / tuning_file), resolve=True)
        if not isinstance(tuning, dict) or config.keys() & tuning.keys():
            raise ValueError("Self-tossing tuning 설정이 중복되거나 잘못되었습니다.")
    applied = {**config, **(tuning or {}), **overrides}
    for key in (*TOSS_TUNING_KEYS, "launch_position_gain_after_per_s"):
        if key not in applied:
            continue
        value = applied[key]
        if type(value) not in (int, float) or not math.isfinite(value) or (value < 0 and key != "release_open_delay_s"):
            raise ValueError(f"Self-tossing 설정 확인 필요 · {key}")
    if config.get("model_file"):
        config["model_file"] = str((source.parent / config["model_file"]).resolve())
    if config.get("log_directory"):
        log = Path(config["log_directory"]).expanduser()
        config["log_directory"] = str(log if log.is_absolute() else (root / log).resolve())
    directory = Path(state) / "run-configs"
    directory.mkdir(parents=True, exist_ok=True)
    snapshot = Path(tempfile.mkdtemp(prefix="toss-", dir=directory)) / "capture.yaml"
    if tuning is not None:
        for key, value in overrides.items():
            (config if key in config else tuning)[key] = value
        OmegaConf.save(config=OmegaConf.create(tuning), f=snapshot.with_name("tuning.yaml"))
        config["tuning_file"] = "tuning.yaml"
    else:
        config.update(overrides)
    OmegaConf.save(config=OmegaConf.create(config), f=snapshot)
    return [actual[0], *(option for option in actual[1:] if not option.startswith("config_file:=")),
            f"config_file:={snapshot}"]


def matches_toss_run(record, options):
    if len(record) != 3:
        return False
    try:
        method, *extra = toss_run_args(record[2])
    except ValueError:
        return False
    expected = [f"method:={method}", "execute:=true", *extra]
    if isinstance(record[2], str):
        # Older records stored only the method. Nominal was launched both with the
        # alias defaults and, in the following release, a fixed velocity override.
        return sorted(options) == sorted(expected) or (
            method == "Nominal" and sorted(options) == sorted([*expected, "tracking_mode:=velocity"]))
    return options == expected


def toss_bench_installed(home=None):
    home = home or Path.home()
    root = home / "self_tossing_hardware"
    if not all(path.is_file() for path in (root / "scripts/env.sh", root / "install_client/local_setup.bash",
                                           home / PRIVATE / "install/local_setup.bash")):
        return False
    try:
        aliases = (home / ".bash_aliases").read_text()
    except OSError:
        return False
    return all(re.search(r"^\s*(?:function\s+)?" + name + r"\s*\(\s*\)\s*\{", aliases, re.MULTILINE)
               for name in ("toss_fss", "toss_run", "toss_start", "toss_stop"))


def detect_toss_workflow(domain=None):
    if not toss_bench_installed():
        return False
    native = False
    for argv, env in device_processes(Path("/proc")):
        if (toss_flight(argv, env) or launch_package(argv) == "self_tossing_hils"
                or executable_name(argv) == "capture_executor"):
            return True
        native |= process_role(argv, environ=env) == "flight"
    return not native


def toss_process(pid, proc):
    # /proc/pid/stat's parenthesized command can contain spaces and parentheses.
    fields = (proc / str(pid) / "stat").read_text().rsplit(")", 1)[1].split()
    return int(fields[1]), int(fields[3]), int(fields[19])  # parent, session, start ticks


def toss_flight(argv, env):
    expected = {"use_arm:=true", "use_thrusters:=false", "use_imu:=true", "use_realsense:=false",
                "auto_activate:=true", "toss_nav_uncertainty_grace:=true",
                f"arm_config:={Path.home() / PRIVATE}/src/hardware/hardware_runtime/config/arm.yaml"}
    options = [arg for arg in argv if ":=" in arg]
    return (launch_package(argv) == "fss_bringup" and "fss_full.launch.py" in argv
            and checkout_for(argv, env) == PRIVATE and len(options) == len(expected) and set(options) == expected)


def active_toss_env():
    fleets = [env for argv, env in device_processes(Path("/proc")) if toss_flight(argv, env)]
    return fleets[0] if len(fleets) == 1 else None


def attach_toss(domain=None):
    # The bench's interactive shell may have its own DDS settings.
    env = active_toss_env()
    if env is not None:
        for key in ROS_ENV_KEYS:
            if key in env:
                os.environ[key] = env[key]
            else:
                os.environ.pop(key, None)
    if domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(domain)


def toss_trigger_reply(output):
    for line in reversed(output.splitlines()):
        try:
            expr = ast.parse(line.strip(), mode="eval").body
            if not isinstance(expr, ast.Call):
                continue
            if getattr(expr.func, "attr", None) != "Trigger_Response":
                continue
            fields = {item.arg: ast.literal_eval(item.value) for item in expr.keywords}
            if fields.get("success") is True:
                return fields.get("message") or "요청 수락"
            raise RuntimeError(fields.get("message") or "실험 서비스가 요청을 거부했습니다.")
        except (ValueError, SyntaxError):
            continue
    raise RuntimeError("실험 서비스 응답을 확인할 수 없습니다.")


def interrupt_fss_processes(processes, proc):
    """Signal verified process identities, never a login shell or an entire process group."""
    import signal
    import select

    handles = {}
    seen = set()
    try:
        for pid, ticks, argv in processes:
            if pid in seen:
                continue
            seen.add(pid)
            fd = None
            try:
                fd = os.pidfd_open(pid)
                if toss_process(pid, proc)[2] != ticks:
                    os.close(fd)
                    fd = None
                    raise RuntimeError("종료 대상 프로세스가 변경되었습니다.")
                handles[fd] = executable_name(argv)
            except (FileNotFoundError, ProcessLookupError):
                if fd is not None:
                    os.close(fd)
                continue
        for fd in handles:
            try:
                signal.pidfd_send_signal(fd, signal.SIGINT)
            except ProcessLookupError:
                pass
        pending = set(handles)
        for timeout, next_signal in ((10, signal.SIGTERM), (5, None)):
            deadline = time.monotonic() + timeout
            while pending and time.monotonic() < deadline:
                ready, _, _ = select.select(list(pending), [], [], .1)
                pending.difference_update(ready)
            if not pending:
                return
            if next_signal is not None:
                for fd in pending:
                    try:
                        signal.pidfd_send_signal(fd, next_signal)
                    except ProcessLookupError:
                        pass
        raise RuntimeError("종료 확인 시간 초과 · " + ", ".join(sorted({handles[fd] for fd in pending})))
    finally:
        for fd in handles:
            os.close(fd)


class TossBench:
    def __init__(self, home=None, proc=Path("/proc")):
        self.home = home or Path.home()
        self.proc = proc
        self.state = self.home / ".ros/mission-control"
        self.children = []
        self.child_lock = threading.Lock()
        self.cancelled = threading.Event()
        self.phase = ""
        self.phase_at = float("-inf")

    def observe(self, status):
        if isinstance(status, dict) and isinstance(status.get("phase"), str):
            self.phase = status["phase"]
            self.phase_at = time.monotonic()

    def _owner_record(self, action):
        try:
            record = json.loads((self.state / (action + ".json")).read_text())
            pid, ticks = record[:2]
            _, session, start = toss_process(pid, self.proc)
            return record if pid == session and ticks == start else None
        except (FileNotFoundError, ValueError, TypeError, IndexError):
            return None

    def _owner(self, action):
        record = self._owner_record(action)
        return record[0] if record else None

    def _stage(self, names, expected_fss=None):
        # Reap only launch wrappers started by this bridge.
        with self.child_lock:
            self.children[:] = [child for child in self.children if child.poll() is None]
        if not (self.home / ".bash_aliases").is_file():
            return (), "~/.bash_aliases 없음"
        ancestry, programs, fss, paper, executors = {}, {}, [], [], []
        paper_options = []
        for pid, argv, env in device_processes(self.proc, with_pid=True):
            try:
                parent, session, _ = toss_process(pid, self.proc)
            except FileNotFoundError:
                continue
            if ros_network(env) != ros_network(os.environ):
                return (), "다른 ROS 환경에서 기체 사용 중"
            ancestry[pid] = (parent, session)
            programs[pid] = executable_name(argv)
            package = launch_package(argv)
            if toss_flight(argv, env):
                fss.append(pid)
            elif package == "self_tossing_hils" and "paper_toss.launch.py" in argv:
                paper.append(pid)
                paper_options = argv[argv.index("paper_toss.launch.py") + 1:]
            elif executable_name(argv) == "capture_executor":
                executors.append(pid)
        if len(fss) > 1 or len(paper) > 1 or len(executors) > 1:
            return (), "기체 또는 실험 실행기 중복"
        if expected_fss and fss and ancestry[fss[0]][1] != expected_fss:
            return (), "기체 실행 소유자 변경"
        # Only launch-tree children may share hardware with a verified full-toss stack.
        roots = set(fss + paper)
        fss_nodes = set()
        for pid in ancestry:
            owner = pid
            while owner in ancestry and owner not in roots and owner > 1:
                owner = ancestry[owner][0]
            if owner not in roots:
                return (), "다른 실험에서 사용 중"
            if owner in fss:
                fss_nodes.add(programs[pid])
        if any(names.count(name) > 1 for name in (*FSS_NODES, "paper_toss_executor")):
            return (), "ROS 기체 또는 실험 실행기 중복"
        phase = self.phase if time.monotonic() - self.phase_at < 3 else ""
        record = self._owner_record("toss_run")
        launch = record[0] if record else None
        if paper or executors or "paper_toss_executor" in names or launch:
            if not launch or any(ancestry[pid][1] != launch for pid in paper + executors):
                return (), "기존 실험 · " + (phase or "상태 수신 대기")
            if paper and not matches_toss_run(record, paper_options):
                return (), "실행기 방법 또는 설정 불일치"
            if not fss or not paper or not executors or "paper_toss_executor" not in names or not phase:
                return (), "실험 상태 수신 대기"
            if phase in TOSS_START_PHASES:
                close = ("toss_close",) if self._has_close_alias() else ()
                if phase in ("A_HELD", "FAULT"):
                    return (*close, "toss_start", "toss_stop"), phase
                return (*close, "toss_start"), phase
            if phase == "WAITING_FOR_A" and self._has_close_alias():
                return ("toss_close", "toss_stop"), phase
            return ("toss_stop",), phase
        if fss:
            if ({"arm_driver_node", "manager_node"}.issubset(names)
                    and "manager_node" in fss_nodes and fss_nodes.intersection(("arm_driver_node", "arm_driver"))):
                return ("toss_run",), "실험 FSW 준비됨"
            return (), "실험 FSW 준비 중"
        if self._owner("toss_fss"):
            return (), "실험 FSW 준비 중"
        if FSS_NODES.intersection(names):
            return (), "ROS 도메인에서 다른 기체 사용 중"
        if not guarded_full_launch(self.home / PRIVATE):
            return (), TOSS_UPGRADE
        return ("toss_fss",), "실험 FSW 대기"

    def available(self, names):
        actions, state = self._stage(names)
        if self.owned_run() is not None:
            actions = (*actions, "toss_reset")
        return actions, state

    def owned_run(self):
        record = self._owner_record("toss_run")
        if record is None:
            return None
        try:
            pid, ticks = record[:2]
            argv = (self.proc / str(pid) / "cmdline").read_bytes().rstrip(b"\0").split(b"\0")
            if len(argv) < 5 or argv[3:5] != [b"mission-control", b"toss_run"]:
                return None
            if Path(os.fsdecode(argv[0])).name not in ("bash", "sh"):
                return None
            return pid, ticks
        except (OSError, ValueError, TypeError):
            return None

    def can_stop_body(self):
        return bool(list(device_processes(self.proc)))

    def stop_body(self):
        from fcntl import LOCK_EX, flock
        self.state.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.cancelled.set()
        with (self.state / "startup.lock").open("a") as lock:
            flock(lock, LOCK_EX)
            targets = []
            for pid, argv, _ in device_processes(self.proc, with_pid=True):
                try:
                    targets.append((pid, toss_process(pid, self.proc)[2], argv))
                except (FileNotFoundError, ProcessLookupError):
                    continue
            interrupt_fss_processes(targets, self.proc)
        return "FSS 프로세스 종료" if targets else "실행 중인 FSS 프로세스 없음"

    def reset_run(self):
        from fcntl import LOCK_EX, LOCK_NB, flock
        with (self.state / "startup.lock").open("a") as lock:
            flock(lock, LOCK_EX | LOCK_NB)
            owner = self.owned_run()
            if owner is None:
                raise RuntimeError("지상국이 시작한 실험 실행기만 대기로 돌릴 수 있습니다.")
            pid, ticks = owner
            targets = [(pid, ticks, ["bash", "toss_run"])]
            for child, argv, env in device_processes(self.proc, with_pid=True):
                _, session, start = toss_process(child, self.proc)
                if session != pid:
                    continue
                if (ros_network(env) != ros_network(os.environ) or executable_name(argv) in FSS_NODES):
                    raise RuntimeError("실험 실행기 소유 범위를 확인할 수 없습니다.")
                targets.append((child, start, argv))
            if self.owned_run() != owner:
                raise RuntimeError("실험 실행 소유자 변경")
            interrupt_fss_processes(targets, self.proc)
        self.phase, self.phase_at = "", float("-inf")
        return "실험 대기 · toss_fss 유지"

    def position_owner(self):
        try:
            pid, ticks = json.loads((self.state / "position-owner.json").read_text())
            _, session, start = toss_process(pid, self.proc)
            argv = (self.proc / str(pid) / "cmdline").read_bytes().split(b"\0")
            if (pid != session or ticks != start or argv[1:3] != [b"-m", b"fss_common.flight_runner"]
                    or b"require_explicit_target:=true" not in argv):
                return None
            devices = list(device_processes(self.proc, with_pid=True))
            if not devices:
                return None
            for child, _, env in devices:
                if ros_network(env) != ros_network(os.environ):
                    return None
                seen = set()
                while child != pid:
                    if child <= 1 or child in seen:
                        return None
                    seen.add(child)
                    child = toss_process(child, self.proc)[0]
            return pid, ticks
        except (OSError, ValueError, TypeError, IndexError):
            return None

    def can_switch_position(self):
        return (toss_bench_installed(self.home) and guarded_full_launch(self.home / PRIVATE)
                and self.position_owner() is not None)

    def switch_position(self, names, verify_service):
        import signal
        import select
        from fcntl import LOCK_EX, LOCK_NB, flock

        with (self.state / "startup.lock").open("a") as lock:
            flock(lock, LOCK_EX | LOCK_NB)
            if not self.can_switch_position() or self.cancelled.is_set():
                raise RuntimeError("지상국이 시작한 위치제어 본체만 전환할 수 있습니다.")
            pid, ticks = self.position_owner()
            # pidfd keeps a reused PID from receiving the shutdown signal.
            fd = os.pidfd_open(pid)
            try:
                if self.position_owner() != (pid, ticks):
                    raise RuntimeError("기체 실행 소유자 변경")
                signal.pidfd_send_signal(fd, signal.SIGINT)
                deadline = time.monotonic() + 15
                while not self.cancelled.wait(.1):
                    if (select.select([fd], [], [], 0)[0]
                            and not list(device_processes(self.proc))
                            and not FSS_NODES.intersection(names())):
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError("위치제어 종료 확인 시간 초과 · Self-tossing을 시작하지 않았습니다.")
                if self.cancelled.is_set():
                    raise RuntimeError("연결 종료")
            finally:
                os.close(fd)
        return self.request("toss_fss", True, names, verify_service)

    def can_control_air(self, names, actions=None):
        actions, state = self._stage(names)
        return (names.count("valve_driver_node") == 1
                and (bool({"toss_run", "toss_start", "toss_close", "toss_stop"}.intersection(actions))
                     or state in ("실험 상태 수신 대기", "실험 FSW 준비 중")))

    def _has_close_alias(self):
        try:
            return bool(re.search(r"^\s*(?:function\s+)?toss_close\s*\(\s*\)\s*\{",
                                  (self.home / ".bash_aliases").read_text(), re.MULTILINE))
        except OSError:
            return False

    def capabilities(self, names, domain):
        actions, status = self.available(names)
        body = False
        if domain is None and (status in (TOSS_UPGRADE, "실험 FSW 대기")):
            try:
                work = select_workspace(os.getenv("FSS_MISSION_PREFERRED", PRIVATE), home=self.home, proc=self.proc)
                body = native_workspace(work, self.home) and guarded_full_launch(self.home / work)
            except (RuntimeError, OSError):
                pass
        return actions, status, body

    def request(self, action, value, names, verify_service):
        from fcntl import LOCK_EX, LOCK_NB, flock

        if action not in TOSS_COMMANDS or (action not in ("toss_prepare", "toss_run") and value is not True):
            raise ValueError("올바르지 않은 실험 요청")
        if action in ("toss_prepare", "toss_run"):
            value = toss_run_args(value)
        self.state.mkdir(mode=0o700, parents=True, exist_ok=True)
        if action == "toss_reset":
            return self.reset_run()
        if action not in ("toss_prepare", "toss_fss", "toss_run"):
            return self._invoke(action, value, names, verify_service)
        with (self.state / "toss-prepare.lock").open("a") as reservation:
            try:
                flock(reservation, LOCK_EX | LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("다른 관제에서 실험 준비 중") from exc
            if action != "toss_prepare":
                return self._invoke(action, value, names, verify_service)
            actions, status = self._stage(names())
            expected_fss = None
            if actions == ("toss_fss",):
                self._invoke("toss_fss", True, names, verify_service)
                expected_fss = self._owner("toss_fss")
                if not expected_fss:
                    raise RuntimeError(f"FSW 기동 실패 · {self.state / 'toss_fss.log'}")
                deadline = time.monotonic() + 25
                while not self.cancelled.wait(0.25):
                    actions, status = self._stage(names(), expected_fss)
                    if actions == ("toss_run",):
                        break
                    if (not self._owner("toss_fss") or time.monotonic() >= deadline
                            or status != "실험 FSW 준비 중"):
                        raise RuntimeError(f"FSW 준비 확인 필요 · {status} · {self.state / 'toss_fss.log'}")
            if self.cancelled.is_set() or actions != ("toss_run",):
                raise RuntimeError(status)
            self._invoke("toss_run", value, names, verify_service, expected_fss)
            deadline = time.monotonic() + 25
            while not self.cancelled.wait(0.25):
                actions, status = self._stage(names(), expected_fss)
                if actions and all(item in ("toss_close", "toss_start", "toss_stop") for item in actions):
                    return f"{value[0]} 실험 준비 완료 · {status}"
                if (not self._owner("toss_run") or time.monotonic() >= deadline
                        or status != "실험 상태 수신 대기"):
                    raise RuntimeError(f"실행기 준비 확인 필요 · {status} · {self.state / 'toss_run.log'}")
            raise RuntimeError("연결 종료")

    def _invoke(self, action, value, names, verify_service, expected_fss=None):
        from fcntl import LOCK_EX, LOCK_NB, flock

        with (self.state / "startup.lock").open("a") as lock:
            try:
                flock(lock, LOCK_EX | LOCK_NB)
            except BlockingIOError as exc:
                raise RuntimeError("다른 기체 요청 처리 중") from exc
            choices, status = self._stage(names(), expected_fss)
            if action not in choices or self.cancelled.is_set():
                raise RuntimeError(status)
            command = ["bash", "-c", 'source "$HOME/.bash_aliases" && "$@"', "mission-control", action]
            if action == "toss_run":
                value = snapshot_toss_tuning(value, self.state)
                command.extend(value)
            if action in ("toss_close", "toss_start", "toss_stop"):
                service = "manual_close_start" if action == "toss_close" else action.removeprefix("toss_")
                verify_service("/paper_toss/" + service)
                # These aliases call the existing Trigger service, including its ownership/interlock checks.
                # A ROS CLI exit status alone does not establish that the request was accepted.
                with (self.state / "toss-trigger.log").open("w+") as output:
                    child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                             start_new_session=True)
                    try:
                        child.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        import signal
                        # Only cancel our transient CLI request. The target executor owns its motion.
                        os.killpg(child.pid, signal.SIGKILL)
                        child.wait()
                        raise RuntimeError("실험 서비스 응답 시간 초과 · 현재 단계를 확인하세요.")
                    output.seek(0)
                    result = output.read(32768)
                if child.returncode:
                    raise RuntimeError(result.strip()[-350:] or "실험 서비스 호출 실패")
                return toss_trigger_reply(result)
            log = self.state / (action + ".log")
            with log.open("ab") as output:
                child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=output, stderr=subprocess.STDOUT,
                                         start_new_session=True)
            with self.child_lock:
                self.children.append(child)
            _, _, ticks = toss_process(child.pid, self.proc)
            record = self.state / (action + ".json")
            owner = [child.pid, ticks] + ([value] if action == "toss_run" else [])
            record.with_suffix(".tmp").write_text(json.dumps(owner))
            record.with_suffix(".tmp").replace(record)
            return f"{action} 기동 요청 · {log}"
