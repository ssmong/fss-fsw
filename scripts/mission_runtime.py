"""Select or start the flight stack on the Jetson hosting this SSH session."""

import ast
import os
import subprocess
import time
from pathlib import Path, PurePosixPath


FSS_NODES = {
    "manager_node", "valve_driver_node", "vicon_bridge_node", "imu_bridge_node",
    "ekf_node", "gnc_node", "plant_model_node", "arm_driver_node", "reaction_wheel_driver_node",
}
ROS_ENV_KEYS = ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION",
                "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE")


class HardwareBusy(RuntimeError):
    def __init__(self, environment, owner="self_tossing_hils"):
        super().__init__(f"{owner} 사용 중 · 모니터링만 가능")
        self.environment = environment


PUBLIC, PRIVATE = "fss-fsw", "FSS_FSW"
PRIVATE_CPP = PRIVATE + "/runtime/cpp"


def program_path(argv):
    if not argv:
        return PurePosixPath("")
    program = PurePosixPath(argv[0])
    if (program.name.startswith("python") or program.name in ("sh", "bash")) and len(argv) > 1:
        program = PurePosixPath(argv[1])
    return program


def executable_name(argv):
    return program_path(argv).name.removesuffix(".py")


def launch_package(argv):
    if executable_name(argv) == "ros2":
        for verb in ("launch", "run"):
            if verb in argv:
                i = argv.index(verb) + 1
                return argv[i] if i < len(argv) else ""


def checkout_for(argv, env):
    paths = [program_path(argv)]
    if package := launch_package(argv):
        launch_file = PurePosixPath(package)
        if launch_file.name != "base.launch.py":
            paths.append(launch_file)
        paths.extend(PurePosixPath(path) for path in env.get("AMENT_PREFIX_PATH", "").split(":"))
        paths.append(launch_file)
    for path in paths:
        if PRIVATE in path.parts:
            i = path.parts.index(PRIVATE)
            return PRIVATE_CPP if path.parts[i + 1:i + 3] == ("runtime", "cpp") else PRIVATE
        if PUBLIC in path.parts:
            return PUBLIC


def process_role(argv, selected=None, environ=None):
    name, package = executable_name(argv), launch_package(argv)
    program = program_path(argv)
    launch_file = PurePosixPath(package or "")
    if (launch_file.name == "arm.launch.py" or (package == "fss_bringup" and "arm.launch.py" in argv)
            or "self_tossing_hils" in launch_file.parts
            or package in ("self_tossing_hils", "self_tossing_hardware") or name in ("capture_executor", "paper_toss_node")
            or ({"self_tossing_hils", "self_tossing_hardware"}.intersection(program.parts) and name in
                ("air_bearing", "arm_tracking", "pid_tuning", "pd_sweep", "run_air_bearing",
                 "run_air_bearing.sh", "run_joint_pid.sh", "test_arm_tracking.sh",
                 "run_arm_reference_ros", "record_gripper_close", "test_gripper_current",
                 "benchmark_capture_ros", "arm_executor", "tracking_executor", "motion_tool"))):
        return "competing"
    if name in ("xsens_mti_node", "xsens_mti_driver") or package == "xsens_mti_ros2_driver":
        return "imu"
    if (name in FSS_NODES or package in ("fss_bringup", "hardware_runtime")
            or launch_file.name in ("fss_full.launch.py", "base.launch.py") or "hardware_runtime" in program.parts):
        project = checkout_for(argv, environ or {})
        if (project not in (PUBLIC, PRIVATE_CPP) or package == "hardware_runtime"
                or "hardware_runtime" in program.parts or (selected and project != selected)
                or program.name == "gnc_node.py" or "gnc_backend:=python" in argv):
            return "competing"
        return "flight"
    return None


def device_processes(proc):
    for process in proc.iterdir():
        if not process.name.isdigit():
            continue
        relevant = False
        try:
            argv = os.fsdecode((process / "cmdline").read_bytes()).strip("\0").split("\0")
            program = program_path(argv)
            if not program.is_absolute() and (len(program.parts) > 1 or program.suffix in (".py", ".sh")):
                # Shell/Python scripts are often started relative to the experiment checkout.
                pos = 0 if PurePosixPath(argv[0]) == program else 1
                argv[pos] = str((process / "cwd").readlink() / program)
            relevant = bool(process_role(argv))
            if not relevant:
                continue
            env = dict(item.split("=", 1) for item in
                       os.fsdecode((process / "environ").read_bytes()).split("\0") if "=" in item)
        except FileNotFoundError:
            continue
        except PermissionError as exc:
            if relevant:
                raise RuntimeError("실행 중인 장치 프로세스의 ROS 환경을 확인할 수 없습니다.") from exc
            continue
        yield argv, env


def native_install(work):
    # A partial colcon build also produces setup.bash. Check the installed GNC binary
    # (both merged and isolated layouts), so a Python launcher cannot masquerade as C++.
    for prefix in (work / "install/fss_gnc", work / "install"):
        try:
            with (prefix / "lib/fss_gnc/gnc_node").open("rb") as binary:
                if binary.read(4) == b"\x7fELF":
                    return True
        except FileNotFoundError:
            continue
    return False


def select_workspace(preferred, home=None, proc=Path("/proc"), monitor_only=False):
    home = home or Path.home()
    candidates = (PRIVATE_CPP, PUBLIC) if preferred == PRIVATE else (PUBLIC, PRIVATE_CPP)
    installed = [name for name in candidates if (home / name / "install/local_setup.bash").is_file()
                 and native_install(home / name)]
    legacy_available = (home / PRIVATE / "install/setup.bash").is_file()
    active = set()
    competing = False
    for argv, env in device_processes(proc):
        role = process_role(argv, environ=env)
        if role in ("flight", "competing"):
            active.add(checkout_for(argv, env))
            competing |= role == "competing"
    if legacy_available and PRIVATE in active:
        return PRIVATE
    for name in installed:
        if name in active:
            return name
    if legacy_available and (competing or (monitor_only and not installed)):
        return PRIVATE
    if installed:
        return installed[0]
    build = ("cd ~/FSS_FSW && bash scripts/cpp_fsw.sh build" if preferred == PRIVATE
             else "cd ~/fss-fsw && colcon build --symlink-install")
    raise RuntimeError(f"Jetson C++ FSW 빌드 필요 · {build}")


def ros_network(env):
    return tuple(env.get(key, "0" if key in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY") else "")
                 for key in ROS_ENV_KEYS)


def monitoring_environment(processes, domain):
    networks = {}
    for name, env in processes:
        if domain is not None and env.get("ROS_DOMAIN_ID", "0") != str(domain):
            continue
        _, names = networks.setdefault(ros_network(env), (env, set()))
        names.add(name)
    if not networks:
        return {key: os.environ[key] for key in ROS_ENV_KEYS if key in os.environ}

    def flight_evidence(names):
        # A separate experiment client may remain alive on a different domain.
        return (len(names & {"valve_driver_node", "manager_node"}), "valve_driver_node" in names,
                bool(names & FSS_NODES))

    best = max(flight_evidence(names) for _, names in networks.values())
    matches = [env for env, names in networks.values() if flight_evidence(names) == best]
    if len(matches) > 1:
        domains = ", ".join(sorted({env.get("ROS_DOMAIN_ID", "0") for env in matches}))
        raise RuntimeError(f"Jetson에서 모니터링할 ROS 환경을 구분할 수 없습니다 (도메인 {domains}).")
    return matches[0]


def private_base():
    home = Path.home()
    lab = home / PRIVATE
    if (os.getenv("FSS_MISSION_PREFERRED") == PRIVATE
            or os.getenv("FSS_MISSION_WORKSPACE") in (PRIVATE, PRIVATE_CPP)
            or (lab / "src/hardware/hardware_runtime/package.xml").is_file()
            or (lab / "install/setup.bash").is_file()):
        return home / PRIVATE_CPP / "base.launch.py"


def local_service_owner(name, proc=Path("/proc")):
    """A visible server must belong to the selected local flight stack."""
    owned = False
    profile = private_base()
    calibrated = profile is None
    for argv, env in device_processes(proc):
        if (process_role(argv, os.getenv("FSS_MISSION_WORKSPACE", PUBLIC), env) == "competing"
                or ros_network(env) != ros_network(os.environ)):
            return False
        if executable_name(argv) == name:
            owned = True
        if profile and launch_package(argv) == profile.as_posix():
            calibrated = True
    return owned and calibrated


def local_stacks(proc=Path("/proc"), monitor=False, domain=None):
    stacks = {"flight": [], "imu": []}
    processes = []
    conflicts = set()
    profile = private_base()
    calibrated = profile is None
    for argv, env in device_processes(proc):
        role = process_role(argv, os.getenv("FSS_MISSION_WORKSPACE", PUBLIC), env)
        ros_env = {key: env[key] for key in ROS_ENV_KEYS if key in env}
        processes.append((executable_name(argv), ros_env))
        if role == "competing":
            owner = checkout_for(argv, env)
            package = launch_package(argv)
            if package in ("self_tossing_hils", "self_tossing_hardware") or "self_tossing_hils" in program_path(argv).parts:
                owner = "self_tossing_hils"
            elif owner == PRIVATE or package == "hardware_runtime":
                owner = "FSS_FSW/hardware_runtime"
            conflicts.add(owner or package or executable_name(argv))
            continue
        stacks[role].append(ros_env)
        if profile and launch_package(argv) == profile.as_posix():
            calibrated = True
    if stacks["flight"] and not calibrated:
        conflicts.add("기존 FSW · 실험실 기체 설정 미확인")
    if conflicts or monitor:
        environment = monitoring_environment(processes, domain)
        if conflicts:
            raise HardwareBusy(environment, " · ".join(sorted(conflicts)))
        stacks["monitor"] = environment
    return stacks


def forwards_target_wait(path):
    try:
        tree = ast.parse(path.read_text(encoding="utf-8"))
    except (OSError, SyntaxError, UnicodeError):
        return False
    return any(isinstance(node, ast.Call) and isinstance(node.func, ast.Name)
               and node.func.id == "LaunchConfiguration" and node.args
               and isinstance(node.args[0], ast.Constant) and node.args[0].value == "require_explicit_target"
               for node in ast.walk(tree))


def prepare_hardware(*, only_if_idle=False):
    """Serialize startup and check ownership again before opening any devices."""
    from fcntl import LOCK_EX, flock

    selected = os.getenv("FSS_MISSION_WORKSPACE", PUBLIC)
    if selected not in (PRIVATE_CPP, PUBLIC):
        raise HardwareBusy({}, selected)
    work = Path.home() / selected
    if selected == PRIVATE_CPP:
        work /= "source"
    state = Path.home() / ".ros" / "mission-control"
    state.mkdir(mode=0o700, parents=True, exist_ok=True)
    with (state / "startup.lock").open("a") as lock:
        flock(lock, LOCK_EX)
        stacks = local_stacks()
        running = stacks["flight"] or stacks["imu"]
        environment = running[0] if running else {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1"}
        if any(ros_network(env) != ros_network(environment) for env in stacks["flight"] + stacks["imu"]):
            raise RuntimeError("이 Jetson에서 서로 다른 ROS 환경의 장치 프로세스가 실행 중입니다.")
        for key in ROS_ENV_KEYS:
            os.environ.pop(key, None)
        os.environ.update(environment)
        if stacks["flight"]:
            return "기존 기체 소프트웨어 연결"
        if stacks["imu"] and environment.get("ROS_LOCALHOST_ONLY", "0") != "1":
            raise HardwareBusy(environment, "공유 ROS 네트워크의 IMU")
        if only_if_idle and stacks["imu"]:
            return "기존 IMU 연결"
        profile = private_base()
        if profile and not profile.is_file():
            raise RuntimeError("실험실 기체 설정 필요 · cd ~/FSS_FSW && bash scripts/cpp_fsw.sh build")
        install = Path.home() / selected / "install"
        feature = "share/ament_index/resource_index/fss_mission_control/fss_gnc"
        if not any((prefix / feature).is_file() and
                   "require_explicit_target" in (prefix / feature).read_text().splitlines()
                   for prefix in (install / "fss_gnc", install)):
            raise RuntimeError("목표 전송 대기를 지원하는 Jetson C++ FSW로 업데이트하고 빌드하세요.")
        launch = install / "fss_bringup/share/fss_bringup/launch"
        if not launch.is_dir():
            launch = install / "share/fss_bringup/launch"
        launch_files = [launch / "fss_full.launch.py", launch / "gnc.launch.py"]
        if profile:
            launch_files.append(profile)
        if not all(forwards_target_wait(path) for path in launch_files):
            raise RuntimeError("기체 런처에 목표 대기 설정이 없습니다. Jetson FSW를 업데이트하세요.")
        if only_if_idle:
            # A second SSH client may connect just after a launcher has failed.
            # An explicit operator retry can still run immediately.
            attempt = state / "last-auto-start"
            if attempt.exists() and 0 <= time.time() - attempt.stat().st_mtime < 10:
                raise RuntimeError("최근 기동 요청이 있습니다. 실행 상태를 확인하고 FSW 시작을 누르세요.")
            attempt.touch()

        # Keep the stack alive across SSH/UI reconnects. Concurrent sessions see
        # the same ros2 launch process under the startup lock.
        def start(command, filename):
            with (state / filename).open("ab") as log:
                child = subprocess.Popen(command, cwd=work, stdin=subprocess.DEVNULL,
                                         stdout=log, stderr=subprocess.STDOUT,
                                         start_new_session=True)
            time.sleep(0.35)
            if child.poll() is not None:
                raise RuntimeError(f"기동 실패: {state / filename}")
            return child

        if not stacks["imu"] and (Path.home() / "xsens_mti_ros2_driver/install/setup.bash").is_file():
            start(["bash", "scripts/launch/start_imu.sh"], "imu.log")
        flight = ([str(profile), "use_sim:=false"] if profile
                  else ["fss_bringup", "fss_full.launch.py", "gnc_backend:=cpp"])
        start(["ros2", "launch", *flight, "require_explicit_target:=true"], "flight.log")
        return "기체 소프트웨어 기동"


def prepare_session(monitor_only=False, domain=None, *, start_if_idle=False):
    """Select the ROS environment, optionally starting an idle native Jetson on connection."""
    selected = os.getenv("FSS_MISSION_WORKSPACE", PUBLIC)
    reason = ""
    try:
        stacks = local_stacks(monitor=True, domain=domain)
        environment = stacks["monitor"]
        if selected == PRIVATE:
            reason = f"{selected} 모니터링 전용"
        elif monitor_only:
            reason = "모니터링 전용"
        elif not stacks["flight"] and not stacks["imu"]:
            environment = {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1"}
        elif any(ros_network(env) != ros_network(environment) for env in stacks["flight"] + stacks["imu"]):
            reason = "서로 다른 ROS 환경 · 모니터링 전용"
        elif not stacks["flight"] and environment.get("ROS_LOCALHOST_ONLY", "0") != "1":
            reason = "공유 ROS 네트워크의 IMU · 모니터링 전용"
    except HardwareBusy as busy:
        environment, reason = busy.environment, str(busy)
    for key in ROS_ENV_KEYS:
        os.environ.pop(key, None)
    os.environ.update(environment)
    if domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(domain)
    if (start_if_idle and not reason and not monitor_only and domain is None
            and selected in (PUBLIC, PRIVATE_CPP) and not stacks["flight"] and not stacks["imu"]):
        try:
            return prepare_hardware(only_if_idle=True), ""
        except HardwareBusy as busy:
            for key in ROS_ENV_KEYS:
                os.environ.pop(key, None)
            os.environ.update(busy.environment)
            return str(busy), str(busy)
        except (OSError, RuntimeError) as exc:
            return f"FSW 자동 시작 실패 · {exc}", ""
    return reason or "기체 연결", reason


def native_overlay_changed(reason):
    selected = os.environ.get("FSS_MISSION_WORKSPACE")
    if selected not in (PRIVATE_CPP, PUBLIC):
        return False
    alternate = PUBLIC if selected == PRIVATE_CPP else PRIVATE_CPP
    if reason != str(HardwareBusy({}, alternate)):
        return False
    return select_workspace(os.environ.get("FSS_MISSION_PREFERRED", PUBLIC)) == alternate
