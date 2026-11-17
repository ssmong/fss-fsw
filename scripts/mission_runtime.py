"""Select or start the flight stack on the Jetson hosting this SSH session."""

import os
import subprocess
import time
from pathlib import Path, PurePosixPath


FSS_NODES = {
    "manager_node", "valve_driver_node", "vicon_bridge_node", "imu_bridge_node",
    "ekf_node", "gnc_node", "plant_model_node", "arm_driver_node",
}
ROS_ENV_KEYS = ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION",
                "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE")


class HardwareBusy(RuntimeError):
    def __init__(self, environment, owner="self_tossing_hils"):
        super().__init__(f"{owner} 사용 중 · 모니터링만 가능")
        self.environment = environment


PUBLIC, PRIVATE = "fss-fsw", "FSS_FSW"


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
    # Test exact path components: 'FSS_FSW' and 'fss-fsw' are distinct Linux checkouts.
    program = program_path(argv)
    for project in (PUBLIC, PRIVATE):
        if project in program.parts:
            return project
    if launch_package(argv) in ("fss_bringup", "hardware_runtime"):
        for path in env.get("AMENT_PREFIX_PATH", "").split(":"):
            for project in (PUBLIC, PRIVATE):
                if project in PurePosixPath(path).parts:
                    return project


def process_role(argv, preferred=None, environ=None):
    if preferred is None:
        preferred = os.getenv("FSS_MISSION_PREFERRED", PUBLIC)
    name, package = executable_name(argv), launch_package(argv)
    program = program_path(argv)
    if (package == "self_tossing_hils" or name in ("capture_executor", "paper_toss_node")
            or ({"self_tossing_hils", "self_tossing_hardware"}.intersection(program.parts) and name in
                ("air_bearing", "arm_tracking", "pid_tuning", "pd_sweep", "run_air_bearing",
                 "run_air_bearing.sh", "run_joint_pid.sh", "test_arm_tracking.sh",
                 "run_arm_reference_ros", "record_gripper_close", "test_gripper_current",
                 "benchmark_capture_ros", "arm_executor", "tracking_executor", "motion_tool"))):
        return "competing"
    if name in ("xsens_mti_node", "xsens_mti_driver") or package == "xsens_mti_ros2_driver":
        return "imu"
    if name in FSS_NODES or package in ("fss_bringup", "hardware_runtime") or "hardware_runtime" in program.parts:
        project = checkout_for(argv, environ or {})
        if project and project != preferred:
            return "competing"
        # Public checkout has no hardware_runtime implementation.
        if preferred == PUBLIC and (package == "hardware_runtime" or "hardware_runtime" in program.parts):
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
            if not program.is_absolute() and len(program.parts) > 1:
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


def select_workspace(preferred, home=None, proc=Path("/proc")):
    home = home or Path.home()
    alternate = PRIVATE if preferred == PUBLIC else PUBLIC
    installed = [project for project in (preferred, alternate)
                 if (home / project / "install/setup.bash").is_file()]
    if not installed:
        raise RuntimeError("Jetson에서 FSS 빌드를 찾을 수 없습니다.")
    active = {checkout_for(argv, env) for argv, env in device_processes(proc)
              if process_role(argv, preferred, env) in ("flight", "competing")}
    return next((name for name in installed if name in active), installed[0])


def local_service_owner(name, proc=Path("/proc")):
    """A visible server must belong to the selected local flight stack."""
    owned = False
    for argv, env in device_processes(proc):
        if process_role(argv, environ=env) == "competing":
            return False
        if (executable_name(argv) == name
                and env.get("ROS_DOMAIN_ID", "0") == os.environ.get("ROS_DOMAIN_ID", "0")
                and env.get("ROS_LOCALHOST_ONLY", "0") == os.environ.get("ROS_LOCALHOST_ONLY", "0")):
            owned = True
    return owned


def local_stacks(proc=Path("/proc")):
    stacks = {"flight": [], "imu": []}
    for argv, env in device_processes(proc):
        role = process_role(argv, environ=env)
        ros_env = {key: env[key] for key in ROS_ENV_KEYS if key in env}
        if role == "competing":
            owner = checkout_for(argv, env)
            label = "FSS_FSW/hardware_runtime" if owner == PRIVATE or launch_package(argv) == "hardware_runtime" else owner
            raise HardwareBusy(ros_env, label or "self_tossing_hils")
        stacks[role].append(ros_env)
    return stacks


def prepare_hardware():
    from fcntl import LOCK_EX, flock

    work = Path.home() / os.getenv("FSS_MISSION_WORKSPACE", PUBLIC)
    state = Path.home() / ".ros" / "mission-control"
    state.mkdir(mode=0o700, parents=True, exist_ok=True)
    with (state / "startup.lock").open("a") as lock:
        flock(lock, LOCK_EX)
        stacks = local_stacks()
        running = stacks["flight"] or stacks["imu"]
        environment = running[0] if running else {"ROS_DOMAIN_ID": "0", "ROS_LOCALHOST_ONLY": "1"}
        def network(env):
            return (env.get("ROS_DOMAIN_ID", "0"), env.get("ROS_LOCALHOST_ONLY", "0"),
                    env.get("RMW_IMPLEMENTATION", ""))
        if any(network(env) != network(environment) for env in stacks["flight"] + stacks["imu"]):
            raise RuntimeError("이 Jetson에서 서로 다른 ROS 환경의 장치 프로세스가 실행 중입니다.")
        for key in ROS_ENV_KEYS:
            os.environ.pop(key, None)
        os.environ.update(environment)
        if stacks["flight"]:
            return "기존 기체 소프트웨어 연결"

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

        if work.name == PUBLIC and not stacks["imu"] and (Path.home() / "xsens_mti_ros2_driver/install/setup.bash").is_file():
            start(["bash", "scripts/launch/start_imu.sh"], "imu.log")
        args = ["ros2", "launch", "fss_bringup", "fss_full.launch.py"]
        # Private bringup owns Xsens acquisition itself. An existing standalone
        # driver must not be started a second time by the private launcher.
        if work.name == PRIVATE and stacks["imu"]:
            args.append("start_imu_driver:=false")
        start(args, "flight.log")
        return "기체 소프트웨어 기동"


def prepare_session(monitor_only=False, domain=None):
    try:
        preferred = os.getenv("FSS_MISSION_PREFERRED", PUBLIC)
        selected = os.getenv("FSS_MISSION_WORKSPACE", preferred)
        fallback = selected != preferred
        if monitor_only or fallback or domain is not None:
            stacks = local_stacks()
            running = stacks["flight"] or stacks["imu"]
            if running:
                for key in ROS_ENV_KEYS:
                    os.environ.pop(key, None)
                os.environ.update(running[0])
            if domain is not None:
                os.environ["ROS_DOMAIN_ID"] = str(domain)
            if fallback:
                return f"{selected} 모니터링 연결", f"{selected} 모니터링 전용"
            return ("모니터링 전용 연결", "모니터링 전용") if monitor_only else ("기체 연결", "")
        return prepare_hardware(), ""
    except HardwareBusy as busy:
        for key in ROS_ENV_KEYS:
            os.environ.pop(key, None)
        os.environ.update(busy.environment)
        return str(busy), str(busy)
