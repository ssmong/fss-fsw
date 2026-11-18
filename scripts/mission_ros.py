"""ROS subscriptions and services used by FSS Mission Control, locally or over SSH."""

import json
import os
import select
import sys
import time
import uuid
from types import SimpleNamespace


class MissionActions:
    """Run one operator action at a time through the manager and valve services."""

    IDLE, PRESSURIZING, READY, ACTIVE, SAFE_HOLD, ESTOP = range(6)

    def __init__(self, call, reply, reject=None):
        self.call = call
        self.reply = reply
        self.reject = reject or reply
        self.mode = None
        self.mode_at = self.health_at = 0.0
        self.air_open = False
        self.steps = self.future = self.step = None
        self.cancel_for_off = False

    def observe(self, name, msg):
        if name == "mode":
            self.mode, self.mode_at = msg.mode, time.monotonic()
        elif name == "health":
            self.air_open, self.health_at = msg.main_valve_open, time.monotonic()

    def _sequence(self, name, value):
        if name == "mode":
            yield "mode", value
            return "모드 변경"
        if name == "valve" and value is False:
            initial = self.mode
            if initial == self.ACTIVE:
                yield "mode", self.SAFE_HOLD
            yield "valve", False
            if initial in (self.PRESSURIZING, self.READY, self.ACTIVE, self.SAFE_HOLD) and self.mode != self.ESTOP:
                yield "mode", self.IDLE
            return "에어베어링 OFF"
        if self.mode is None or time.monotonic() - self.mode_at > 2.5:
            raise ValueError("기체 모드 수신 대기")
        if name == "valve":
            if self.mode in (self.SAFE_HOLD, self.ESTOP):
                raise ValueError("정지 모드에서 에어베어링을 열 수 없습니다.")
            prepare = self.mode in (self.IDLE, self.PRESSURIZING)
            if self.mode == self.IDLE:
                yield "mode", self.PRESSURIZING
            yield "valve", True
            if prepare:
                yield "wait_air", True
                yield "mode", self.READY
            return "에어베어링 ON"
        if name == "target":
            if self.mode in (self.SAFE_HOLD, self.ESTOP):
                raise ValueError("정지 모드에서 목표를 실행할 수 없습니다.")
            yield "target", value
            if self.mode == self.READY:
                yield "mode", self.ACTIVE
            return "목표 설정" if self.mode == self.ACTIVE else "목표 저장 · 에어베어링 ON 후 전송"
        if name == "controller":
            yield "controller", value
            return "제어기 적용"
        raise ValueError("알 수 없는 기체 명령")

    def request(self, name, value):
        if self.steps is not None:
            if name == "mode" and value == self.ESTOP:
                self.cancel_for_off = False
            elif name == "valve" and value is False:
                # Wait for the single outstanding service before closing the air.
                self.cancel_for_off = True
                return
            else:
                self.reject("기체 명령 처리 중")
                return
        self.steps = self._sequence(name, value)
        self.future = self.step = None
        self.tick()

    def tick(self):
        if self.steps is None:
            return
        now = time.monotonic()
        try:
            if self.step:
                if self.future is not None and self.future.done():
                    result = self.future.result()
                    if not result.success:
                        raise ValueError(result.message)
                    if self.step == "mode":
                        self.mode, self.mode_at = result.actual_mode, now
                elif self.step == "wait_air" and self.air_open and now - self.health_at < 1.0:
                    pass
                elif now < self.deadline:
                    return
                else:
                    raise ValueError("기체 응답 시간 초과 · 상태 확인")
            if self.cancel_for_off:
                self.steps = self._sequence("valve", False)
                self.cancel_for_off = False
            self.step, value = next(self.steps)
            self.deadline = now + 3.0
            self.future = None if self.step == "wait_air" else self.call(self.step, value)
        except StopIteration as complete:
            self.steps = None
            self.reply(str(complete.value))
        except Exception as exc:
            self.steps = None
            self.reject(str(exc))
            if self.cancel_for_off:
                self.cancel_for_off = False
                self.request("valve", False)


class ArmRegistration:
    """Read the tracking-frame calibration from the active link publisher."""

    def __init__(self, node, receive, trigger):
        self.node, self.receive = node, receive
        self.client = node.create_client(trigger, "/arm/kinematics_model")
        self.request = trigger.Request()
        self.owner = self.pending = None
        self.loaded = False
        self.retry_at = 0.0

    def poll(self):
        links = self.node.get_publishers_info_by_topic("/arm/link_poses_base")
        camera = self.node.get_publishers_info_by_topic("/sensors/vicon_state")
        owner = None
        if (len(links) == len(camera) == 1 and links[0].node_name == "arm_kinematics_node"
                and links[0].node_namespace == "/" and camera[0].node_namespace == "/"
                and camera[0].node_name in ("vicon_bridge_node", "plant_model_node")
                and self.node.get_node_names().count("arm_kinematics_node") == 1):
            owner = bytes(links[0].endpoint_gid), bytes(camera[0].endpoint_gid)
        if owner != self.owner:
            if self.pending:
                self.pending.cancel()
            self.owner, self.pending, self.loaded, self.retry_at = owner, None, False, 0.0
            self.receive("arm_registration", None)
        if owner is None or self.loaded:
            return
        if self.pending and self.pending.done():
            try:
                response = self.pending.result()
                if response.success and len(response.message) <= 65536:
                    info = json.loads(response.message)
                    cfg = info["configuration"]
                    if (info["schema_version"] == 1 and cfg["frame_id"] == "table" and cfg["base_frame_id"] == "base_link"
                            and cfg["navigation_topic"] in ("/navigation/state", "/sensors/vicon_state")):
                        self.receive("arm_registration", [*cfg["navigation_to_base"], cfg["vicon_only_alignment_ms"],
                                                          cfg["maximum_sample_skew_s"], cfg["feedback_timeout_s"]])
                        self.loaded = True
            except Exception:
                # Optional calibration may disappear during a driver restart.
                pass
            self.pending = None
        now = time.monotonic()
        if not self.loaded and now >= self.retry_at:
            if self.pending:
                self.pending.cancel()
                self.pending = None
            if self.client.service_is_ready():
                self.retry_at = now + 3.0
                try:
                    self.pending = self.client.call_async(self.request)
                except RuntimeError:
                    pass


class ValveOutput:
    topic = "/actuators/valve_commands"
    status_topic = "/actuators/valve_status"

    def __init__(self, node, receive, parameters):
        self.node, self.receive = node, receive
        self.client = node.create_client(parameters, "/valve_driver_node/get_parameters")
        self.range_request = parameters.Request(names=["plc.value_range"])
        self.publisher_id = self.maximum = self.pending = None
        self.next_query = 0.0

    @staticmethod
    def is_driver(endpoint):
        return endpoint.node_name == "valve_driver_node" and endpoint.node_namespace == "/"

    def output(self, header, duties):
        self.receive("valve_output", SimpleNamespace(header=header, duty_cycles=duties))

    def native(self, msg):
        # In older workspaces GNC publishes *demand* on the very same topic.
        publishers = self.node.get_publishers_info_by_topic(self.topic)
        if len(publishers) == 1 and self.is_driver(publishers[0]):
            self.output(msg.header, list(msg.duty_cycles))

    def legacy(self, msg):
        # Only the legacy driver subscribes to valve_commands. Its status
        # holds a copy of its last successful software write in integer units.
        if not any(self.is_driver(sub) for sub in self.node.get_subscriptions_info_by_topic(self.topic)):
            return
        publishers = self.node.get_publishers_info_by_topic(self.status_topic)
        if len(publishers) != 1 or not self.is_driver(publishers[0]):
            return
        maximum = self.legacy_maximum(bytes(publishers[0].endpoint_gid))
        values = msg.applied_values
        duties = None
        if len(values) == 8:
            if all(value == 0 for value in values):
                duties = [0.0] * 8
            elif maximum and all(0 <= value <= maximum for value in values):
                duties = [value / maximum for value in values]
        self.output(msg.header, duties)

    def legacy_maximum(self, publisher_id):
        if publisher_id != self.publisher_id:
            if self.pending:
                self.pending.cancel()
            self.publisher_id, self.maximum, self.pending = publisher_id, None, None
            self.next_query = 0.0
        if self.pending and self.pending.done():
            try:
                response = self.pending.result()
                if response and response.values:
                    limits = response.values[0].integer_array_value
                    if len(limits) == 2 and limits[0] == 0 and limits[1] > 0:
                        self.maximum = limits[1]
            except Exception:
                pass  # A driver restart can interrupt an in-flight request.
            self.pending = None
        now = time.monotonic()
        if self.maximum is None and now >= self.next_query and self.client.service_is_ready():
            if self.pending:
                self.pending.cancel()
            self.pending = self.client.call_async(self.range_request)
            self.next_query = now + 3.0
        return self.maximum


def open_node(receive, reply, args=None, reject=None):
    import rclpy
    from fss_interfaces.msg import (
        SpacecraftState, SystemHealth, SystemMode, TrajectoryReference, ValveCommands, ValveStatus,
    )
    from fss_interfaces.srv import MainValveControl, SetController, SetSystemMode
    try:
        from fss_interfaces.srv import SetTarget
    except ImportError:
        SetTarget = None  # Older workspaces still support monitoring and air control.
    from rcl_interfaces.msg import Log
    from rcl_interfaces.srv import GetParameters
    from geometry_msgs.msg import PoseArray, PoseStamped
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, JointState
    from std_msgs.msg import String
    from std_srvs.srv import Trigger

    rclpy.init(args=args)
    node = rclpy.create_node("fss_mission_" + uuid.uuid4().hex)
    def received(name, msg):
        if name in ("arm_joints", "arm_base", "arm_table", "arm_tcp"):
            stamp = msg.header.stamp
            age = node.get_clock().now().nanoseconds - (stamp.sec * 10**9 + stamp.nanosec)
            if not 0 <= age <= 500_000_000:
                return
        actions.observe(name, msg)
        receive(name, msg)
        if name == "valve":
            valve_output.legacy(msg)

    # Standalone SSH code includes the reader; local ROS imports its sibling.
    read_plans = globals().get("load_table_plans")
    read_elapsed = globals().get("paper_plan_elapsed")
    read_run = globals().get("paper_run_files")
    tracker_type = globals().get("PaperTracking")
    catalog_type = globals().get("PlanCatalog")
    if read_plans is None:
        from mission_plan import load_table_plans as read_plans
        from mission_plan import paper_plan_elapsed as read_elapsed
        from mission_plan import paper_run_files as read_run
        from mission_tracking import PaperTracking as tracker_type
        from mission_plan import PlanCatalog as catalog_type
    catalog = catalog_type()
    catalog.refresh()
    tracking = tracker_type(catalog.plans, catalog.signatures, node.get_clock().now().nanoseconds / 1e9)
    receive("plans", catalog.plans)
    receive("plan_defaults", catalog.defaults)
    last_run = None
    published_extra = None
    last_paper_status = None
    last_paper_status_at = 0.0
    last_elapsed = None

    def refresh_plans():
        nonlocal published_extra
        if not catalog.refresh():
            return
        tracking.refresh_catalog(catalog.plans, catalog.signatures)
        published_extra = tracking.extra_plan
        receive("plans", catalog.plans + ([published_extra] if published_extra else []))
        receive("plan_defaults", catalog.defaults)
        if last_paper_status and time.monotonic() - last_paper_status_at <= 2.5:
            status = last_paper_status
            if not status.get("log_directory"):
                tracking.poll(status, node.get_clock().now().nanoseconds / 1e9, None)
            receive("paper_trial", [status.get("plan_id", ""), last_elapsed, tracking.active_key])

    node.create_timer(1.0, refresh_plans)

    def paper_status(msg):
        nonlocal last_run, published_extra, last_paper_status, last_paper_status_at, last_elapsed
        try:
            status = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        terminal = isinstance(status, dict) and status.get("phase") in ("STOPPED", "MISSED", "FAULT")
        if not terminal:
            receive("paper_status", status)
        plan_id = status.get("plan_id") if isinstance(status, dict) and status.get("frame_id") == "table" else None
        if isinstance(plan_id, str) and plan_id:
            last_paper_status, last_paper_status_at = status, time.monotonic()
            source = read_run(status)
            identity = (source[0], plan_id) if source else None
            catalog_changed = False
            if identity and identity != last_run:
                last_run = identity
                catalog_changed = catalog.refresh()
                tracking.set_catalog(catalog.plans, catalog.signatures)
            errors = tracking.poll(status, node.get_clock().now().nanoseconds / 1e9, source)
            if catalog_changed or published_extra is not tracking.extra_plan:
                published_extra = tracking.extra_plan
                receive("plans", tracking.plans + ([published_extra] if published_extra else []))
                receive("plan_defaults", catalog.defaults)
            last_elapsed = read_elapsed(status, source=source) if source else None
            receive("paper_trial", [plan_id, last_elapsed, tracking.active_key])
            for sample in errors:
                receive("tracking_error", sample)
        else:
            last_paper_status, last_elapsed = None, None
            if published_extra is not None:
                published_extra = None
                receive("plans", tracking.plans)
                receive("plan_defaults", catalog.defaults)
            receive("paper_trial", ["", None, None])
        # The recorder closes on terminal status, after the final clock and errors.
        if terminal:
            receive("paper_status", status)

    node.create_subscription(String, "/paper_toss/status", paper_status, qos_profile_sensor_data)
    registration = ArmRegistration(node, receive, Trigger)
    node.create_timer(1.0, registration.poll)
    valve_output = ValveOutput(node, receive, GetParameters)
    node.create_subscription(ValveCommands, valve_output.topic, valve_output.native, 10)

    for name, topic, msg_type, depth in (
        ("vicon", "/sensors/vicon_state", SpacecraftState, 10),
        ("ekf", "/navigation/state", SpacecraftState, 10),
        ("valve", "/actuators/valve_status", ValveStatus, 10),
        ("ref", "/gnc/trajectory_ref", TrajectoryReference, 10),
        ("mode", "/system/mode", SystemMode, 10),
        ("health", "/system/health", SystemHealth, 10),
        ("imu", "/sensors/imu", Imu, 20),
        ("arm_joints", "/arm/joint_states", JointState, qos_profile_sensor_data),
        ("arm_driver", "/arm/status", String, qos_profile_sensor_data),
        ("arm_table", "/arm/link_poses", PoseArray, qos_profile_sensor_data),
        ("arm_tcp", "/arm/end_effector_pose", PoseStamped, qos_profile_sensor_data),
        ("arm_base", "/arm/link_poses_base", PoseArray, qos_profile_sensor_data),
        ("log", "/rosout", Log, 50),
    ):
        node.create_subscription(msg_type, topic, lambda msg, n=name: received(n, msg), depth)

    services = {
        "controller": (node.create_client(SetController, "/gnc/set_controller"), SetController, "controller_type"),
        "mode": (node.create_client(SetSystemMode, "/system/set_mode"), SetSystemMode, "requested_mode"),
        "valve": (node.create_client(MainValveControl, "/actuators/main_valve"), MainValveControl, "open"),
    }

    if SetTarget is not None:
        services["target"] = (node.create_client(SetTarget, "/gnc/set_target"), SetTarget, None)

    def call(name, value):
        if globals().get("read_only"):
            raise ValueError(globals()["read_only"])
        if name not in services:
            raise ValueError("/gnc/set_target 미설치 · Jetson GNC 업데이트 필요")
        client, srv_type, field = services[name]
        if not client.service_is_ready():
            raise ValueError(f"{client.srv_name} 서비스 미연결")
        owner = {"mode": "manager_node", "valve": "valve_driver_node",
                 "target": "gnc_node", "controller": "gnc_node"}[name]
        owns_service = globals().get("local_service_owner")
        if owns_service and not owns_service(owner):
            raise ValueError(f"이 Jetson에서 {owner}를 찾을 수 없습니다.")
        if node.get_node_names().count(owner) > 1:
            raise ValueError("같은 ROS 도메인에서 기체가 중복되어 명령 대상을 구분할 수 없습니다.")
        req = srv_type.Request()
        if name == "target":
            req.controller_type, req.x, req.y, req.yaw = value
        else:
            setattr(req, field, value)
        return client.call_async(req)

    actions = MissionActions(call, reply, reject)
    node.create_timer(0.05, actions.tick)
    return node, actions.request


def serve(hub=None):
    import rclpy
    from rosidl_runtime_py.convert import message_to_ordereddict

    command_owner = None
    command_error = "reply_error"

    def emit(kind, data):
        nonlocal command_owner
        if hub is not None:
            if kind in ("reply", "reply_error", "body_reply", "body_error", "toss_reply", "toss_error"):
                if kind == "reply_error" and getattr(getattr(request, "__self__", None), "cancel_for_off", False):
                    hub.publish("log", {"level": 40, "name": "Mission Control", "msg": data})
                    return
                if command_owner is not None:
                    hub.publish(kind, data, target=command_owner)
                    command_owner = None
            else:
                hub.publish(kind, data)
        else:
            print(json.dumps([kind, data], separators=(",", ":")), flush=True)

    display_last = {}
    recording = False

    def forward(name, msg):
        nonlocal recording
        if hub is not None:
            recording = hub.recording
        if name == "paper_status" and globals().get("toss_bench"):
            toss_bench.observe(msg)
        if not recording and name in ("vicon", "ekf", "ref", "arm_joints", "arm_driver", "arm_table", "arm_base", "arm_tcp"):
            if name == "arm_driver":
                sample_ns = time.monotonic_ns()
            else:
                stamp = msg.header.stamp
                sample_ns = stamp.sec * 10**9 + stamp.nanosec
                sample_ns = sample_ns or time.monotonic_ns()
            # Link poses and TCP from one kinematics update share this stamp.
            bucket = sample_ns // 50_000_000
            if display_last.get(name) == bucket:
                return
            display_last[name] = bucket
        # The dashboard uses IMU arrivals only to measure the incoming rate.
        if name == "imu" and not recording:
            data = {}
        elif name in ("plans", "plan_defaults", "paper_trial", "paper_status", "tracking_error", "arm_registration"):
            data = msg
        elif name == "valve_output":
            data = {"duty_cycles": msg.duty_cycles}
            if recording:
                data["header"] = message_to_ordereddict(msg.header)
        elif name == "valve":
            data = {"commanded_duty_cycles": list(msg.commanded_duty_cycles),
                    "main_valve_open": msg.main_valve_open, "modbus_rtt_ms": msg.modbus_rtt_ms}
            if recording:
                data["header"] = message_to_ordereddict(msg.header)
        else:
            data = message_to_ordereddict(msg)
        emit(name, data)

    node, request = open_node(forward, lambda text: emit("reply", text),
                              reject=lambda text: emit("reply_error", text))
    if status := globals().get("startup_status"):
        failed = status.startswith("FSW 자동 시작 실패")
        emit("log", {"level": 40 if failed else 20, "name": "Mission Control", "msg": status})
        if failed and hub is None:
            emit("reply_error", status)
    heartbeat = 0.0
    pending = b""
    bench = globals().get("toss_bench")
    passive = globals().get("toss_monitor", False)
    domain = globals().get("toss_domain")
    next_workflow_check = 0.0
    toss_pool = toss_future = None
    if bench:
        from concurrent.futures import ThreadPoolExecutor
        toss_pool = ThreadPoolExecutor(max_workers=1)

    def verify_toss_service(service):
        publishers = node.get_publishers_info_by_topic("/paper_toss/status")
        if (len(publishers) != 1 or publishers[0].node_name != "paper_toss_executor"
                or publishers[0].node_namespace != "/"):
            raise RuntimeError("실험 상태 발행자를 확인할 수 없습니다.")
        owners = []
        for name, namespace in node.get_node_names_and_namespaces():
            if any(s == service and "std_srvs/srv/Trigger" in types
                   for s, types in node.get_service_names_and_types_by_node(name, namespace)):
                owners.append((name, namespace))
        if owners != [("paper_toss_executor", "/")]:
            raise RuntimeError("실험 서비스 소유자를 확인할 수 없습니다.")
    next_idle_check = 0.0
    idle = False
    attached_writable = False

    def can_start_body(names):
        nonlocal next_idle_check, idle
        if (globals().get("read_only") or "prepare_hardware" not in globals()
                or os.getenv("FSS_MISSION_WORKSPACE") not in (PUBLIC, PRIVATE_CPP)
                or os.getenv("ROS_LOCALHOST_ONLY") != "1" or os.getenv("ROS_DOMAIN_ID", "0") != "0"
                or globals().get("_starting_flight") is not None or FSS_NODES.intersection(names)):
            next_idle_check = 0.0
            return False
        # Another bridge's launcher can be alive before its ROS nodes appear.
        now = time.monotonic()
        if now >= next_idle_check:
            try:
                idle = not local_stacks()["flight"]
            except (OSError, RuntimeError):
                idle = False
            next_idle_check = now + 5.0
        return idle

    try:
        while rclpy.ok():
            if hub is not None and hub.expired() and command_owner is None and toss_future is None:
                break
            if toss_future and toss_future.done():
                try:
                    emit("toss_reply", toss_future.result())
                except Exception as exc:
                    emit("toss_error", str(exc) or type(exc).__name__)
                toss_future = None
                heartbeat = 0.0
            if time.monotonic() >= heartbeat:
                names = node.get_node_names()
                peers = [name for name in names if name.startswith("fss_mission_")]
                check_start = globals().get("startup_failure")
                if check_start and (error := check_start(names)):
                    emit("log", {"level": 40, "name": "Mission Control", "msg": error})
                    if hub is None:
                        emit("reply_error", error)
                toss_actions, toss_state = (), ""
                body_allowed = False
                if bench:
                    try:
                        toss_actions, toss_state, body_allowed = bench.capabilities(names, domain)
                    except (OSError, RuntimeError, ValueError) as exc:
                        toss_state = str(exc)
                else:
                    body_allowed = can_start_body(names)
                if (globals().get("follow_workflow") and toss_future is None
                        and time.monotonic() >= next_workflow_check):
                    next_workflow_check = time.monotonic() + 3
                    next_toss = detect_toss_workflow(domain)
                    env = active_toss_env() if bench and next_toss and domain is None else None
                    if (bool(bench) != next_toss or (env is not None and ros_network(env) != ros_network(os.environ))):
                        raise SystemExit(77)
                emit("connected", {"observer": node.get_name(), "peers": peers,
                                   "workspace": os.getenv("FSS_MISSION_WORKSPACE"),
                                   "private_arm": globals().get("private_arm", False),
                                   "can_start_body": body_allowed and not passive and toss_future is None,
                                   "workflow": "toss" if bench else "control",
                                   "toss_actions": toss_actions if toss_future is None and not passive else (),
                                   "toss_state": toss_state, "read_only": ("모니터링 전용" if passive else "")
                                   if bench else globals().get("read_only", "")})
                heartbeat = time.monotonic() + 1
            commands = hub.poll() if hub is not None else []
            if hub is None and select.select([sys.stdin], [], [], 0)[0]:
                data = os.read(sys.stdin.fileno(), 4096)
                if not data:
                    break
                pending += data
                while b"\n" in pending:
                    command, pending = pending.split(b"\n", 1)
                    name, value = json.loads(command)
                    commands.append((None, name, value))
            for owner, name, value in commands:
                if name == "__attach__" and hub is not None:
                    if not attached_writable:
                        attached_writable = True
                        if (not bench and domain is None and not toss_bench_installed()
                                and can_start_body(node.get_node_names())):
                            try:
                                emit("log", {"level": 20, "name": "Mission Control",
                                             "msg": prepare_hardware(only_if_idle=True)})
                            except (OSError, RuntimeError, ValueError) as exc:
                                hub.publish("body_error", str(exc), target=owner)
                    continue
                error_kind = "toss_error" if name.startswith("toss_") else (
                    "body_error" if name == "start_body" else "reply_error")
                if hub is not None and name != "record_stream":
                    if command_owner is not None:
                        urgent = not bench and ((name == "mode" and value == MissionActions.ESTOP)
                                                 or (name == "valve" and value is False))
                        if not urgent:
                            hub.publish(error_kind, "다른 명령 처리 중", target=owner)
                            continue
                        hub.publish(command_error, "다른 연결의 정지 요청으로 명령 종료", target=command_owner)
                    command_owner, command_error = owner, error_kind
                if name == "record_stream":
                    recording = value is True
                elif bench:
                    if name == "start_body":
                        try:
                            if (value is not True or passive or toss_future is not None
                                    or not bench.capabilities(node.get_node_names(), domain)[2]):
                                raise ValueError("기체 실행 가능 상태를 확인하세요.")
                            emit("body_reply", "기체 제어 FSW 전환 중")
                            raise SystemExit(76)
                        except (OSError, RuntimeError, ValueError) as exc:
                            emit("body_error", str(exc))
                    elif passive or name not in TOSS_COMMANDS or toss_future is not None:
                        emit("toss_error", "실험 명령 또는 처리 상태를 확인하세요.")
                    else:
                        toss_future = toss_pool.submit(bench.request, name, value, node.get_node_names, verify_toss_service)
                elif name == "start_body":
                    try:
                        if value is not True or not can_start_body(node.get_node_names()):
                            raise ValueError("기체 실행 상태를 확인하세요.")
                        emit("body_reply", prepare_hardware())
                    except (OSError, RuntimeError, ValueError) as exc:
                        emit("body_error", str(exc))
                    next_idle_check = 0.0
                else:
                    request(name, value)
            rclpy.spin_once(node, timeout_sec=0.02)
    except (BrokenPipeError, KeyboardInterrupt):
        pass
    finally:
        if bench:
            bench.cancelled.set()
            toss_pool.shutdown(wait=True, cancel_futures=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    if "run_shared" in globals():
        run_shared(serve)
    else:
        serve()
