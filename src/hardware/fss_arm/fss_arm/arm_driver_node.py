"""FSS 운용 모드에 연동되는 로봇팔 위치·속도·전류 명령 노드."""

import json
import time

import numpy as np
import rclpy
from fss_arm_msgs.msg import JointCurrent
from fss_arm_msgs.srv import SetCommandMode
from fss_interfaces.msg import SystemMode
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from fss_arm.commands import (
    COMMAND_MODES,
    JOINT_NAMES,
    CommandGate,
    JointMapping,
    decode_current,
    encode_current,
    position_profiles,
    return_delay_ticks,
    vector3,
    watchdog_ticks,
)
from fss_arm.dynamixel_bus import DynamixelBus


class ArmDriverNode(Node):
    """실HW 연결은 확인된 설정에서만, 출력은 별도 enable 요청 후에 허용한다."""

    def __init__(self) -> None:
        super().__init__("arm_driver_node")
        p = Parameter.Type
        schema = {
            "dry_run": p.BOOL,
            "hardware_confirmed": p.BOOL,
            "port": p.STRING,
            "baudrate": p.INTEGER,
            "return_delay_us": p.INTEGER,
            "command_mode": p.STRING,
            "current_limits_a": p.DOUBLE_ARRAY,
            "position_profile_velocity": p.DOUBLE_ARRAY,
            "position_profile_acceleration": p.DOUBLE_ARRAY,
            "model_numbers": p.INTEGER_ARRAY,
            "control_rate_hz": p.DOUBLE,
            "command_timeout_s": p.DOUBLE,
            "mode_timeout_s": p.DOUBLE,
            "bus_watchdog_s": p.DOUBLE,
            "ids": p.INTEGER_ARRAY,
            "directions": p.INTEGER_ARRAY,
            "gear_ratios": p.DOUBLE_ARRAY,
            "zero_ticks": p.INTEGER_ARRAY,
            "velocity_limits": p.DOUBLE_ARRAY,
            "lower_limits": p.DOUBLE_ARRAY,
            "upper_limits": p.DOUBLE_ARRAY,
            "acceleration_limits": p.DOUBLE_ARRAY,
            "initial_position_rad": p.DOUBLE_ARRAY,
        }
        for key, kind in schema.items():
            self.declare_parameter(key, kind, ParameterDescriptor(read_only=True))
        cfg = {key: self.get_parameter(key).value for key in schema}
        self.mapping = JointMapping(
            **{key: tuple(cfg[key]) for key in JointMapping.__dataclass_fields__}
        )
        self.gate = CommandGate(cfg["mode_timeout_s"], cfg["command_timeout_s"])
        rate = cfg["control_rate_hz"]
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError("invalid control rate")
        effective_watchdog = watchdog_ticks(cfg["bus_watchdog_s"]) * 0.02
        return_delay_ticks(cfg["return_delay_us"])
        if 1 / rate >= cfg["command_timeout_s"] or 1 / rate >= effective_watchdog:
            raise ValueError("control period must be shorter than both watchdogs")
        self.dry_run = cfg["dry_run"]
        self.command_mode = cfg["command_mode"]
        if self.command_mode not in COMMAND_MODES:
            raise ValueError("command_mode must be velocity, position or current")
        self.current_limits = vector3(cfg["current_limits_a"], "current limits")
        encode_current(self.mapping, np.zeros(3), self.current_limits)
        self.profile_velocity = vector3(
            cfg["position_profile_velocity"], "position profile velocity"
        )
        self.profile_acceleration = vector3(
            cfg["position_profile_acceleration"], "position profile acceleration"
        )
        position_profiles(self.mapping, self.profile_velocity, self.profile_acceleration)
        if np.any(
            self.profile_acceleration > vector3(cfg["acceleration_limits"], "acceleration limits")
        ):
            raise ValueError("position profile exceeds acceleration limits")
        self.position = vector3(cfg["initial_position_rad"], "initial position")
        self.mapping.check_position(self.position)
        self.velocity = np.zeros(3)
        self.current = np.zeros(3)
        self.command = np.zeros(3)
        self.enabled = False
        self.last_tick = time.monotonic()
        self.last_stamp_ns = -1
        self.enable_stamp_ns = -1
        self.period = 1 / rate
        self.timing = {
            "target_period_ms": self.period * 1000,
            "cycles": 0,
            "cycle_overruns": 0,
            "late_intervals": 0,
            "last_interval_ms": 0.0,
            "max_interval_ms": 0.0,
            "last_read_ms": 0.0,
            "max_read_ms": 0.0,
            "last_write_ms": 0.0,
            "max_write_ms": 0.0,
            "last_cycle_ms": 0.0,
            "max_cycle_ms": 0.0,
        }
        self.last_timing_publish = self.last_tick
        self.bus = None
        if not self.dry_run:
            if not cfg["hardware_confirmed"]:
                raise ValueError("hardware mapping/model/zero configuration is unconfirmed")
            if self.get_parameter("use_sim_time").value:
                raise ValueError("hardware execution cannot use simulation clock")
            self.bus = DynamixelBus(
                self.mapping,
                cfg["port"],
                cfg["baudrate"],
                cfg["model_numbers"],
                cfg["bus_watchdog_s"],
                return_delay_us=cfg["return_delay_us"],
                command_mode=self.command_mode,
                profile_velocity=tuple(self.profile_velocity),
                profile_acceleration=tuple(self.profile_acceleration),
            )
            self.bus.connect()
        self.state_pub = self.create_publisher(JointState, "/arm/joint_states", 10)
        self.status_pub = self.create_publisher(String, "/arm/status", 10)
        self.timing_pub = self.create_publisher(String, "/arm/timing", 10)
        self.current_pub = self.create_publisher(JointCurrent, "/arm/currents", 10)
        self.create_subscription(SystemMode, "/system/mode", self._mode, 1)
        self.create_subscription(JointState, "/arm/velocity_command", self._command, 1)
        self.create_subscription(JointState, "/arm/position_command", self._position_command, 1)
        self.create_subscription(JointCurrent, "/arm/current_command", self._current_command, 1)
        self.create_service(SetBool, "/arm/set_enabled", self._enable)
        self.create_service(SetCommandMode, "/arm/set_command_mode", self._set_command_mode)
        # /clock 정지에도 명령 만료와 정지를 처리할 수 있도록 steady timer를 사용한다.
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1 / rate, self._tick, clock=self.steady_clock)
        self.get_logger().info(f"arm ready: dry_run={self.dry_run}, mode={self.command_mode}")

    def _mode(self, msg: SystemMode) -> None:
        self.gate.update_mode(msg.mode == SystemMode.ACTIVE, time.monotonic())
        if self.enabled and not self.gate.active:
            self._trip("system mode left ACTIVE")

    def _command(self, msg: JointState) -> None:
        self._accept_command(msg, "velocity", msg.velocity, bool(msg.position or msg.effort))

    def _position_command(self, msg: JointState) -> None:
        """상대 관절각 rad 목표를 받는다."""
        self._accept_command(msg, "position", msg.position, bool(msg.velocity or msg.effort))

    def _current_command(self, msg: JointCurrent) -> None:
        """관절 +방향 기준 모터 전류 A 목표를 받는다."""
        self._accept_command(msg, "current", msg.current, False)

    def _accept_command(self, msg: object, mode: str, values: object, extra_fields: bool) -> None:
        """선택한 모드와 단위에 맞는 최신 3축 명령만 허용한다."""
        if not self.enabled:
            return
        try:
            stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds - stamp) / 1e9
            if not 0 <= age <= self.gate.command_timeout_s:
                raise ValueError("stale or future command")
            # 발행자를 enable 전에 준비할 때 DDS에 남아 있는 이전 세대 명령은 폐기한다.
            if stamp <= self.enable_stamp_ns:
                return
            if mode != self.command_mode:
                raise ValueError(f"{mode} command rejected in {self.command_mode} mode")
            if tuple(msg.name) != JOINT_NAMES or extra_fields:
                raise ValueError("command requires ordered names and selected field only")
            if stamp <= self.last_stamp_ns:
                raise ValueError("out-of-order command")
            command = vector3(values, f"{mode} command")
            if mode == "velocity":
                self.mapping.encode_velocity(command)
            elif mode == "position":
                self.mapping.encode_position(command)
            else:
                encode_current(self.mapping, command, self.current_limits)
            self.command = command
            self.last_stamp_ns = stamp
            self.gate.command_received = time.monotonic() - age
        except (ValueError, TypeError, OverflowError) as error:
            self._trip(str(error))

    def _set_command_mode(
        self, request: SetCommandMode.Request, response: SetCommandMode.Response
    ) -> SetCommandMode.Response:
        """disable 상태에서 모드를 바꾸고 이전 명령을 폐기한다."""
        if self.enabled or self.gate.fault or request.mode not in COMMAND_MODES:
            response.success, response.message = (
                False,
                "disable and clear fault; choose velocity, position or current",
            )
            return response
        try:
            if np.max(np.abs(self.velocity)) > 0.02:
                raise ValueError("arm must be stopped for mode change")
            if self.bus:
                self.bus.set_mode(request.mode)
            self.command_mode = request.mode
            self.command = self.position.copy() if request.mode == "position" else np.zeros(3)
            self.gate.command_received = float("-inf")
            self.last_stamp_ns = self.get_clock().now().nanoseconds
            response.success, response.message = True, f"mode={self.command_mode}; disabled"
        except Exception as error:
            self._trip(str(error))
            response.success, response.message = False, str(error)
        return response

    def _enable(self, request: SetBool.Request, response: SetBool.Response) -> SetBool.Response:
        try:
            if not request.data:
                self._stop()
                self.gate.fault = ""
                response.success, response.message = True, "disabled; fault reset"
                return response
            if self.enabled:
                raise ValueError("already enabled; disable before restarting")
            if self.gate.fault or not self.gate.mode_ok(time.monotonic()):
                raise ValueError("fresh ACTIVE mode and cleared fault required")
            if self.bus:
                self.position, self.velocity = self.bus.read_state()
            self.mapping.check_position(self.position)
            if np.max(np.abs(self.velocity)) > 0.02:
                raise ValueError("arm must be stopped before enabling")
            if self.bus:
                self.bus.enable()
            if not self.gate.mode_ok(time.monotonic()):
                raise ValueError("ACTIVE heartbeat expired during enable")
            self.command = self.position.copy() if self.command_mode == "position" else np.zeros(3)
            self.enabled = True
            self.last_tick = time.monotonic()
            self.last_stamp_ns = self.get_clock().now().nanoseconds
            self.gate.command_received = self.last_tick
            self.enable_stamp_ns = self.last_stamp_ns
            response.success, response.message = True, "enabled"
        except Exception as error:
            self._trip(str(error))
            response.success, response.message = False, str(error)
        return response

    def _stop(self) -> None:
        self.enabled = False
        self.command = np.zeros(3)
        self.gate.command_received = float("-inf")
        if self.bus:
            self.bus.disable()
        else:
            self.velocity = np.zeros(3)
            self.current = np.zeros(3)

    def _trip(self, reason: str) -> None:
        self.gate.fault = reason
        self.get_logger().error(f"arm stopped: {reason}")
        try:
            self._stop()
        except Exception as error:
            self.get_logger().error(f"arm stop transport failure: {error}")

    def _publish(self, publisher: object, position: np.ndarray, velocity: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position, msg.velocity = position.tolist(), velocity.tolist()
        publisher.publish(msg)

    def _tick(self) -> None:
        now = time.monotonic()
        tick_start = now
        was_enabled = self.enabled
        read_s, write_s = 0.0, 0.0
        dt, self.last_tick = now - self.last_tick, now
        interval = dt
        if self.gate.fault:
            # 정지 패킷도 실패한 경우 추가 read가 장치 watchdog을 연장하지 않게 한다.
            msg = String()
            msg.data = (
                f"dry_run={self.dry_run}; mode={self.command_mode}; enabled=False; "
                f"fault={self.gate.fault}"
            )
            self.status_pub.publish(msg)
            self._publish_timing(now)
            return
        try:
            if self.bus:
                read_start = time.monotonic()
                try:
                    self.position, self.velocity = self.bus.read_state()
                    self.current = self.bus.current.copy()
                finally:
                    read_s = time.monotonic() - read_start
                # 직렬 read가 지연된 동안 명령/모드가 만료될 수 있으므로 다시 시간을 읽는다.
                now = time.monotonic()
                dt += now - self.last_tick
            else:
                self.position += self.velocity * dt
            self.mapping.check_position(self.position)
            if self.enabled:
                if self.command_mode == "current":
                    if np.any(np.abs(self.velocity) > self.mapping.velocity_limits):
                        raise ValueError("measured velocity limit exceeded in current mode")
                    self.mapping.check_position(
                        self.position + self.velocity * self.gate.command_timeout_s
                    )
                if dt > self.gate.command_timeout_s:
                    raise ValueError("control loop deadline missed")
                if not self.gate.ready(now):
                    raise ValueError("command or ACTIVE heartbeat expired")
                if self.command_mode == "velocity":
                    raw = self.mapping.encode_velocity(self.command)
                    actual = self.mapping.decode_velocity(raw)
                    self.mapping.check_position(
                        self.position + actual * self.gate.command_timeout_s
                    )
                elif self.command_mode == "position":
                    raw = self.mapping.encode_position(self.command)
                    actual = self.mapping.decode_position(raw)
                else:
                    raw = encode_current(self.mapping, self.command, self.current_limits)
                    actual = decode_current(self.mapping, raw)
                if self.bus:
                    write_start = time.monotonic()
                    try:
                        getattr(self.bus, f"write_{self.command_mode}")(raw)
                    finally:
                        write_s = time.monotonic() - write_start
                    if time.monotonic() - tick_start > self.gate.command_timeout_s:
                        raise ValueError("control loop deadline missed during write")
                else:
                    if self.command_mode == "velocity":
                        self.velocity = actual
                    elif self.command_mode == "position":
                        self.velocity = np.clip(
                            (actual - self.position) / max(dt, self.period),
                            -self.profile_velocity,
                            self.profile_velocity,
                        )
                    else:
                        # 전류 dry_run은 단위·통신 검사용이며 전류에서 운동을 만들지 않는다.
                        self.current = actual
            self._publish(self.state_pub, self.position, self.velocity)
            current_msg = JointCurrent()
            current_msg.header.stamp = self.get_clock().now().to_msg()
            current_msg.name, current_msg.current = list(JOINT_NAMES), self.current.tolist()
            self.current_pub.publish(current_msg)
        except Exception as error:
            self._trip(str(error))
        msg = String()
        msg.data = (
            f"dry_run={self.dry_run}; mode={self.command_mode}; enabled={self.enabled}; "
            f"fault={self.gate.fault}"
        )
        self.status_pub.publish(msg)
        end = time.monotonic()
        if was_enabled:
            self._record_timing(interval, read_s, write_s, end - tick_start)
        self._publish_timing(end)

    def _record_timing(self, interval: float, read: float, write: float, cycle: float) -> None:
        """enabled 주기의 실행 시간과 기동 이후 최대값·초과 횟수를 기록한다."""
        self.timing["cycles"] += 1
        self.timing["cycle_overruns"] += int(cycle > self.period)
        # 작은 타이머 jitter와 한 주기 이상 밀리는 현상을 구분한다.
        self.timing["late_intervals"] += int(interval > self.period * 1.5)
        for name, seconds in (
            ("interval", interval),
            ("read", read),
            ("write", write),
            ("cycle", cycle),
        ):
            self.timing[f"last_{name}_ms"] = seconds * 1000
            self.timing[f"max_{name}_ms"] = max(self.timing[f"max_{name}_ms"], seconds * 1000)

    def _publish_timing(self, now: float) -> None:
        """실측 주기 통계를 1 Hz로 발행한다. fault 중에도 직전 통계를 유지한다."""
        if now - self.last_timing_publish < 1.0:
            return
        self.last_timing_publish = now
        msg = String()
        msg.data = json.dumps({"dry_run": self.dry_run, "enabled": self.enabled, **self.timing})
        self.timing_pub.publish(msg)

    def close(self) -> None:
        """노드 종료 전에 버스의 정지 절차를 실행한다."""
        if self.bus:
            self.bus.close()


def main(args: list[str] | None = None) -> None:
    """ROS2 실행 진입점."""
    rclpy.init(args=args)
    node = None
    try:
        node = ArmDriverNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.close()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
