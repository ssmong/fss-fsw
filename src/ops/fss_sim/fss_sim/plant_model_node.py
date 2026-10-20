"""SIL 플랜트 모델 ROS2 노드.

3-DOF 평면 동역학, 밸브 모델, 센서 노이즈를 통합하여
하드웨어 없이 GNC 소프트웨어를 검증할 수 있는 SIL 환경을 제공한다.

데이터 흐름:
  드라이버의 /actuators/valve_commands → 밸브 모델 → B·thrust = wrench
                                          → 동역학 (RK4)
                                            → 센서 모델 (Vicon + IMU)
                                              → /sensors/vicon_state
                                              → /sensors/imu

시간: 이 노드가 sim time의 소스다 — /clock을 발행하고 모든 메시지를
sim time으로 스탬프한다. 다른 노드들은 use_sim_time:=true로 실행해
타이머 지연 시에도 물리 시간과 항법 dt가 일치한다.
(이 노드 자체는 wall-clock 타이머로 구동 — 자기 시계 의존 순환 방지)

ZOH 동작: 새 밸브 명령이 없으면 마지막 수신 명령을 유지 (현실 PLC와 일치).
"""

import numpy as np
import rclpy
import yaml
from builtin_interfaces.msg import Time
from fss_common.math_utils import build_thruster_matrix
from fss_common.paths import config_file
from fss_common.valve_thrust_map import load_valve_thrust_map
from fss_interfaces.msg import SpacecraftState, SystemMode, ValveCommands
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu

from fss_sim.catalog_curve import PVQ30CatalogCurve
from fss_sim.dynamics import PlanarDynamics
from fss_sim.sensor_model import IMUModel, ViconModel
from fss_sim.valve_model import ValveModel


class PlantModelNode(Node):
    """SIL 플랜트 모델 노드."""

    def __init__(self) -> None:
        """노드 초기화."""
        super().__init__("plant_model_node")
        self.declare_parameter("use_thrusters", True, ParameterDescriptor(read_only=True))
        self._use_thrusters = self.get_parameter("use_thrusters").value

        # ── 파라미터 (타입만 선언 — 값은 반드시 YAML에서, 누락 시 기동 실패) ──
        # 의미/단위: config/system/sim.yaml과 system/sim_valve/*.yaml 참조
        S = Parameter.Type.STRING
        self.declare_parameter("valve_model_mode", S)
        self.declare_parameter("catalog_curve_file", S)
        self.declare_parameter("thrust_map_file", S)

        D = Parameter.Type.DOUBLE
        for name in (
            "mass",
            "inertia_zz",
            "initial_x",
            "initial_y",
            "initial_theta",
            "max_thrust_per_valve",
            "pressure_diff_mpa",
            "signal_current_min_ma",
            "signal_current_max_ma",
            "signal_to_coil_current_gain",
            "input_delay_ms",
            "tau_valve_ms",
            "vicon_rate_hz",
            "imu_rate_hz",
            "sim_rate_hz",
            "vicon_position_noise",
            "vicon_heading_noise",
            "gyro_noise_density",
            "accel_noise_density",
            "gyro_bias_random_walk",
            "accel_bias_random_walk",
        ):
            self.declare_parameter(name, D)

        # ── 파라미터 읽기 ──
        mass = self.get_parameter("mass").value
        izz = self.get_parameter("inertia_zz").value
        sim_rate = self.get_parameter("sim_rate_hz").value
        vicon_rate = self.get_parameter("vicon_rate_hz").value
        imu_rate = self.get_parameter("imu_rate_hz").value

        # ── 동역학 모델 ──
        self._dynamics = PlanarDynamics(mass=mass, inertia_zz=izz)
        self._dynamics.set_state(
            px=self.get_parameter("initial_x").value,
            py=self.get_parameter("initial_y").value,
            theta=self.get_parameter("initial_theta").value,
        )
        self._mass = mass

        # ── 밸브 모델 ──
        valve_model_mode = self.get_parameter("valve_model_mode").value
        catalog_curve = None
        if valve_model_mode == "real":
            curve_file = self.get_parameter("catalog_curve_file").value
            catalog_curve = PVQ30CatalogCurve.from_yaml(
                config_file(*curve_file.replace("\\", "/").split("/"))
            )
        self._valve_model = ValveModel(
            max_thrust_per_valve=self.get_parameter("max_thrust_per_valve").value,
            model_mode=valve_model_mode,
            thrust_map=load_valve_thrust_map(self.get_parameter("thrust_map_file").value),
            catalog_curve=catalog_curve,
            pressure_diff_mpa=self.get_parameter("pressure_diff_mpa").value,
            signal_current_min_ma=self.get_parameter("signal_current_min_ma").value,
            signal_current_max_ma=self.get_parameter("signal_current_max_ma").value,
            signal_to_coil_current_gain=self.get_parameter("signal_to_coil_current_gain").value,
            input_delay_ms=self.get_parameter("input_delay_ms").value,
            tau_valve_ms=self.get_parameter("tau_valve_ms").value,
        )

        # ── B-matrix 로드 (추력기 기하학) ──
        self._B = self._load_B_matrix()

        # ── 센서 모델 ──
        self._vicon = ViconModel(
            position_noise=self.get_parameter("vicon_position_noise").value,
            heading_noise=self.get_parameter("vicon_heading_noise").value,
        )
        self._imu = IMUModel(
            gyro_noise_density=self.get_parameter("gyro_noise_density").value,
            accel_noise_density=self.get_parameter("accel_noise_density").value,
            gyro_bias_random_walk=self.get_parameter("gyro_bias_random_walk").value,
            accel_bias_random_walk=self.get_parameter("accel_bias_random_walk").value,
            sample_rate_hz=imu_rate,
        )

        # ── ZOH: 마지막 수신 밸브 명령 유지 ──
        self._last_duty = [0.0] * 8
        self._latest_mode: int | None = None
        self._last_wrench = np.zeros(3)

        # ── 구독/발행 ──
        self.create_subscription(
            ValveCommands,
            "/actuators/valve_commands",
            self._on_valve_cmd,
            10,
        )
        self.create_subscription(SystemMode, "/system/mode", self._on_mode, 10)
        self._vicon_pub = self.create_publisher(SpacecraftState, "/sensors/vicon_state", 10)
        self._imu_pub = self.create_publisher(Imu, "/sensors/imu", 10)
        self._true_state_pub = self.create_publisher(
            SpacecraftState,
            "/sim/true_state",
            10,
        )
        self._clock_pub = self.create_publisher(Clock, "/clock", 10)

        # ── sim time + 타이머 ──
        self._sim_dt = 1.0 / sim_rate
        self._sim_time_ns = 0  # 스텝마다 sim_dt씩 순증가
        self._sim_timer = self.create_timer(self._sim_dt, self._sim_step)

        # 센서 발행 카운터 (sim 루프 대비 비율)
        self._vicon_every = max(1, int(sim_rate / vicon_rate))
        self._imu_every = max(1, int(sim_rate / imu_rate))
        self._step_count = 0

        self.get_logger().info(
            f"SIL 플랜트 시작 (mass={mass}kg, Izz={izz}, "
            f"valve={valve_model_mode}, 추력기 사용: {self._use_thrusters}, "
            f"sim={sim_rate}Hz, vicon={vicon_rate}Hz, imu={imu_rate}Hz, /clock 발행)"
        )

    def _load_B_matrix(self) -> np.ndarray:
        """thruster_geometry.yaml → 기하 B행렬(추력[N]→wrench). 실패 시 기동 중단.

        Raises:
            RuntimeError: 파일/키 누락 시 (zero-B 무추력 SIL 방지)
        """
        try:
            p = config_file("vehicle", "thruster_geometry.yaml")
            with open(p, encoding="utf-8") as f:
                data = yaml.safe_load(f)
            # ValueError: 방향 비단위벡터 (build_thruster_matrix)
            return build_thruster_matrix(data["thruster_geometry"]["thrusters"])
        except (OSError, KeyError, TypeError, ValueError, RuntimeError) as e:
            msg = f"[plant_model_node] 추력기 B행렬 로드 실패: {e!r}"
            self.get_logger().fatal(msg)
            raise RuntimeError(msg) from e

    def _sim_stamp(self) -> Time:
        """현재 sim time을 메시지 스탬프로 변환한다."""
        t = Time()
        t.sec = self._sim_time_ns // 1_000_000_000
        t.nanosec = self._sim_time_ns % 1_000_000_000
        return t

    def _on_valve_cmd(self, msg: ValveCommands) -> None:
        """밸브 명령 수신 (ZOH: 마지막 명령 유지)."""
        if not self._use_thrusters or self._latest_mode != SystemMode.ACTIVE:
            self._last_duty = [0.0] * 8
            return
        self._last_duty = [
            d if enabled else 0.0 for d, enabled in zip(msg.duty_cycles, msg.enabled)
        ]

    def _on_mode(self, msg: SystemMode) -> None:
        """ACTIVE 이외의 모드에서는 새 명령을 기다리지 않고 추력을 차단한다."""
        self._latest_mode = msg.mode
        if msg.mode != SystemMode.ACTIVE:
            self._last_duty = [0.0] * 8
            self._valve_model.reset()

    def _sim_step(self) -> None:
        """시뮬레이션 한 스텝 (1000Hz)."""
        dt = self._sim_dt

        # 밸브 모델 → 실제 추력
        if self._use_thrusters:
            thrust = self._valve_model.step(self._last_duty, dt)
            wrench = self._B @ thrust
        else:
            wrench = np.zeros(3)
        self._last_wrench = wrench

        # 동역학 전파
        self._dynamics.step(wrench, dt)

        self._step_count += 1
        self._sim_time_ns += int(dt * 1e9)

        # /clock 발행 — 다른 노드(use_sim_time)의 시간 소스
        clock_msg = Clock()
        clock_msg.clock = self._sim_stamp()
        self._clock_pub.publish(clock_msg)

        # 실제 상태 발행 (디버그용, Vicon과 동일 주기)
        if self._step_count % self._vicon_every == 0:
            st = self._dynamics.state
            # 플랜트는 본체 프레임 속도를 적분(진실)하지만, EKF/컨트롤러는
            # 테이블(관성) 프레임 속도를 쓰므로 발행 시 R(θ)로 회전한다.
            cos_th = np.cos(st[2])
            sin_th = np.sin(st[2])
            vx_table = st[3] * cos_th - st[4] * sin_th
            vy_table = st[3] * sin_th + st[4] * cos_th
            true_msg = SpacecraftState()
            true_msg.header.stamp = self._sim_stamp()
            true_msg.header.frame_id = "table"
            true_msg.x = float(st[0])
            true_msg.y = float(st[1])
            true_msg.theta = float(st[2])
            true_msg.vx = float(vx_table)
            true_msg.vy = float(vy_table)
            true_msg.omega = float(st[5])
            self._true_state_pub.publish(true_msg)

        # Vicon 발행 (노이즈 포함)
        if self._step_count % self._vicon_every == 0:
            x, y, th = self._vicon.measure(self._dynamics.state)
            msg = SpacecraftState()
            msg.header.stamp = self._sim_stamp()
            msg.header.frame_id = "table"
            msg.x = x
            msg.y = y
            msg.theta = th
            # 속도는 EKF가 추정 — 여기서는 0
            self._vicon_pub.publish(msg)

        # IMU 발행
        if self._step_count % self._imu_every == 0:
            # dt는 IMU 발행 간격 (sim 내부 스텝이 아님 — 바이어스 RW 분산이 걸린 시간)
            imu_dt = self._imu_every * self._sim_dt
            omega_z, ax, ay = self._imu.measure(
                self._dynamics.state,
                self._last_wrench,
                self._mass,
                imu_dt,
            )
            msg = Imu()
            msg.header.stamp = self._sim_stamp()
            msg.header.frame_id = "body"
            msg.angular_velocity.z = omega_z
            msg.linear_acceleration.x = ax
            msg.linear_acceleration.y = ay
            self._imu_pub.publish(msg)


def main(args: list[str] | None = None) -> None:
    """메인 엔트리포인트."""
    rclpy.init(args=args)
    node = PlantModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
