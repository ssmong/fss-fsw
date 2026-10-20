#!/usr/bin/env python3
"""선택적 Python 연구용 GNC 노드 (20Hz 제어 루프).

기본 실행 파일은 C++ gnc_node다. 이 백엔드는
FSS_GNC_BUILD_PYTHON_BINDINGS=ON으로 빌드한 뒤 gnc_node.py로 명시 실행한다.

유도 → 제어 → 할당 파이프라인을 실행한다.
병진/회전 제어기를 독립적으로 교체할 수 있다.

지원 제어기:
  - pid: PID (C++ core, 기본)
  - lqr: LQR (C++ core, 사전계산 게인)

데이터 흐름:
  /navigation/state → Guidance → Controller → Allocator(C++) → /actuators/thrust_commands

세이프가드:
  - state age > 100ms → 제어 출력 0 (발산 방지)
  - NaN/Inf 검사
"""

import math

import rclpy
import yaml
from rclpy.node import Node
from rclpy.parameter import Parameter

from fss_common.math_utils import build_thruster_matrix
from fss_common.paths import config_file
from fss_common.valve_thrust_map import load_valve_thrust_map
from fss_interfaces.msg import SpacecraftState, ThrustCommands, TrajectoryReference
from fss_interfaces.srv import SetController, SetGuidanceMode
from fss_gnc._gnc_core import (
    LQRRotational,
    LQRTranslational,
    PIDRotational,
    PIDTranslational,
    QPAllocator,
    RotationalRef,
    RotationalState,
    TranslationalRef,
    TranslationalState,
)
from fss_gnc.guidance import GuidanceManager


class GNCNode(Node):
    """GNC 메인 노드 — 유도/제어/할당 (20Hz)."""

    def __init__(self) -> None:
        """노드 초기화."""
        super().__init__("gnc_node")

        # ── 파라미터 (전부 타입만 선언 — 값은 반드시 YAML에서, 누락 시 기동 실패) ──
        D, S, DA = (
            Parameter.Type.DOUBLE, Parameter.Type.STRING,
            Parameter.Type.DOUBLE_ARRAY,
        )
        self.declare_parameter("control_rate_hz", D)
        self.declare_parameter("state_age_limit_ms", D)

        # 병진/회전 제어기
        self.declare_parameter("translational.type", S)
        self.declare_parameter("translational.kp", D)
        self.declare_parameter("translational.ki", D)
        self.declare_parameter("translational.kd", D)
        self.declare_parameter("translational.max_force", D)
        self.declare_parameter("rotational.type", S)
        self.declare_parameter("rotational.kp", D)
        self.declare_parameter("rotational.ki", D)
        self.declare_parameter("rotational.kd", D)
        self.declare_parameter("rotational.max_torque", D)

        # 할당기
        self.declare_parameter("allocator.thrust_weight", D)
        self.declare_parameter("allocator.min_thrust", D)
        self.declare_parameter("allocator.max_thrust_per_nozzle", D)  # 맵 미사용 시 최대 추력 [N]
        self.declare_parameter("allocator.thrust_map_file", S)

        # 유도
        self.declare_parameter("guidance.mode", S)
        self.declare_parameter("guidance.target_x", D)
        self.declare_parameter("guidance.target_y", D)
        self.declare_parameter("guidance.target_theta", D)
        # trajectory 모드에서만 필수인 경유점 배열은 모드 확인 후 조건부 선언
        if self.get_parameter("guidance.mode").value == "trajectory":
            for name in ("traj_t", "traj_x", "traj_y", "traj_theta"):
                self.declare_parameter(f"guidance.{name}", DA)

        control_rate = self.get_parameter("control_rate_hz").value
        self._state_age_limit_ns = int(self.get_parameter("state_age_limit_ms").value * 1e6)

        # 본체 프레임 추력 한계 (할당 직전 본체 프레임에서 클램핑용)
        # 추력기는 본체 고정이므로 가용 추력 한계는 본체 프레임 기준이다.
        self._max_force = self.get_parameter("translational.max_force").value

        # ── 유도 ──
        guidance_mode = self.get_parameter("guidance.mode").value
        # 화이트리스트 검증 — 오타가 조용히 target(정지)로 폴백되지 않도록 기동 중단
        if guidance_mode not in ("target", "trajectory"):
            raise RuntimeError(
                f"[gnc_node] 알 수 없는 guidance.mode: '{guidance_mode}' "
                f"(target/trajectory 중 하나여야 함)"
            )
        traj_points = self._load_trajectory_params()  # 경유점 <2개면 None
        if guidance_mode == "trajectory" and traj_points is None:
            raise RuntimeError(
                "[gnc_node] guidance.mode=trajectory인데 유효한 경유점(traj_*, 길이≥2)이 없음"
            )
        self._guidance = GuidanceManager(
            mode=guidance_mode,
            target={
                "x": self.get_parameter("guidance.target_x").value,
                "y": self.get_parameter("guidance.target_y").value,
                "theta": self.get_parameter("guidance.target_theta").value,
            },
            trajectory_points=traj_points,
        )
        # set_trajectory가 mode를 덮어쓰므로 파라미터 값으로 복원
        self._guidance.mode = guidance_mode

        # ── 제어기 타입 결정 ──
        self._trans_type = self.get_parameter("translational.type").value
        self._rot_type = self.get_parameter("rotational.type").value

        # ── 제어기 생성 (병진/회전 독립) ──
        self._trans_ctrl = self._create_trans_controller()
        self._rot_ctrl = self._create_rot_controller()

        # ── 할당기 (C++ core) ──
        self._allocator = QPAllocator()
        self._allocator.thrust_weight = self.get_parameter("allocator.thrust_weight").value
        self._allocator.min_thrust = self.get_parameter("allocator.min_thrust").value
        self._setup_allocator_B()

        # ── 상태 저장 ──
        self._latest_state: SpacecraftState | None = None
        self._latest_state_ns: int = 0
        self._t0: float | None = None

        # ── ROS2 인터페이스 ──
        self.create_subscription(
            SpacecraftState, "/navigation/state", self._on_state, 10,
        )
        self._cmd_pub = self.create_publisher(ThrustCommands, "/actuators/thrust_commands", 10)
        self._ref_pub = self.create_publisher(TrajectoryReference, "/gnc/trajectory_ref", 10)

        # 서비스
        self.create_service(SetGuidanceMode, "/gnc/set_guidance_mode", self._srv_guidance)
        self.create_service(SetController, "/gnc/set_controller", self._srv_controller)

        # 제어 루프 타이머
        self._dt = 1.0 / control_rate
        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f"GNC 노드 시작 ({control_rate}Hz, "
            f"trans={self._trans_type}, rot={self._rot_type})"
        )

    def _create_trans_controller(self):
        """병진 제어기를 파라미터에 따라 생성한다 (pid/lqr)."""
        ctrl_type = self._trans_type

        if ctrl_type == "lqr":
            ctrl = LQRTranslational()
            ctrl.set_gain(self._load_lqr_gains("K_trans"), self._max_force)
            self.get_logger().info("LQR 병진 게인 로드 완료")
            return ctrl

        if ctrl_type == "pid":
            ctrl = PIDTranslational()
            ctrl.set_gains(
                self.get_parameter("translational.kp").value,
                self.get_parameter("translational.ki").value,
                self.get_parameter("translational.kd").value,
                self.get_parameter("translational.max_force").value,
            )
            return ctrl

        # 알 수 없는 타입은 조용히 PID로 폴백하지 않고 기동 중단
        raise RuntimeError(
            f"[gnc_node] 알 수 없는 translational.type: '{ctrl_type}' (pid/lqr만 지원)"
        )

    def _create_rot_controller(self):
        """회전 제어기를 파라미터에 따라 생성한다 (pid/lqr)."""
        ctrl_type = self._rot_type

        if ctrl_type == "lqr":
            ctrl = LQRRotational()
            ctrl.set_gain(
                self._load_lqr_gains("K_rot"),
                self.get_parameter("rotational.max_torque").value,
            )
            self.get_logger().info("LQR 회전 게인 로드 완료")
            return ctrl

        if ctrl_type == "pid":
            ctrl = PIDRotational()
            ctrl.set_gains(
                self.get_parameter("rotational.kp").value,
                self.get_parameter("rotational.ki").value,
                self.get_parameter("rotational.kd").value,
                self.get_parameter("rotational.max_torque").value,
            )
            return ctrl

        raise RuntimeError(
            f"[gnc_node] 알 수 없는 rotational.type: '{ctrl_type}' (pid/lqr만 지원)"
        )

    def _load_trajectory_params(self) -> list[dict] | None:
        """guidance.traj_* 파라미터에서 경유점을 만든다. 미지정(<2점)이면 None.

        Raises:
            RuntimeError: 배열 길이 불일치 또는 t 비오름차순
        """
        if not self.has_parameter("guidance.traj_t"):
            return None  # trajectory 모드가 아니면 미선언
        tt = list(self.get_parameter("guidance.traj_t").value)
        tx = list(self.get_parameter("guidance.traj_x").value)
        ty = list(self.get_parameter("guidance.traj_y").value)
        tth = list(self.get_parameter("guidance.traj_theta").value)
        if len(tt) < 2:
            return None
        if not (len(tt) == len(tx) == len(ty) == len(tth)):
            raise RuntimeError(
                f"[gnc_node] traj_* 배열 길이 불일치: "
                f"t={len(tt)}, x={len(tx)}, y={len(ty)}, theta={len(tth)}"
            )
        if any(t1 <= t0 for t0, t1 in zip(tt, tt[1:])):
            raise RuntimeError(f"[gnc_node] traj_t가 순증가가 아님: {tt}")
        return [
            {"t": t, "x": x, "y": y, "theta": th}
            for t, x, y, th in zip(tt, tx, ty, tth)
        ]

    def _load_lqr_gains(self, key_name: str) -> list[float]:
        """lqr_gains.yaml에서 K 게인을 로드한다. 실패 시 기동 중단 (zero-gain 무제어 방지).

        Raises:
            RuntimeError: 파일/키 누락 시
        """
        try:
            p = config_file("gnc", "controller", "lqr_gains.yaml")
            with open(p, encoding="utf-8") as f:
                data = yaml.safe_load(f)
            return list(data["lqr"][key_name])
        except (OSError, KeyError, TypeError, RuntimeError) as e:
            msg = f"[gnc_node] LQR 게인({key_name}) 로드 실패: {e!r} — scripts/compute_lqr_gains.py 실행 필요"
            self.get_logger().fatal(msg)
            raise RuntimeError(msg) from e

    def _setup_allocator_B(self) -> None:
        """추력기 기하와 최대 추력으로 정규화 추력→wrench 행렬을 설정한다.

        Raises:
            RuntimeError: 파일/키 누락 시 (zero-B 무추력 방지)
        """
        try:
            p = config_file("vehicle", "thruster_geometry.yaml")
            with open(p, encoding="utf-8") as f:
                data = yaml.safe_load(f)
            thrusters = data["thruster_geometry"]["thrusters"]
            f_max = self.get_parameter("allocator.max_thrust_per_nozzle").value
            # GNC는 할당 한계만 사용한다. 목표 추력→밸브 입력 변환은 드라이버가 맡는다.
            thrust_map = load_valve_thrust_map(
                self.get_parameter("allocator.thrust_map_file").value
            )
            if thrust_map is not None:
                self._max_thrust_n = thrust_map.max_thrust_n.tolist()
            else:
                if not math.isfinite(f_max) or f_max <= 0.0:
                    raise ValueError("allocator.max_thrust_per_nozzle은 유한한 양수여야 한다")
                self._max_thrust_n = [f_max] * len(thrusters)
            # 기하 B × 노즐별 최대 추력 = 정규화 추력→wrench[N, N·m] 행렬
            # ValueError: 방향 비단위벡터(build_thruster_matrix) 또는 길이≠24(set_B_matrix)
            B = build_thruster_matrix(thrusters) * self._max_thrust_n
            self._allocator.set_B_matrix(list(B.flatten()))
        except (OSError, KeyError, TypeError, ValueError, RuntimeError) as e:
            msg = f"[gnc_node] 추력기 B행렬 로드 실패: {e!r}"
            self.get_logger().fatal(msg)
            raise RuntimeError(msg) from e

    def _on_state(self, msg: SpacecraftState) -> None:
        """항법 상태 수신."""
        self._latest_state = msg
        self._latest_state_ns = self.get_clock().now().nanoseconds

    def _control_loop(self) -> None:
        """20Hz 제어 루프: 유도 → 제어 → 할당."""
        now_ns = self.get_clock().now().nanoseconds

        # ── Step 0: state age 체크 ──
        if self._latest_state is None:
            return

        age_ns = now_ns - self._latest_state_ns
        if age_ns > self._state_age_limit_ns:
            self._publish_zero_command()
            self.get_logger().warn(
                f"State age {age_ns * 1e-6:.0f}ms > 한계 — 제어 출력 0"
            )
            return

        s = self._latest_state

        # 시간 계산
        t_sec = s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t_sec
        t = t_sec - self._t0

        # ── Step 1: 유도 ──
        state_dict = {"x": s.x, "y": s.y, "theta": s.theta,
                       "vx": s.vx, "vy": s.vy, "omega": s.omega}
        ref = self._guidance.compute(state_dict, t)

        # 기준 발행
        ref_msg = TrajectoryReference()
        ref_msg.header.stamp = self.get_clock().now().to_msg()
        ref_msg.x_ref = ref["x_ref"]
        ref_msg.y_ref = ref["y_ref"]
        ref_msg.theta_ref = ref["theta_ref"]
        ref_msg.vx_ref = ref["vx_ref"]
        ref_msg.vy_ref = ref["vy_ref"]
        ref_msg.omega_ref = ref["omega_ref"]
        self._ref_pub.publish(ref_msg)

        # ── Step 2 & 3: 제어 (PID/LQR 병진·회전 독립) ──
        ts = TranslationalState(s.x, s.y, s.vx, s.vy, s.theta)
        tr = TranslationalRef(ref["x_ref"], ref["y_ref"], ref["vx_ref"], ref["vy_ref"])
        tout = self._trans_ctrl.compute(ts, tr, self._dt)
        fx, fy = tout.fx, tout.fy

        rs = RotationalState(s.theta, s.omega)
        rr = RotationalRef(ref["theta_ref"], ref["omega_ref"])
        mz = self._rot_ctrl.compute(rs, rr, self._dt)

        # ── Step 3.5: 테이블 → 본체 프레임 힘 회전 + 본체 프레임 추력 클램프 ──
        # 컨트롤러(PID/LQR)는 관성 프레임 정식화로 테이블 프레임 힘을 출력한다.
        # 추력기는 본체 고정이고 할당 B-행렬도 본체 프레임이므로,
        # 할당 직전에 [Fx, Fy]_table → [Fx, Fy]_body 회전을 1회 수행한다.
        # (Mz 토크는 회전 불변 스칼라이므로 그대로 전달)
        cos_th = math.cos(s.theta)
        sin_th = math.sin(s.theta)
        fx_body = cos_th * fx + sin_th * fy
        fy_body = -sin_th * fx + cos_th * fy

        # 추력 한계는 본체 고정 추력기 기준이므로 회전 후 본체 프레임에서 클램핑한다.
        # (컨트롤러가 테이블 프레임 축별로 클램핑하면 θ≠0에서 박스가 회전되어
        #  실제 가용 추력과 불일치 — 권위 있는 포화 한계는 여기서 본체 프레임으로 적용.
        #  진짜 추력기 다면체 한계는 QP 할당기가 duty∈[0,1]로 최종 강제.)
        fx_body = max(-self._max_force, min(self._max_force, fx_body))
        fy_body = max(-self._max_force, min(self._max_force, fy_body))

        # ── Step 4: 할당 (C++) ──
        alloc = self._allocator.allocate(fx_body, fy_body, mz)
        normalized_thrust = alloc.duty_list()

        # NaN 검사
        if any(not math.isfinite(d) for d in normalized_thrust):
            self.get_logger().error("할당기 NaN 출력 — 제어 출력 0")
            self._publish_zero_command()
            return

        # ── Step 5: 노즐별 목표 추력 [N] 발행 ──
        cmd = ThrustCommands()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.thrust_n = [
            value * maximum
            for value, maximum in zip(normalized_thrust, self._max_thrust_n, strict=True)
        ]
        cmd.enabled = [True] * 8
        self._cmd_pub.publish(cmd)

    def _publish_zero_command(self) -> None:
        """모든 노즐에 목표 추력 0N을 발행한다."""
        cmd = ThrustCommands()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.thrust_n = [0.0] * 8
        cmd.enabled = [True] * 8
        self._cmd_pub.publish(cmd)

    def _srv_guidance(self, req, resp):
        """유도 모드 변경 서비스."""
        # GuidanceManager가 실제 지원하는 모드만 허용 (미구현 모드는 실패 반환)
        # STATION_KEEP=0 → 고정 목표 유지("target"), TRAJECTORY=2 → 시간 기반 궤적
        mode_map = {
            SetGuidanceMode.Request.STATION_KEEP: "target",
            SetGuidanceMode.Request.TRAJECTORY: "trajectory",
        }
        if req.mode not in mode_map:
            resp.success = False
            resp.message = f"미지원 유도 모드: {req.mode} (WAYPOINT/PROXIMITY/MANUAL 미구현)"
            self.get_logger().warn(resp.message)
            return resp

        # 궤적이 로드되지 않은 상태의 trajectory 전환은 거부 (조용한 station-keep 폴백 방지)
        if mode_map[req.mode] == "trajectory" and not self._guidance.has_trajectory:
            resp.success = False
            resp.message = "trajectory 모드 전환 불가: 로드된 궤적 없음 (guidance.traj_* 설정 필요)"
            self.get_logger().warn(resp.message)
            return resp

        self._guidance.mode = mode_map[req.mode]
        resp.success = True
        resp.message = f"유도 모드 변경: {self._guidance.mode}"
        self.get_logger().info(resp.message)
        return resp

    def _srv_controller(self, req, resp):
        """제어기 변경/리셋 서비스."""
        new_type = req.controller_type.strip().lower() if req.controller_type else ""

        if new_type in ("pid", "lqr"):
            # 전환 실패(게인 파일/파라미터 누락) 시 기존 제어기를 유지하고 실패 응답
            prev = (self._trans_type, self._rot_type, self._trans_ctrl, self._rot_ctrl)
            try:
                self._trans_type = new_type
                self._rot_type = new_type
                self._trans_ctrl = self._create_trans_controller()
                self._rot_ctrl = self._create_rot_controller()
            except Exception as e:
                self._trans_type, self._rot_type, self._trans_ctrl, self._rot_ctrl = prev
                resp.success = False
                resp.message = f"제어기 전환 실패({new_type}): {e}"
                self.get_logger().error(resp.message)
                return resp
            resp.success = True
            resp.message = f"제어기 변경: {new_type}"
        elif new_type == "":
            # 빈 문자열 = 명시적 리셋 요청 (현재 제어기 유지)
            self._trans_ctrl.reset()
            self._rot_ctrl.reset()
            resp.success = True
            resp.message = f"제어기 리셋: {self._trans_type}"
        else:
            # 오타/미지원 타입은 조용히 리셋으로 처리하지 않고 명시적 실패 반환
            resp.success = False
            resp.message = (
                f"알 수 없는 제어기 타입: '{req.controller_type}' "
                f"(pid/lqr 또는 빈 문자열(리셋)만 허용)"
            )
            self.get_logger().warn(resp.message)
            return resp

        self.get_logger().info(resp.message)
        return resp


def main(args: list[str] | None = None) -> None:
    """메인 엔트리포인트."""
    rclpy.init(args=args)
    node = GNCNode()
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
