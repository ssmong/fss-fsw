#!/usr/bin/env python3
"""FSS 실시간 미션 관제 콘솔 (PySide6 + pyqtgraph).

운용 판단에 필요한 정보 + 실험 제어를 한 화면에서 다룬다.

- 제어 바: E-STOP / 모드 스테퍼(PRESSURIZE→READY→ACTIVE) / ABORT / RESET / 에어베어링 토글.
  각 버튼은 FSM 허용 전이 + 준비도 충족 시에만 활성화된다 (준비 미충족 시 툴팁으로 사유 표시).
- 서브시스템 준비도 등: Vicon / IMU / EKF·Nav / PLC·Valve / 에어베어링.
- Vicon·EKF·기준 XY 궤적, 현재/목표/오차, 명령 렌치.

상세 시계열, 추정 불확실도, 밸브 상태, 로그는 하단 탭에서 확인한다. 자동 스케일 플롯은 최소
Y 스팬을 강제해 작은 신호에서 축이 확대돼 요동치는 것을 막는다.
새 시계열은 ``Dashboard._build_detail_tabs()``에 ``SeriesSpec``을 추가하면 된다.

사용법 (ROS2 DDS 네트워크에 연결된 노트북):
    source /opt/ros/humble/setup.bash
    source ~/fss-fsw/install/setup.bash
    python3 scripts/live_plot.py

옵션:
    --history-sec 60   표시할 이력 길이 [s]
    --fps 20           화면 갱신률
    --opengl           pyqtgraph OpenGL 렌더링 사용
    --fullscreen       전체 화면으로 시작
"""

import argparse
import html
import math
import os
import sys
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
import pyqtgraph as pg
import yaml
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QTransform
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSplitter,
    QTabWidget,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rcl_interfaces.msg import Log
from rclpy.node import Node
from sensor_msgs.msg import Imu

from fss_interfaces.msg import (
    SpacecraftState,
    SystemHealth,
    SystemMode,
    TrajectoryReference,
    ValveStatus,
)
from fss_interfaces.srv import MainValveControl, SetSystemMode


# ── 색상 팔레트 ───────────────────────────────────────────────────────────────
C_BG = "#0b0f14"
C_PANEL = "#141a22"
C_PANEL_ALT = "#10161d"
C_BORDER = "#27313d"
C_TEXT = "#e6edf3"
C_TEXT_DIM = "#a7b1bd"
C_VICON = "#58a6ff"
C_EKF = "#00d4aa"
C_REFERENCE = "#f2cc60"
C_WARN = "#f0883e"
C_ERROR = "#f85149"
C_OK = "#3fb950"
C_INFO = "#00b8d9"
C_IDLE = "#6e7681"
C_GRID = "#25303b"
C_WRENCH_POS = "#f0883e"
C_WRENCH_NEG = "#58a6ff"

BODY_HALF_W = 0.15  # 우주선 반폭 [m]
SOURCE_TIMEOUT_SEC = 2.5

MODE_COLORS = {
    "IDLE": C_IDLE,
    "PRESSURIZING": C_WARN,
    "READY": C_INFO,
    "ACTIVE": C_OK,
    "SAFE_HOLD": C_WARN,
    "EMERGENCY_STOP": C_ERROR,
}

# SystemMode 상수 (msg와 동일)
MODE_IDLE, MODE_PRESSURIZING, MODE_READY = 0, 1, 2
MODE_ACTIVE, MODE_SAFE_HOLD, MODE_ESTOP = 3, 4, 5

# FSM 허용 전이 (현재 모드명 → 허용 대상 모드값). mode_fsm._TRANSITIONS와 일치.
ALLOWED_TRANSITIONS = {
    "IDLE": {MODE_PRESSURIZING, MODE_ESTOP},
    "PRESSURIZING": {MODE_READY, MODE_IDLE, MODE_ESTOP},
    "READY": {MODE_ACTIVE, MODE_IDLE, MODE_ESTOP},
    "ACTIVE": {MODE_SAFE_HOLD, MODE_READY, MODE_ESTOP},
    "SAFE_HOLD": {MODE_READY, MODE_IDLE, MODE_ESTOP},
    "EMERGENCY_STOP": {MODE_IDLE},
}


@dataclass(frozen=True)
class SeriesSpec:
    """공통 시계열 플롯의 한 데이터 계열 정의."""

    time_attr: str
    value_attr: str
    label: str
    color: str
    width: float = 2.0
    line_style: str = "solid"


LINE_STYLES = {
    "solid": Qt.SolidLine,
    "dashed": Qt.DashLine,
    "dotted": Qt.DotLine,
}


def _style_plot(
    plot: pg.PlotWidget,
    title: str,
    *,
    x_label: str = "",
    y_label: str = "",
    grid_x: bool = True,
    grid_y: bool = True,
) -> None:
    """어두운 배경에서도 제목, 축, 눈금이 읽히도록 공통 스타일을 적용한다."""
    plot.setTitle(title, color=C_TEXT, size="12pt")
    label_style = {"color": C_TEXT_DIM, "font-size": "11pt"}
    if x_label:
        plot.setLabel("bottom", x_label, **label_style)
    if y_label:
        plot.setLabel("left", y_label, **label_style)

    tick_font = QFont("sans-serif", 10)
    for axis_name in ("bottom", "left"):
        axis = plot.getAxis(axis_name)
        axis.setTickFont(tick_font)
        axis.setPen(C_BORDER)
        # setPen() also changes the label color, so text color must be set last.
        axis.setTextPen(C_TEXT_DIM)
        axis.enableAutoSIPrefix(False)
    plot.showGrid(x=grid_x, y=grid_y, alpha=0.2)


def _add_readable_legend(
    plot: pg.PlotWidget,
    offset: tuple[int, int],
    columns: int = 1,
) -> None:
    """데이터 위에서도 읽기 쉬운 반투명 범례를 추가한다."""
    plot.addLegend(
        offset=offset,
        colCount=max(1, columns),
        horSpacing=12,
        labelTextColor=C_TEXT,
        labelTextSize="10pt",
        brush=pg.mkBrush(11, 15, 20, 220),
        pen=pg.mkPen(C_BORDER),
    )


def _wrap_angle_rad(angle: float) -> float:
    """각도를 [-π, π]로 정규화한다."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _config_path(*parts: str) -> str:
    """저장소의 config 파일 경로를 반환한다."""
    return os.path.normpath(
        os.path.join(os.path.dirname(__file__), "..", "config", *parts)
    )


def _load_thruster_matrix() -> np.ndarray:
    """추력기 형상 설정에서 3×8 B 행렬을 생성한다."""
    path = _config_path("vehicle", "thruster_geometry.yaml")
    try:
        with open(path, encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        thrusters = data["thruster_geometry"]["thrusters"]
        matrix = np.zeros((3, len(thrusters)))
        for index, thruster in enumerate(thrusters):
            direction = np.asarray(thruster["direction"], dtype=float)
            position = np.asarray(thruster["position"], dtype=float)
            matrix[0, index] = direction[0]
            matrix[1, index] = direction[1]
            matrix[2, index] = (
                position[0] * direction[1] - position[1] * direction[0]
            )
        return matrix
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        print(
            f"[live_plot] 추력기 B 행렬 로드 실패({exc!r}) — 렌치 표시는 0으로 유지",
            file=sys.stderr,
        )
        return np.zeros((3, 8))


def _load_plc_max_value() -> float:
    """PLC 출력 최댓값을 설정에서 읽는다."""
    path = _config_path("hardware", "plc.yaml")
    try:
        with open(path, encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        value_range = data["plc"]["value_range"]
        maximum = float(value_range[1])
        if maximum <= 0.0:
            raise ValueError("PLC 출력 최댓값은 양수여야 함")
        return maximum
    except (FileNotFoundError, KeyError, IndexError, TypeError, ValueError) as exc:
        print(
            f"[live_plot] PLC 범위 로드 실패({exc!r}) — 기본값 4000 사용",
            file=sys.stderr,
        )
        return 4000.0


def _load_max_thrust() -> float:
    """캘리브레이션 전 추정에 사용할 노즐당 최대 추력을 읽는다."""
    path = _config_path("vehicle", "mass_properties.yaml")
    try:
        with open(path, encoding="utf-8") as stream:
            data = yaml.safe_load(stream)
        maximum = float(data["max_thrust_per_nozzle"])
        if maximum <= 0.0:
            raise ValueError("노즐 최대 추력은 양수여야 함")
        return maximum
    except (FileNotFoundError, KeyError, TypeError, ValueError) as exc:
        print(
            f"[live_plot] 최대 추력 로드 실패({exc!r}) — 기본값 1.0 N 사용",
            file=sys.stderr,
        )
        return 1.0


class DashboardNode(Node):
    """대시보드에 필요한 ROS2 토픽을 수집한다."""

    def __init__(self, history_sec: float = 30.0) -> None:
        super().__init__("dashboard_node")
        history_size = max(100, int(history_sec * 100.0))

        # Vicon 상태
        self.t_v: deque[float] = deque(maxlen=history_size)
        self.x_v: deque[float] = deque(maxlen=history_size)
        self.y_v: deque[float] = deque(maxlen=history_size)
        self.th_v: deque[float] = deque(maxlen=history_size)

        # EKF 상태와 불확실도
        self.t_e: deque[float] = deque(maxlen=history_size)
        self.x_e: deque[float] = deque(maxlen=history_size)
        self.y_e: deque[float] = deque(maxlen=history_size)
        self.th_e: deque[float] = deque(maxlen=history_size)
        self.vx_e: deque[float] = deque(maxlen=history_size)
        self.vy_e: deque[float] = deque(maxlen=history_size)
        self.om_e: deque[float] = deque(maxlen=history_size)
        self.std_x: deque[float] = deque(maxlen=history_size)
        self.std_y: deque[float] = deque(maxlen=history_size)
        self.std_th: deque[float] = deque(maxlen=history_size)

        # 유도 기준
        self.t_r: deque[float] = deque(maxlen=history_size)
        self.ref_x: deque[float] = deque(maxlen=history_size)
        self.ref_y: deque[float] = deque(maxlen=history_size)
        self.ref_th: deque[float] = deque(maxlen=history_size)
        self.latest_ref: tuple[float, float, float] | None = None

        # 추종 오차
        self.t_err: deque[float] = deque(maxlen=history_size)
        self.pos_err: deque[float] = deque(maxlen=history_size)
        self.hdg_err: deque[float] = deque(maxlen=history_size)

        # 밸브와 추정 렌치
        self.valve_cmd = [0.0] * 8
        self.valve_app = [0.0] * 8
        self.valve_thrust = [0.0] * 8
        self.wrench = np.zeros(3)
        self._thruster_matrix = _load_thruster_matrix()
        self._plc_max = _load_plc_max_value()
        self._max_thrust = _load_max_thrust()

        # 시스템 상태
        self.mode_name = "NO DATA"
        self.navigation_ok = False
        self.actuators_ok = False
        self.main_valve_open = False
        self.navigation_degraded = False
        self.navigation_age_sec = math.inf
        self.control_loop_rate_hz = 0.0
        self.modbus_rtt_ms = 0.0
        self.warnings: list[str] = []
        self.errors: list[str] = []

        # 수신률과 신선도
        self.vicon_hz = 0.0
        self.ekf_hz = 0.0
        self.imu_hz = 0.0
        self._vicon_count = 0
        self._ekf_count = 0
        self._imu_count = 0
        self._last_rx: dict[str, float] = {}
        self._t0: float | None = None

        # 제어 명령 응답 (버튼 → 서비스 호출 결과 표시용)
        self.last_command_msg = ""

        # ROS 로그
        self.logs: deque[str] = deque(maxlen=200)

        self.create_subscription(
            SpacecraftState, "/sensors/vicon_state", self._cb_vicon, 10
        )
        self.create_subscription(
            SpacecraftState, "/navigation/state", self._cb_ekf, 10
        )
        self.create_subscription(
            ValveStatus, "/actuators/valve_status", self._cb_valve, 10
        )
        self.create_subscription(
            TrajectoryReference, "/gnc/trajectory_ref", self._cb_ref, 10
        )
        self.create_subscription(SystemMode, "/system/mode", self._cb_mode, 10)
        self.create_subscription(SystemHealth, "/system/health", self._cb_health, 10)
        self.create_subscription(Imu, "/sensors/imu", self._cb_imu, 20)
        self.create_subscription(Log, "/rosout", self._cb_log, 50)
        self.create_timer(1.0, self._tick_hz)

        # 제어 서비스 클라이언트 (E-STOP/RESET/모드/메인밸브 버튼)
        self._set_mode_cli = self.create_client(SetSystemMode, "/system/set_mode")
        self._main_valve_cli = self.create_client(MainValveControl, "/actuators/main_valve")

    def request_mode(self, mode: int) -> None:
        """모드 전이 서비스를 비동기 호출한다 (응답은 상태줄에 표시)."""
        if not self._set_mode_cli.service_is_ready():
            self.last_command_msg = "set_mode 서비스 미연결 — manager 실행 확인"
            return
        request = SetSystemMode.Request()
        request.requested_mode = mode
        self._set_mode_cli.call_async(request).add_done_callback(self._on_service_done)

    def request_main_valve(self, open_valve: bool) -> None:
        """메인 밸브(에어베어링) 서비스를 비동기 호출한다."""
        if not self._main_valve_cli.service_is_ready():
            self.last_command_msg = "main_valve 서비스 미연결 — valve_driver 실행 확인"
            return
        request = MainValveControl.Request()
        request.open = open_valve
        self._main_valve_cli.call_async(request).add_done_callback(self._on_service_done)

    def _on_service_done(self, future) -> None:
        try:
            self.last_command_msg = future.result().message
        except Exception as exc:  # noqa: BLE001 — 서비스 실패를 UI에 표시
            self.last_command_msg = f"서비스 호출 실패: {exc}"

    def readiness(self) -> dict[str, bool]:
        """서브시스템 준비도 (health + 신선도에서 유도 — manager 게이팅과 동일 기준)."""
        health_fresh = self.source_age("health") <= SOURCE_TIMEOUT_SEC
        ekf_fresh = self.source_age("ekf") <= SOURCE_TIMEOUT_SEC
        return {
            "vicon": self.source_age("vicon") <= SOURCE_TIMEOUT_SEC,
            "imu": self.source_age("imu") <= SOURCE_TIMEOUT_SEC,
            "nav": health_fresh and ekf_fresh and self.navigation_ok
            and not self.navigation_degraded,
            "act": health_fresh and self.actuators_ok,
            "main_valve": self.main_valve_open,
        }

    def source_age(self, source: str) -> float:
        """마지막 토픽 수신 이후 경과 시간을 wall-clock 기준으로 반환한다."""
        last = self._last_rx.get(source)
        return math.inf if last is None else time.monotonic() - last

    def _mark_rx(self, source: str) -> None:
        self._last_rx[source] = time.monotonic()

    def _relative_time(self, stamp) -> float:
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = timestamp
        return timestamp - self._t0

    def _cb_vicon(self, msg: SpacecraftState) -> None:
        timestamp = self._relative_time(msg.header.stamp)
        self.t_v.append(timestamp)
        self.x_v.append(msg.x)
        self.y_v.append(msg.y)
        self.th_v.append(math.degrees(msg.theta))
        self._vicon_count += 1
        self._mark_rx("vicon")

    def _cb_ekf(self, msg: SpacecraftState) -> None:
        timestamp = self._relative_time(msg.header.stamp)
        self.t_e.append(timestamp)
        self.x_e.append(msg.x)
        self.y_e.append(msg.y)
        self.th_e.append(math.degrees(msg.theta))
        self.vx_e.append(msg.vx)
        self.vy_e.append(msg.vy)
        self.om_e.append(math.degrees(msg.omega))
        self.std_x.append(msg.std_x)
        self.std_y.append(msg.std_y)
        self.std_th.append(math.degrees(msg.std_theta))

        if self.latest_ref is not None:
            ref_x, ref_y, ref_theta = self.latest_ref
            self.t_err.append(timestamp)
            self.pos_err.append(math.hypot(msg.x - ref_x, msg.y - ref_y))
            self.hdg_err.append(
                math.degrees(_wrap_angle_rad(msg.theta - ref_theta))
            )

        self._ekf_count += 1
        self._mark_rx("ekf")

    def _cb_valve(self, msg: ValveStatus) -> None:
        self.valve_cmd = list(msg.commanded_duty_cycles)
        self.valve_app = [
            max(0.0, min(1.0, float(value) / self._plc_max))
            for value in msg.applied_values
        ]
        estimated_thrust = list(msg.estimated_thrust)
        # 캘리브레이션 모델이 아직 0만 발행하면 PLC 적용값으로 선형 추정한다.
        if any(abs(value) > 1e-12 for value in estimated_thrust):
            self.valve_thrust = estimated_thrust
        else:
            self.valve_thrust = [
                value * self._max_thrust for value in self.valve_app
            ]
        thrust = np.asarray(self.valve_thrust, dtype=float)
        if thrust.shape == (self._thruster_matrix.shape[1],):
            self.wrench = self._thruster_matrix @ thrust
        self.main_valve_open = msg.main_valve_open
        self.modbus_rtt_ms = msg.modbus_rtt_ms
        self._mark_rx("valve")

    def _cb_ref(self, msg: TrajectoryReference) -> None:
        timestamp = self._relative_time(msg.header.stamp)
        self.t_r.append(timestamp)
        self.ref_x.append(msg.x_ref)
        self.ref_y.append(msg.y_ref)
        self.ref_th.append(math.degrees(msg.theta_ref))
        self.latest_ref = (msg.x_ref, msg.y_ref, msg.theta_ref)
        self._mark_rx("reference")

    def _cb_imu(self, msg: Imu) -> None:
        self._imu_count += 1
        self._mark_rx("imu")

    def _cb_mode(self, msg: SystemMode) -> None:
        self.mode_name = msg.mode_name or f"MODE {msg.mode}"
        self._mark_rx("mode")

    def _cb_health(self, msg: SystemHealth) -> None:
        self.navigation_ok = msg.navigation_ok
        self.actuators_ok = msg.actuators_ok
        self.main_valve_open = msg.main_valve_open
        self.navigation_degraded = msg.navigation_degraded
        self.navigation_age_sec = msg.navigation_age_sec
        self.control_loop_rate_hz = msg.control_loop_rate_hz
        self.modbus_rtt_ms = msg.modbus_rtt_ms
        self.warnings = list(msg.warnings)
        self.errors = list(msg.errors)
        self._mark_rx("health")

    _LOG_LEVELS = {10: "DBG", 20: "INF", 30: "WRN", 40: "ERR", 50: "FTL"}

    def _cb_log(self, msg: Log) -> None:
        if msg.level < 20:
            return
        level = self._LOG_LEVELS.get(msg.level, "?")
        node_name = msg.name.split(".")[-1] if "." in msg.name else msg.name
        color = C_ERROR if msg.level >= 40 else C_WARN if msg.level >= 30 else C_TEXT_DIM
        safe_name = html.escape(node_name)
        safe_message = html.escape(msg.msg[:160])
        self.logs.append(
            f'<span style="color:{color}">[{level}] {safe_name}: {safe_message}</span>'
        )

    def _tick_hz(self) -> None:
        self.vicon_hz = float(self._vicon_count)
        self.ekf_hz = float(self._ekf_count)
        self.imu_hz = float(self._imu_count)
        self._vicon_count = 0
        self._ekf_count = 0
        self._imu_count = 0


class StatusCard(QFrame):
    """상태 하나를 큰 글씨와 색으로 표시하는 카드."""

    def __init__(self, title: str) -> None:
        super().__init__()
        self.setObjectName("statusCard")
        self.setMinimumHeight(92)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(13, 9, 13, 9)
        layout.setSpacing(2)

        title_label = QLabel(title.upper())
        title_label.setStyleSheet(
            f"color:{C_TEXT_DIM}; font-size:11pt; font-weight:600; border:none;"
        )
        self._value = QLabel("—")
        self._value.setFont(QFont("sans-serif", 22, QFont.Bold))
        self._value.setStyleSheet(f"color:{C_TEXT}; border:none;")
        self._detail = QLabel("데이터 대기 중")
        self._detail.setStyleSheet(f"color:{C_TEXT_DIM}; font-size:11pt; border:none;")

        layout.addWidget(title_label)
        layout.addWidget(self._value)
        layout.addWidget(self._detail)
        self.set_status("—", "데이터 대기 중", C_IDLE)

    def set_status(self, value: str, detail: str, color: str) -> None:
        self._value.setText(value)
        self._value.setStyleSheet(f"color:{color}; border:none;")
        self._detail.setText(detail)
        self.setStyleSheet(
            f"QFrame#statusCard {{ background:{C_PANEL}; border:1px solid {C_BORDER}; "
            f"border-left:4px solid {color}; border-radius:7px; }}"
        )


class ReadinessLight(QFrame):
    """서브시스템 하나의 준비 상태를 색 + 값으로 표시하는 컴팩트 표시등."""

    def __init__(self, name: str) -> None:
        super().__init__()
        self.setObjectName("readyLight")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(11, 7, 11, 7)
        layout.setSpacing(1)
        name_label = QLabel(name.upper())
        name_label.setStyleSheet(
            f"color:{C_TEXT_DIM}; font-size:11pt; font-weight:600; border:none;"
        )
        self._value = QLabel("—")
        self._value.setFont(QFont("monospace", 13, QFont.Bold))
        self._value.setStyleSheet(f"color:{C_TEXT}; border:none;")
        layout.addWidget(name_label)
        layout.addWidget(self._value)
        self.set_state(C_IDLE, "—")

    def set_state(self, color: str, value: str) -> None:
        self._value.setText(value)
        self._value.setStyleSheet(f"color:{color}; border:none;")
        self.setStyleSheet(
            f"QFrame#readyLight {{ background:{C_PANEL}; border:1px solid {C_BORDER}; "
            f"border-left:5px solid {color}; border-radius:6px; }}"
        )


def _make_button(text: str, color: str, *, big: bool = False) -> QPushButton:
    """관제 콘솔용 버튼. 비활성 시 흐리게 표시된다."""
    button = QPushButton(text)
    button.setCursor(Qt.PointingHandCursor)
    height = 64 if big else 44
    font_size = 20 if big else 13
    button.setMinimumHeight(height)
    button.setStyleSheet(
        f"""
        QPushButton {{ background:{color}; color:#0b0f14; border:none; border-radius:7px;
                       font-size:{font_size}pt; font-weight:700; padding:4px 10px; }}
        QPushButton:hover {{ background:{color}; }}
        QPushButton:disabled {{ background:{C_PANEL}; color:{C_IDLE};
                                border:1px solid {C_BORDER}; }}
        """
    )
    return button


class Dashboard(QMainWindow):
    """FSS 미션 관제용 실시간 대시보드."""

    def __init__(self, node: DashboardNode, fps: int = 30) -> None:
        super().__init__()
        self._node = node
        self._series_bindings: list[tuple[pg.PlotDataItem, str, str]] = []
        self._last_log_count = -1

        self.setWindowTitle("FSS Mission Control")
        self.resize(1720, 1000)
        self.setMinimumSize(1320, 820)
        self.setStyleSheet(
            f"""
            QMainWindow, QWidget {{ background-color:{C_BG}; color:{C_TEXT}; }}
            QLabel {{ color:{C_TEXT}; }}
            QFrame#panel {{ background:{C_PANEL}; border:1px solid {C_BORDER};
                            border-radius:7px; }}
            QTabWidget::pane {{ border:1px solid {C_BORDER}; background:{C_PANEL};
                            border-radius:6px; }}
            QTabBar::tab {{ background:{C_BG}; color:{C_TEXT_DIM}; padding:9px 20px;
                            border:1px solid {C_BORDER}; border-bottom:none;
                            font-size:12pt; font-weight:600; min-height:22px; }}
            QTabBar::tab:selected {{ background:{C_PANEL}; color:{C_INFO};
                                     border-bottom:2px solid {C_INFO}; }}
            QTextEdit {{ background:{C_PANEL_ALT}; color:{C_TEXT};
                         border:1px solid {C_BORDER}; font-family:monospace;
                         font-size:10pt; padding:6px; }}
            QSplitter::handle {{ background:{C_BG}; }}
            """
        )

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        root.addLayout(self._build_header())
        root.addWidget(self._build_control_bar())

        top_splitter = QSplitter(Qt.Horizontal)
        top_splitter.setChildrenCollapsible(False)
        top_splitter.addWidget(self._build_trajectory_panel())
        top_splitter.addWidget(self._build_right_column())
        top_splitter.setStretchFactor(0, 7)
        top_splitter.setStretchFactor(1, 3)
        root.addWidget(top_splitter, stretch=3)

        self._tabs = self._build_detail_tabs()
        root.addWidget(self._tabs, stretch=2)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update)
        self._timer.start(max(16, round(1000 / max(1, fps))))

    def _build_header(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        layout.setSpacing(8)

        title_box = QVBoxLayout()
        title = QLabel("FSS")
        title.setFont(QFont("sans-serif", 28, QFont.Bold))
        title.setStyleSheet(f"color:{C_TEXT};")
        subtitle = QLabel("MISSION CONTROL")
        subtitle.setStyleSheet(
            f"color:{C_TEXT_DIM}; font-size:11pt; font-weight:600; letter-spacing:2px;"
        )
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        title_box.addStretch()
        layout.addLayout(title_box, stretch=2)

        self._card_mode = StatusCard("Mode")
        self._card_nav = StatusCard("Navigation")
        self._card_act = StatusCard("Actuators")
        self._card_main = StatusCard("Main valve")
        self._card_timing = StatusCard("Data link")
        for card in (
            self._card_mode,
            self._card_nav,
            self._card_act,
            self._card_main,
            self._card_timing,
        ):
            layout.addWidget(card, stretch=2)
        return layout

    def _build_control_bar(self) -> QWidget:
        """E-STOP + 모드 스테퍼 + RESET + 에어베어링 토글 + 명령 상태."""
        bar = QFrame()
        bar.setObjectName("panel")
        layout = QHBoxLayout(bar)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(10)

        self._btn_estop = _make_button("E-STOP", C_ERROR, big=True)
        self._btn_estop.setMinimumWidth(190)
        self._btn_estop.clicked.connect(lambda: self._node.request_mode(MODE_ESTOP))
        layout.addWidget(self._btn_estop)

        # 모드 스테퍼 (준비도 미충족 시 비활성)
        self._btn_pressurize = _make_button("① PRESSURIZE", C_WARN)
        self._btn_pressurize.clicked.connect(lambda: self._node.request_mode(MODE_PRESSURIZING))
        self._btn_ready = _make_button("② READY", C_INFO)
        self._btn_ready.clicked.connect(lambda: self._node.request_mode(MODE_READY))
        self._btn_active = _make_button("③ ACTIVE", C_OK)
        self._btn_active.clicked.connect(lambda: self._node.request_mode(MODE_ACTIVE))
        self._btn_abort = _make_button("ABORT → HOLD", C_WARN)
        self._btn_abort.clicked.connect(lambda: self._node.request_mode(MODE_SAFE_HOLD))
        self._btn_reset = _make_button("RESET → IDLE", C_INFO)
        self._btn_reset.clicked.connect(lambda: self._node.request_mode(MODE_IDLE))
        for button in (
            self._btn_pressurize, self._btn_ready, self._btn_active,
            self._btn_abort, self._btn_reset,
        ):
            layout.addWidget(button)

        # 에어베어링(메인 밸브) 토글
        self._btn_air_on = _make_button("AIR ON", C_OK)
        self._btn_air_on.clicked.connect(lambda: self._node.request_main_valve(True))
        self._btn_air_off = _make_button("AIR OFF", C_TEXT_DIM)
        self._btn_air_off.clicked.connect(lambda: self._node.request_main_valve(False))
        layout.addWidget(self._btn_air_on)
        layout.addWidget(self._btn_air_off)

        layout.addStretch()
        self._command_status = QLabel("")
        self._command_status.setWordWrap(True)
        self._command_status.setMinimumWidth(240)
        self._command_status.setStyleSheet(f"color:{C_TEXT_DIM}; font-size:11pt; border:none;")
        layout.addWidget(self._command_status)
        return bar

    def _build_right_column(self) -> QWidget:
        column = QWidget()
        layout = QVBoxLayout(column)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(8)
        layout.addWidget(self._build_readiness_panel())
        layout.addWidget(self._build_summary_panel(), stretch=1)
        return column

    def _build_readiness_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(8)
        title = QLabel("서브시스템 준비도")
        title.setFont(QFont("sans-serif", 14, QFont.Bold))
        title.setStyleSheet(f"color:{C_INFO}; border:none;")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setHorizontalSpacing(8)
        grid.setVerticalSpacing(8)
        self._lights: dict[str, ReadinessLight] = {}
        specs = [
            ("Vicon", "vicon"), ("IMU", "imu"), ("EKF / Nav", "nav"),
            ("PLC / Valve", "act"), ("Air bearing", "main_valve"),
        ]
        for index, (label, key) in enumerate(specs):
            light = ReadinessLight(label)
            self._lights[key] = light
            grid.addWidget(light, index // 2, index % 2)
        layout.addLayout(grid)
        return panel

    def _build_trajectory_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(8, 8, 8, 8)

        self._xy_plot = pg.PlotWidget()
        self._xy_plot.setAspectLocked(True)
        self._xy_plot.setXRange(-2.0, 2.0)
        self._xy_plot.setYRange(-1.5, 1.5)
        _style_plot(
            self._xy_plot,
            "XY 궤적",
            x_label="X [m]",
            y_label="Y [m]",
        )
        _add_readable_legend(self._xy_plot, (10, 10))

        self._vicon_curve = self._xy_plot.plot(
            pen=None,
            symbol="o",
            symbolSize=4,
            symbolBrush=C_VICON,
            symbolPen=None,
            name="Vicon",
        )
        self._ekf_curve = self._xy_plot.plot(
            pen=pg.mkPen(C_EKF, width=2.6), name="EKF"
        )
        self._ref_curve = self._xy_plot.plot(
            pen=pg.mkPen(C_REFERENCE, width=2.2, style=Qt.DashLine), name="Reference"
        )

        self._spacecraft_body = pg.QtWidgets.QGraphicsRectItem(
            -BODY_HALF_W, -BODY_HALF_W, BODY_HALF_W * 2, BODY_HALF_W * 2
        )
        self._spacecraft_body.setPen(pg.mkPen(C_TEXT_DIM, width=1.5))
        self._spacecraft_body.setBrush(QBrush(QColor(48, 54, 61, 150)))
        self._xy_plot.addItem(self._spacecraft_body)

        self._spacecraft_front = pg.QtWidgets.QGraphicsLineItem(0, 0, 0, 0)
        self._spacecraft_front.setPen(pg.mkPen(C_ERROR, width=3))
        self._xy_plot.addItem(self._spacecraft_front)
        self._position_dot = self._xy_plot.plot(
            pen=None,
            symbol="o",
            symbolSize=10,
            symbolBrush=C_EKF,
            symbolPen=pg.mkPen("#ffffff", width=1),
        )

        layout.addWidget(self._xy_plot)
        return panel

    def _build_summary_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("panel")
        panel.setMinimumWidth(380)
        panel.setMaximumWidth(500)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(8)

        title = QLabel("현재 / 목표 / 오차")
        title.setFont(QFont("sans-serif", 15, QFont.Bold))
        title.setStyleSheet(f"color:{C_INFO}; border:none;")
        layout.addWidget(title)

        grid = QGridLayout()
        grid.setHorizontalSpacing(10)
        grid.setVerticalSpacing(8)
        self._summary_values: dict[str, QLabel] = {}
        rows = [
            ("현재 위치", "current_pos"),
            ("목표 위치", "target_pos"),
            ("위치 오차", "position_error"),
            ("현재 방향", "current_heading"),
            ("목표 방향", "target_heading"),
            ("방향 오차", "heading_error"),
            ("속도", "velocity"),
            ("명령 렌치", "wrench"),
        ]
        for row, (name, key) in enumerate(rows):
            name_label = QLabel(name)
            name_label.setStyleSheet(f"color:{C_TEXT_DIM}; font-size:12pt; border:none;")
            value_label = QLabel("—")
            value_label.setFont(QFont("monospace", 16, QFont.Bold))
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            value_label.setStyleSheet(f"color:{C_TEXT}; border:none;")
            grid.addWidget(name_label, row, 0)
            grid.addWidget(value_label, row, 1)
            self._summary_values[key] = value_label
        layout.addLayout(grid)

        separator = QFrame()
        separator.setFrameShape(QFrame.HLine)
        separator.setStyleSheet(f"border:none; border-top:1px solid {C_BORDER};")
        layout.addWidget(separator)

        alert_title = QLabel("상태 메시지")
        alert_title.setStyleSheet(f"color:{C_TEXT_DIM}; font-size:10pt; border:none;")
        self._alert = QLabel("데이터 대기 중")
        self._alert.setWordWrap(True)
        self._alert.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        self._alert.setStyleSheet(
            f"color:{C_TEXT_DIM}; background:{C_PANEL_ALT}; border:1px solid {C_BORDER}; "
            "border-radius:5px; padding:8px; font-size:10pt;"
        )
        layout.addWidget(alert_title)
        layout.addWidget(self._alert, stretch=1)
        return panel

    def _make_plot(
        self,
        title: str,
        series: list[SeriesSpec],
        y_label: str = "",
        y_range: tuple[float, float] | None = None,
        min_y_span: float | None = None,
    ) -> pg.PlotWidget:
        """공통 스타일의 시계열 플롯을 만들고 데이터 바인딩을 등록한다.

        min_y_span을 주면 자동 스케일이 그보다 좁아지지 않도록 최소 Y 범위를 강제한다
        (작은 신호에서 축이 확대돼 잡음이 요동치는 현상 방지).
        """
        plot = pg.PlotWidget()
        _style_plot(plot, title, x_label="시간 [s]", y_label=y_label)
        _add_readable_legend(plot, (8, 8), min(3, len(series)))
        if y_range is not None:
            plot.setYRange(*y_range)
        if min_y_span is not None:
            plot.getViewBox().setLimits(minYRange=min_y_span)

        for spec in series:
            if spec.line_style not in LINE_STYLES:
                choices = ", ".join(LINE_STYLES)
                raise ValueError(f"선 종류는 {choices} 중 하나여야 합니다.")
            curve = plot.plot(
                pen=pg.mkPen(
                    spec.color,
                    width=spec.width,
                    style=LINE_STYLES[spec.line_style],
                ),
                name=spec.label,
            )
            self._series_bindings.append(
                (curve, spec.time_attr, spec.value_attr)
            )
        return plot

    def _build_detail_tabs(self) -> QTabWidget:
        """상세 탭을 구성한다.

        새 plot은 아래처럼 ``_make_plot`` 호출과 ``SeriesSpec``만 추가하면 된다.
        ROS 콜백에서 같은 길이의 시간 deque와 값 deque를 준비해야 한다.
        """
        tabs = QTabWidget()

        # 초기 탭: 운용 판단에 가장 중요한 추종 오차
        error_tab = QWidget()
        error_layout = QHBoxLayout(error_tab)
        error_layout.setContentsMargins(6, 6, 6, 6)
        error_layout.addWidget(
            self._make_plot(
                "위치 추종 오차",
                [SeriesSpec("t_err", "pos_err", "|eₚ|", C_ERROR, 2.0)],
                "m",
                (0.0, 0.25),
            )
        )
        error_layout.addWidget(
            self._make_plot(
                "방향 추종 오차",
                [SeriesSpec("t_err", "hdg_err", "eθ", C_WARN, 2.0)],
                "deg",
                (-15.0, 15.0),
            )
        )
        tabs.addTab(error_tab, "추종 오차")

        state_tab = QWidget()
        state_layout = QGridLayout(state_tab)
        state_layout.setContentsMargins(6, 6, 6, 6)
        state_layout.addWidget(
            self._make_plot(
                "X 위치",
                [
                    SeriesSpec("t_e", "x_e", "EKF", C_EKF, 2.5),
                    SeriesSpec("t_v", "x_v", "Vicon", C_VICON, 2.0, "dotted"),
                    SeriesSpec(
                        "t_r", "ref_x", "Reference", C_REFERENCE, 2.2, "dashed"
                    ),
                ],
                "m",
                min_y_span=0.2,
            ),
            0,
            0,
        )
        state_layout.addWidget(
            self._make_plot(
                "Y 위치",
                [
                    SeriesSpec("t_e", "y_e", "EKF", C_EKF, 2.5),
                    SeriesSpec("t_v", "y_v", "Vicon", C_VICON, 2.0, "dotted"),
                    SeriesSpec(
                        "t_r", "ref_y", "Reference", C_REFERENCE, 2.2, "dashed"
                    ),
                ],
                "m",
                min_y_span=0.2,
            ),
            0,
            1,
        )
        state_layout.addWidget(
            self._make_plot(
                "테이블 프레임 속도",
                [
                    SeriesSpec("t_e", "vx_e", "vx", "#ff7b72"),
                    SeriesSpec("t_e", "vy_e", "vy", "#d2a8ff"),
                ],
                "m/s",
                min_y_span=0.1,
            ),
            1,
            0,
        )
        state_layout.addWidget(
            self._make_plot(
                "방향과 각속도",
                [
                    SeriesSpec("t_e", "th_e", "θ EKF", C_EKF, 2.5),
                    SeriesSpec(
                        "t_r", "ref_th", "θ Ref", C_REFERENCE, 2.2, "dashed"
                    ),
                    SeriesSpec("t_e", "om_e", "ω", C_WARN),
                ],
                "deg / deg/s",
                min_y_span=20.0,
            ),
            1,
            1,
        )
        tabs.addTab(state_tab, "상태 추세")

        quality_tab = QWidget()
        quality_layout = QHBoxLayout(quality_tab)
        quality_layout.setContentsMargins(6, 6, 6, 6)
        quality_layout.addWidget(
            self._make_plot(
                "위치 추정 불확실도 (1σ)",
                [
                    SeriesSpec("t_e", "std_x", "σx", C_VICON),
                    SeriesSpec("t_e", "std_y", "σy", C_EKF),
                ],
                "m",
                min_y_span=0.02,
            )
        )
        quality_layout.addWidget(
            self._make_plot(
                "방향 추정 불확실도 (1σ)",
                [SeriesSpec("t_e", "std_th", "σθ", C_WARN)],
                "deg",
                min_y_span=2.0,
            )
        )
        tabs.addTab(quality_tab, "추정 품질")

        valve_tab = QWidget()
        valve_layout = QHBoxLayout(valve_tab)
        valve_layout.setContentsMargins(6, 6, 6, 6)

        self._valve_plot = pg.PlotWidget()
        self._valve_plot.setYRange(0.0, 1.05)
        _style_plot(
            self._valve_plot,
            "밸브 명령 / 적용값",
            y_label="정규화 값",
            grid_x=False,
        )
        _add_readable_legend(self._valve_plot, (8, 8), 2)
        valve_labels = ["+X₁", "+X₂", "−X₁", "−X₂", "+Y₁", "+Y₂", "−Y₁", "−Y₂"]
        self._valve_plot.getAxis("bottom").setTicks(
            [[(index, label) for index, label in enumerate(valve_labels)]]
        )
        self._valve_cmd_bar = pg.BarGraphItem(
            x=np.arange(8) - 0.18,
            height=self._node.valve_cmd,
            width=0.32,
            brush=C_WRENCH_POS,
            name="명령",
        )
        self._valve_app_bar = pg.BarGraphItem(
            x=np.arange(8) + 0.18,
            height=self._node.valve_app,
            width=0.32,
            brush=C_EKF,
            name="PLC 적용",
        )
        self._valve_plot.addItem(self._valve_cmd_bar)
        self._valve_plot.addItem(self._valve_app_bar)
        valve_layout.addWidget(self._valve_plot, stretch=3)

        self._thrust_plot = pg.PlotWidget()
        _style_plot(
            self._thrust_plot,
            "밸브별 추정 추력",
            y_label="N",
            grid_x=False,
        )
        self._thrust_plot.getAxis("bottom").setTicks(
            [[(index, label) for index, label in enumerate(valve_labels)]]
        )
        self._thrust_bar = pg.BarGraphItem(
            x=np.arange(8),
            height=self._node.valve_thrust,
            width=0.58,
            brush=C_INFO,
        )
        self._thrust_plot.addItem(self._thrust_bar)
        valve_layout.addWidget(self._thrust_plot, stretch=2)
        tabs.addTab(valve_tab, "밸브")

        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        tabs.addTab(self._log_text, "로그")
        return tabs

    def _set_summary_value(self, key: str, text: str, color: str = C_TEXT) -> None:
        label = self._summary_values[key]
        label.setText(text)
        label.setStyleSheet(f"color:{color}; border:none;")

    def _update_status_cards(self) -> None:
        node = self._node

        if node.source_age("mode") > SOURCE_TIMEOUT_SEC:
            self._card_mode.set_status("NO DATA", "/system/mode", C_ERROR)
        else:
            mode_color = MODE_COLORS.get(node.mode_name, C_IDLE)
            self._card_mode.set_status(node.mode_name, "system mode", mode_color)

        health_fresh = node.source_age("health") <= SOURCE_TIMEOUT_SEC
        ekf_fresh = node.source_age("ekf") <= SOURCE_TIMEOUT_SEC
        if not health_fresh or not ekf_fresh:
            self._card_nav.set_status("NO DATA", "health 또는 EKF 미수신", C_ERROR)
        elif node.navigation_degraded:
            self._card_nav.set_status(
                "DEGRADED", f"Vicon {node.vicon_hz:.0f} Hz · EKF {node.ekf_hz:.0f} Hz", C_WARN
            )
        elif node.navigation_ok:
            self._card_nav.set_status(
                "OK", f"Vicon {node.vicon_hz:.0f} Hz · EKF {node.ekf_hz:.0f} Hz", C_OK
            )
        else:
            self._card_nav.set_status("FAULT", "항법 상태 확인 필요", C_ERROR)

        valve_fresh = node.source_age("valve") <= SOURCE_TIMEOUT_SEC
        if not health_fresh or not valve_fresh:
            self._card_act.set_status("NO DATA", "/actuators/valve_status", C_ERROR)
        elif node.actuators_ok:
            peak = max(node.valve_cmd, default=0.0)
            self._card_act.set_status("OK", f"peak duty {peak:.2f}", C_OK)
        else:
            self._card_act.set_status("FAULT", "밸브/PLC 상태 확인", C_ERROR)

        if not valve_fresh:
            self._card_main.set_status("NO DATA", "밸브 상태 미수신", C_ERROR)
        elif node.main_valve_open:
            self._card_main.set_status("OPEN", "에어베어링 공급", C_OK)
        else:
            color = C_ERROR if node.mode_name in ("READY", "ACTIVE") else C_IDLE
            self._card_main.set_status("CLOSED", "에어베어링 차단", color)

        if not health_fresh:
            self._card_timing.set_status("NO DATA", "/system/health", C_ERROR)
        else:
            rate_color = C_OK if node.ekf_hz >= 30.0 else C_WARN
            rtt_text = "SIL" if node.modbus_rtt_ms == 0.0 else f"PLC {node.modbus_rtt_ms:.1f} ms"
            self._card_timing.set_status(
                f"EKF {node.ekf_hz:.0f} Hz", rtt_text, rate_color
            )

    def _update_summary(self) -> None:
        node = self._node
        if node.x_e:
            self._set_summary_value(
                "current_pos", f"({node.x_e[-1]:+.3f}, {node.y_e[-1]:+.3f}) m"
            )
            self._set_summary_value("current_heading", f"{node.th_e[-1]:+.2f}°")
            self._set_summary_value(
                "velocity", f"({node.vx_e[-1]:+.3f}, {node.vy_e[-1]:+.3f}) m/s"
            )
        else:
            for key in ("current_pos", "current_heading", "velocity"):
                self._set_summary_value(key, "—", C_TEXT_DIM)

        if node.latest_ref is not None:
            ref_x, ref_y, ref_theta = node.latest_ref
            self._set_summary_value("target_pos", f"({ref_x:+.3f}, {ref_y:+.3f}) m")
            self._set_summary_value("target_heading", f"{math.degrees(ref_theta):+.2f}°")
        else:
            self._set_summary_value("target_pos", "—", C_TEXT_DIM)
            self._set_summary_value("target_heading", "—", C_TEXT_DIM)

        if node.pos_err:
            position_error = node.pos_err[-1]
            heading_error = node.hdg_err[-1]
            pos_color = (
                C_OK if position_error < 0.03 else C_WARN if position_error < 0.10 else C_ERROR
            )
            hdg_color = (
                C_OK
                if abs(heading_error) < 2.0
                else C_WARN if abs(heading_error) < 8.0 else C_ERROR
            )
            self._set_summary_value("position_error", f"{position_error:.3f} m", pos_color)
            self._set_summary_value("heading_error", f"{heading_error:+.2f}°", hdg_color)
        else:
            self._set_summary_value("position_error", "—", C_TEXT_DIM)
            self._set_summary_value("heading_error", "—", C_TEXT_DIM)

        self._set_summary_value(
            "wrench",
            f"{node.wrench[0]:+.2f}, {node.wrench[1]:+.2f}, {node.wrench[2]:+.2f}",
        )

        if node.errors:
            self._alert.setText("ERROR  " + "\n".join(node.errors[:3]))
            self._alert.setStyleSheet(
                f"color:{C_ERROR}; background:{C_PANEL_ALT}; border:1px solid {C_ERROR}; "
                "border-radius:5px; padding:8px; font-size:10pt;"
            )
        elif node.warnings:
            self._alert.setText("WARNING  " + "\n".join(node.warnings[:3]))
            self._alert.setStyleSheet(
                f"color:{C_WARN}; background:{C_PANEL_ALT}; border:1px solid {C_WARN}; "
                "border-radius:5px; padding:8px; font-size:10pt;"
            )
        elif node.source_age("health") <= SOURCE_TIMEOUT_SEC:
            self._alert.setText("● 모든 감시 항목 정상")
            self._alert.setStyleSheet(
                f"color:{C_OK}; background:{C_PANEL_ALT}; border:1px solid {C_BORDER}; "
                "border-radius:5px; padding:8px; font-size:10pt;"
            )
        else:
            self._alert.setText("시스템 상태 데이터 대기 중")
            self._alert.setStyleSheet(
                f"color:{C_TEXT_DIM}; background:{C_PANEL_ALT}; border:1px solid {C_BORDER}; "
                "border-radius:5px; padding:8px; font-size:10pt;"
            )

    def _update_trajectory(self) -> None:
        node = self._node
        if node.x_v:
            self._vicon_curve.setData(list(node.x_v), list(node.y_v))
        if node.x_e:
            self._ekf_curve.setData(list(node.x_e), list(node.y_e))
            x, y = node.x_e[-1], node.y_e[-1]
            heading = math.radians(node.th_e[-1])

            transform = QTransform()
            transform.translate(x, y)
            transform.rotate(math.degrees(heading))
            self._spacecraft_body.setTransform(transform)

            cosine, sine = math.cos(heading), math.sin(heading)
            half_width = BODY_HALF_W
            x1 = x + cosine * half_width - sine * (-half_width * 0.7)
            y1 = y + sine * half_width + cosine * (-half_width * 0.7)
            x2 = x + cosine * half_width - sine * (half_width * 0.7)
            y2 = y + sine * half_width + cosine * (half_width * 0.7)
            self._spacecraft_front.setLine(x1, y1, x2, y2)
            self._position_dot.setData([x], [y])
        if node.ref_x:
            self._ref_curve.setData(list(node.ref_x), list(node.ref_y))

    def _update_detail_plots(self) -> None:
        node = self._node
        for curve, time_attr, value_attr in self._series_bindings:
            times = getattr(node, time_attr)
            values = getattr(node, value_attr)
            if times and len(times) == len(values):
                curve.setData(list(times), list(values))

        self._valve_cmd_bar.setOpts(height=node.valve_cmd)
        self._valve_app_bar.setOpts(height=node.valve_app)
        self._thrust_bar.setOpts(height=node.valve_thrust)

        if len(node.logs) != self._last_log_count:
            self._last_log_count = len(node.logs)
            self._log_text.setHtml("<br>".join(node.logs))
            scrollbar = self._log_text.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())

    def _update_controls(self) -> None:
        """준비도 등 + 버튼 활성/비활성 + 명령 상태를 갱신한다."""
        node = self._node
        ready = node.readiness()
        allowed = ALLOWED_TRANSITIONS.get(node.mode_name, set())

        # ── 준비도 등 ──
        self._lights["vicon"].set_state(
            C_OK if ready["vicon"] else C_ERROR,
            f"{node.vicon_hz:.0f} Hz" if ready["vicon"] else "미수신",
        )
        self._lights["imu"].set_state(
            C_OK if ready["imu"] else C_ERROR,
            f"{node.imu_hz:.0f} Hz" if ready["imu"] else "미수신",
        )
        if ready["nav"]:
            std_mm = max(node.std_x[-1], node.std_y[-1]) * 1000.0 if node.std_x else 0.0
            self._lights["nav"].set_state(C_OK, f"σ {std_mm:.0f} mm")
        elif node.navigation_degraded:
            self._lights["nav"].set_state(C_WARN, "저하")
        else:
            self._lights["nav"].set_state(C_ERROR, "미수렴")
        if ready["act"]:
            rtt = "SIL" if node.modbus_rtt_ms == 0.0 else f"{node.modbus_rtt_ms:.1f} ms"
            self._lights["act"].set_state(C_OK, rtt)
        else:
            self._lights["act"].set_state(C_ERROR, "미수신")
        armed_mode = node.mode_name in ("READY", "ACTIVE")
        if node.main_valve_open:
            self._lights["main_valve"].set_state(C_OK, "OPEN")
        else:
            self._lights["main_valve"].set_state(
                C_ERROR if armed_mode else C_IDLE, "CLOSED"
            )

        # ── 버튼 활성/비활성 (FSM 허용 전이 AND 준비도) ──
        def gate(button: QPushButton, target: int, ok: bool, why: str) -> None:
            allowed_now = target in allowed
            button.setEnabled(allowed_now and ok)
            button.setToolTip("" if (allowed_now and ok) else (why if allowed_now else "현재 모드에서 불가"))

        gate(self._btn_pressurize, MODE_PRESSURIZING, ready["act"], "PLC/밸브 미수신")
        gate(self._btn_ready, MODE_READY, ready["act"] and ready["main_valve"],
             "PLC 미수신 또는 에어베어링 미개방")
        gate(self._btn_active, MODE_ACTIVE,
             ready["nav"] and ready["vicon"] and ready["main_valve"],
             "항법 미수렴/Vicon 미수신 또는 에어베어링 미개방")
        gate(self._btn_abort, MODE_SAFE_HOLD, True, "")
        gate(self._btn_reset, MODE_IDLE, True, "")
        # 메인 밸브: E-STOP/SAFE_HOLD에선 열기 금지 (밸브드라이버 게이팅 미러)
        self._btn_air_on.setEnabled(node.mode_name not in ("EMERGENCY_STOP", "SAFE_HOLD", "NO DATA"))
        self._btn_air_off.setEnabled(node.mode_name != "NO DATA")

        if node.last_command_msg:
            self._command_status.setText(node.last_command_msg)

    def _update(self) -> None:
        """전체 대시보드를 갱신한다."""
        self._update_status_cards()
        self._update_controls()
        self._update_summary()
        self._update_trajectory()
        self._update_detail_plots()


def _parse_args() -> tuple[argparse.Namespace, list[str]]:
    parser = argparse.ArgumentParser(description="FSS 실시간 미션 대시보드")
    parser.add_argument(
        "--history-sec", type=float, default=30.0, help="표시할 이력 길이 [s]"
    )
    parser.add_argument("--fps", type=int, default=30, help="화면 갱신률")
    parser.add_argument(
        "--opengl", action="store_true", help="pyqtgraph OpenGL 렌더링 사용"
    )
    parser.add_argument(
        "--fullscreen", action="store_true", help="전체 화면으로 시작"
    )
    return parser.parse_known_args()


def main() -> None:
    args, ros_args = _parse_args()
    if args.history_sec <= 0.0:
        raise SystemExit("--history-sec는 양수여야 합니다.")
    if args.fps <= 0:
        raise SystemExit("--fps는 양수여야 합니다.")

    pg.setConfigOptions(
        antialias=True,
        background=C_BG,
        foreground=C_TEXT,
        useOpenGL=args.opengl,
    )

    rclpy.init(args=ros_args)
    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    node = DashboardNode(history_sec=args.history_sec)
    window = Dashboard(node, fps=args.fps)

    # Qt와 ROS 콜백을 같은 스레드에서 처리해 GUI 데이터 경합을 없앤다.
    ros_timer = QTimer(window)
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(2)

    if args.fullscreen:
        window.showFullScreen()
    else:
        window.show()

    try:
        exit_code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
