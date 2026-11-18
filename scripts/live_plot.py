#!/usr/bin/env python3
"""FSS 실시간 미션 관제 콘솔 (PySide6 + pyqtgraph).

FSS 연결, 기체별 XY 궤적, 밸브 명령 및 PID/LQR 선택을 제공한다.
Windows: py scripts/live_plot.py
ROS 환경: python3 scripts/live_plot.py
옵션:
    --history-sec 60   표시할 이력 길이 [s]
    --fps 20           화면 갱신률
    --opengl           pyqtgraph OpenGL 렌더링 사용
    --fullscreen       전체 화면으로 시작
"""

from __future__ import annotations

import argparse
import html
import math
import shlex
import sys
import time
from collections import Counter, deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QCoreApplication, QEvent, QLocale, QRectF, QSettings, QTimer, Qt
from PySide6.QtGui import QBrush, QColor, QFont, QFontDatabase, QFontMetricsF, QPainterPath, QTransform
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QDialog,
    QDialogButtonBox,
    QFormLayout,
    QInputDialog,
    QLineEdit,
    QMessageBox,
    QStackedWidget,
    QTabBar,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QLayout,
    QSizePolicy,
    QTabWidget,
    QTextEdit,
    QToolTip,
    QVBoxLayout,
    QWidget,
)

from mission_arm import ViconArmOverlay, feedback_source, joint_angles, link_points, plan_arm_points, pose_yaw_deg
from mission_link import JetsonLink, PREFERRED_WORKSPACE, SOURCE_TIMEOUT_SEC
from mission_recording import ExperimentRecorder
from mission_terminal import TerminalView
from mission_toss import toss_run_args
from mission_ros import open_node


# ── 색상 팔레트 ───────────────────────────────────────────────────────────────
C_BG = "#f5f5f7"
C_PANEL = "#ffffff"
C_PANEL_ALT = "#fafafc"
C_BORDER = "#e0e0e0"
C_TEXT = "#1d1d1f"
C_TEXT_DIM = "#636368"
C_INFO = "#0066cc"
C_FOCUS = "#0071e3"
C_IDLE = "#6e6e73"
C_VICON = "#0066cc"
C_ARM = "#087e74"
C_EKF = "#15736c"
C_REFERENCE = "#a35a10"
C_WARN = "#945109"
C_ERROR = "#b52b26"
C_OK = "#216b41"
LOG_LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARNING", 40: "ERROR", 50: "FATAL"}


@dataclass(frozen=True)
class LogEntry:
    timestamp: str
    level: int
    message: str

    def to_html(self):
        color = C_ERROR if self.level >= 40 else C_WARN if self.level >= 30 else C_TEXT_DIM
        level = LOG_LEVELS.get(self.level, str(self.level))
        return (f'<span style="color:{color}">{self.timestamp} &nbsp;[{level}] '
                f'{html.escape(self.message)}</span>')

BODY_HALF_W = 0.15  # 우주선 반폭 [m]
XY_TRAIL_SEC = 5.0
ARM_TIMEOUT_SEC = 0.5

MODE_COLORS = {
    "IDLE": C_IDLE,
    "PRESSURIZING": C_WARN,
    "READY": C_INFO,
    "ACTIVE": C_OK,
    "SAFE_HOLD": C_WARN,
    "EMERGENCY_STOP": C_ERROR,
}

MODE_ESTOP = 5


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
    """흰색 플롯에 축과 눈금의 공통 스타일을 적용한다."""
    plot.setBackground(C_PANEL)
    plot.setTitle(title, color=C_TEXT, size="12pt")
    label_style = {"color": C_TEXT_DIM, "font-size": "11pt"}
    if x_label:
        plot.setLabel("bottom", x_label, **label_style)
    if y_label:
        plot.setLabel("left", y_label, **label_style)

    tick_font = QFont(QApplication.font())
    tick_font.setPointSize(9)
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
        brush=pg.mkBrush(255, 255, 255, 235),
        pen=pg.mkPen(C_BORDER),
    )


def _wrap_angle_rad(angle: float) -> float:
    """각도를 [-π, π]로 정규화한다."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class XYTrail:
    def __init__(self):
        self.samples = deque()

    def append(self, x, y):
        now = time.monotonic()
        self._expire(now)
        self.samples.append((now, x, y))

    def _expire(self, now):
        cutoff = now - XY_TRAIL_SEC
        while self.samples and self.samples[0][0] <= cutoff:
            self.samples.popleft()

    def coordinates(self):
        self._expire(time.monotonic())
        return [p[1] for p in self.samples], [p[2] for p in self.samples]


class TablePlot(pg.PlotWidget):
    """Fit the physical table to the window without changing its coordinates."""

    LEFT_AXIS = 58
    BOTTOM_AXIS = 44

    def __init__(self):
        super().__init__()
        self.plotItem.layout.setContentsMargins(0, 0, 0, 0)
        self.plotItem.layout.setSpacing(0)
        self.getAxis("left").setWidth(self.LEFT_AXIS)
        self.getAxis("bottom").setHeight(self.BOTTOM_AXIS)
        for axis in ("left", "bottom"):
            self.getAxis(axis).setTickSpacing(1.0, 0.5)
            self.getAxis(axis).setStyle(hideOverlappingLabels=False)
        self.setMouseEnabled(x=False, y=False)
        self.plotItem.setMenuEnabled(False)
        self.plotItem.hideButtons()
        self.setLimits(xMin=0, xMax=3, yMin=0, yMax=4,
                       minXRange=3, maxXRange=3, minYRange=4, maxYRange=4)
        self.setRange(xRange=(0, 3), yRange=(0, 4), padding=0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.plotItem is None:
            return
        w, h = self.viewport().width(), self.viewport().height()
        pixels_per_m = max(1.0, min((w - self.LEFT_AXIS - 16) / 3, (h - self.BOTTOM_AXIS - 16) / 4))
        width = self.LEFT_AXIS + 3 * pixels_per_m
        height = self.BOTTOM_AXIS + 4 * pixels_per_m
        self.plotItem.setGeometry(QRectF((w - width) / 2, (h - height) / 2, width, height))


class VehicleBubble(pg.TextItem):
    """Vehicle-local readout with screen-sized text on the fixed table map."""

    def __init__(self):
        super().__init__(color=C_TEXT)
        font = QFont(QApplication.font())
        font.setPointSize(8)
        self.setFont(font)
        self.setZValue(20)
        self.setAcceptedMouseButtons(Qt.NoButton)
        self.hide()

    def paint(self, painter, *args):
        super().paint(painter, *args)
        painter.setRenderHint(painter.RenderHint.Antialiasing)
        painter.setPen(pg.mkPen(134, 145, 157, 110))
        painter.setBrush(pg.mkBrush(255, 255, 255, 206))
        painter.drawRoundedRect(self.boundingRect(), 6, 6)

    def show_vehicle(self, node, name=""):
        pose = None
        if node.x_v and node.source_age("vicon") < XY_TRAIL_SEC:
            pose = node.x_v[-1], node.y_v[-1], node.th_v[-1]
        elif node.arm_equipped and node.arm_table and node.source_age("arm_table") < XY_TRAIL_SEC:
            pose = node.arm_table_origin
        if pose is None or not all(math.isfinite(v) for v in pose):
            self.hide()
            return
        x, y, yaw = pose
        view = self.getViewBox()
        metrics = QFontMetricsF(self.textItem.font())
        max_width = max(1.0, view.sceneBoundingRect().width() - 2 * self.textItem.document().documentMargin() - 4)
        lines = [metrics.elidedText(name, Qt.ElideRight, max_width)] if name else []
        line = f"x {x:.2f}"
        for field in (f"y {y:.2f}", f"yaw {yaw:.1f}°"):
            candidate = line + "   " + field
            if metrics.horizontalAdvance(candidate) > max_width:
                lines.append(line)
                line = field
            else:
                line = candidate
        lines.append(line)
        label = "\n".join(lines)
        if self.toPlainText() != label:
            self.setText(label)
        # Grow toward the interior so labels remain readable at table edges.
        right, top = x > 1.5, y > 2
        self.setAnchor((int(right), int(not top)))
        dx, dy = view.viewPixelSize()
        bounds = self.boundingRect()
        width = min(3.0, abs(dx) * bounds.width())
        height = min(4.0, abs(dy) * bounds.height())
        bx = x + (-0.16 if right else 0.16)
        by = y + (-0.18 if top else 0.18)
        bx = max(width if right else 0, min(3 if right else 3 - width, bx))
        by = max(height if top else 0, min(4 if top else 4 - height, by))
        self.setPos(bx, by)
        self.show()

    def avoid_other_bubbles(self, occupied):
        if not self.isVisible():
            return
        box = self.sceneBoundingRect()
        view = self.getViewBox()
        bounds = view.sceneBoundingRect()
        tops = [box.top()]
        for other in occupied:
            if box.right() > other.left() and box.left() < other.right():
                tops.extend((other.top() - box.height() - 3, other.bottom() + 3))
        best = (math.inf, math.inf)
        placed = box
        for top in tops:
            top = max(bounds.top(), min(top, bounds.bottom() - box.height()))
            candidate = box.translated(0, top - box.top())
            overlap = 0
            for other in occupied:
                intersection = candidate.intersected(other)
                overlap += intersection.width() * intersection.height()
            score = (overlap, abs(candidate.top() - box.top()))
            if score < best:
                best, placed = score, candidate
                if score == (0, 0):
                    break
        y_offset = (placed.top() - box.top()) * abs(view.viewPixelSize()[1])
        self.setY(self.y() - y_offset)
        occupied.append(placed)


class DashboardNode:
    """대시보드에 필요한 ROS2 토픽을 수집한다."""

    def __init__(self, history_sec: float = 30.0) -> None:
        self.request = None
        self.recorder = ExperimentRecorder(self.recording_context)
        self.recording_status = None
        self.auto_recording_run = None
        self.auto_recording_deadline = None
        self.source_run = None
        self.target_revisions = {"paper_trial": 0, "reference": 0}
        self.recording_target_revisions = {}
        self.arm_equipped = True
        self.tracking_since = None
        self.target_draft = ("PID", 0.0, 0.0, 0.0)
        self.toss_method = "Nominal"
        self.control_view = "position"
        self.toss_options = ""
        self.toss_tuning_draft = {}
        self.toss_tuning_applied = None
        self.toss_phase = self.toss_phase_label = ""
        self.toss_gripper = self.toss_alignment = ""
        self.toss_contact = self.toss_aligned = False
        self.reported_recording_error = ""
        self.reported_video_error = ""
        history_size = max(100, int(history_sec * 100.0))

        # Vicon 상태
        self.vicon_trail = XYTrail()
        self.arm_tcp_trail = XYTrail()
        self.arm_overlay = ViconArmOverlay()
        self.arm_joints = self.arm_table = self.arm_base = self.arm_tcp = None
        self.arm_tcp_yaw_deg = None
        self.arm_table_origin = None
        self.arm_table_stamp = self.arm_tcp_stamp = None
        self.arm_gripper_pair = None
        self.arm_source = "출처 미확인"
        self.t_v: deque[float] = deque(maxlen=history_size)
        self.x_v: deque[float] = deque(maxlen=history_size)
        self.y_v: deque[float] = deque(maxlen=history_size)
        self.th_v: deque[float] = deque(maxlen=history_size)
        self.t_j = deque(maxlen=history_size)
        self.joint_history = [deque(maxlen=history_size) for _ in range(3)]

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
        self.paper_plans = {}
        self.plan_defaults = None
        self.paper_trial = ""
        self.paper_elapsed = None
        self.paper_match = []
        self.plan_choice = "auto"

        # 추종 오차
        self.t_err: deque[float] = deque(maxlen=history_size)
        self.pos_err: deque[float] = deque(maxlen=history_size)
        self.hdg_err: deque[float] = deque(maxlen=history_size)
        self.plan_samples = deque(maxlen=history_size)
        self.error_run = self.error_plan = self.trial_directory = ""
        self.comparison_source = None

        # 밸브 명령
        self.valve_cmd = None
        # 시스템 상태
        self.mode_name = "NO DATA"
        self.navigation_ok = False
        self.actuators_ok = False
        self._driver_air_open = None
        self._health_air_open = None
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
        self.logs: deque[LogEntry] = deque(maxlen=200)
        self.log_revision = 0

    def recording_context(self):
        return {"arm_equipped": self.arm_equipped, "selected_plan": self.plan_choice,
                "active_plan_id": self.paper_trial, "plans": self.paper_plans, "source_run": self.source_run,
                "tuning": self.toss_tuning_applied}

    def receive(self, name, msg):
        if name == "paper_status":
            status = msg if isinstance(msg, dict) else vars(msg)
            self.toss_tuning_applied = status.get("tuning")
        if name == "paper_status" and self.recording_status:
            self.recording_status(msg, False)
        if self.recorder.active:
            payload = msg
            if name == "valve":
                payload = {"commanded_duty_cycles": list(msg.commanded_duty_cycles),
                           "main_valve_open": msg.main_valve_open, "modbus_rtt_ms": msg.modbus_rtt_ms}
                if hasattr(msg, "header"):
                    payload["header"] = msg.header
            self.recorder.record(name, payload)
        getattr(self, "_cb_" + name)(msg)
        if name == "paper_status" and self.recording_status:
            self.recording_status(msg, True)

    def send_command(self, name, value):
        if self.request:
            self.recorder.record("command", {"name": name, "value": value})
            self.request(name, value)

    def request_mode(self, mode: int) -> None:
        self.send_command("mode", mode)

    def request_main_valve(self, open_valve: bool) -> None:
        self.send_command("valve", open_valve)

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
            "main_valve": self.reported_air_state() is True,
        }

    def reported_air_state(self):
        if self.source_age("valve") <= SOURCE_TIMEOUT_SEC:
            return self._driver_air_open
        if self.actuators_ok and self.source_age("health") <= SOURCE_TIMEOUT_SEC:
            return self._health_air_open
        return None

    def source_age(self, source: str) -> float:
        """마지막 토픽 수신 이후 경과 시간을 wall-clock 기준으로 반환한다."""
        last = self._last_rx.get(source)
        return math.inf if last is None else time.monotonic() - last

    def _mark_rx(self, source: str) -> None:
        self._last_rx[source] = time.monotonic()
        if source in self.target_revisions:
            self.target_revisions[source] += 1

    def tracking_age(self):
        age = self.source_age("vicon")
        if self.arm_equipped and self.arm_table is not None:
            age = min(age, self.source_age("arm_table"))
        return age

    def tracking_status(self):
        if self.tracking_since is None:
            self.tracking_since = time.monotonic()
        age = self.tracking_age()
        if age < XY_TRAIL_SEC:
            return "Online"
        if math.isinf(age) and time.monotonic() - self.tracking_since < XY_TRAIL_SEC:
            return "수신 대기"
        return "Missing"

    def _relative_time(self, stamp) -> float:
        timestamp = stamp.sec + stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = timestamp
        return timestamp - self._t0

    def _cb_vicon(self, msg) -> None:
        if not all(math.isfinite(value) for value in (msg.x, msg.y, msg.theta)):
            return
        self.vicon_trail.append(msg.x, msg.y)
        self.arm_overlay.add("table", msg.header, (msg.x, msg.y, msg.theta))
        timestamp = self._relative_time(msg.header.stamp)
        self.t_v.append(timestamp)
        self.x_v.append(msg.x)
        self.y_v.append(msg.y)
        self.th_v.append(math.degrees(msg.theta))
        self._vicon_count += 1
        self._mark_rx("vicon")

    def _cb_arm_joints(self, msg):
        if (sample := joint_angles(msg)) is not None:
            self.arm_joints = sample
            self._mark_rx("arm_joints")
            stamp = getattr(getattr(msg, "header", None), "stamp", None)
            if stamp is not None:
                self.t_j.append(self._relative_time(stamp))
                for history, angle in zip(self.joint_history, sample):
                    history.append(math.degrees(angle))

    def _cb_arm_driver(self, msg):
        source = feedback_source(msg.data)
        if self.arm_source != "출처 미확인" and self.arm_source != source:
            self.arm_tcp_trail.samples.clear()
            self.arm_joints = self.arm_table = self.arm_base = self.arm_tcp = None
            self.arm_tcp_yaw_deg = None
            self.arm_table_origin = self.arm_table_stamp = self.arm_tcp_stamp = None
            self.arm_gripper_pair = None
            self.arm_overlay.links.clear()
        self.arm_source = source
        self._mark_rx("arm_driver")

    def _cb_arm_table(self, msg):
        if (sample := link_points(msg, "table")) is not None:
            self.arm_table = sample
            stamp = getattr(msg.header, "stamp", None)
            self.arm_table_stamp = (stamp.sec, stamp.nanosec) if stamp else None
            q = msg.poses[0].orientation
            self.arm_table_origin = None
            if math.isfinite(q.z) and math.isfinite(q.w) and math.hypot(q.z, q.w) > 1e-9:
                self.arm_table_origin = (*sample[0][0], math.degrees(_wrap_angle_rad(2 * math.atan2(q.z, q.w))))
            self._mark_rx("arm_table")
            self._pair_gripper_sample()

    def _cb_arm_base(self, msg):
        if (sample := link_points(msg, "base_link")) is not None:
            self.arm_base = sample
            self.arm_overlay.add("base_link", msg.header, sample)
            self._mark_rx("arm_base")

    def _cb_arm_registration(self, values):
        self.arm_overlay.configure(values)

    def _cb_arm_tcp(self, msg):
        p = msg.pose.position
        if msg.header.frame_id != "table" or not all(math.isfinite(v) for v in (p.x, p.y)):
            return
        age = self.source_age("arm_tcp")
        if ARM_TIMEOUT_SEC < age < XY_TRAIL_SEC:
            self.arm_tcp_trail.append(math.nan, math.nan)
        self.arm_tcp_trail.append(p.x, p.y)
        self.arm_tcp = (p.x, p.y)
        self.arm_tcp_yaw_deg = pose_yaw_deg(getattr(msg.pose, "orientation", None))
        stamp = getattr(msg.header, "stamp", None)
        self.arm_tcp_stamp = (stamp.sec, stamp.nanosec) if stamp else None
        self._mark_rx("arm_tcp")
        self._pair_gripper_sample()

    def _pair_gripper_sample(self):
        if self.arm_tcp_stamp is not None and self.arm_table_stamp == self.arm_tcp_stamp:
            self.arm_gripper_pair = self.arm_table, self.arm_tcp
            self._mark_rx("arm_pair")

    def measured_arm_in_base(self):
        pair = self.arm_gripper_pair
        if (pair and pair[0] is self.arm_table and self.arm_table_origin is not None
                and self.source_age("arm_pair") < ARM_TIMEOUT_SEC):
            x, y, yaw_deg = self.arm_table_origin
            yaw = math.radians(yaw_deg)
            c, s = math.cos(yaw), math.sin(yaw)
            return tuple((c * (px - x) + s * (py - y), -s * (px - x) + c * (py - y))
                         for px, py in (*pair[0][0], pair[1]))
        if self.arm_base and self.source_age("arm_base") < ARM_TIMEOUT_SEC:
            return self.arm_base[0]

    def _cb_ekf(self, msg) -> None:
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

    def _cb_valve(self, msg) -> None:
        self._driver_air_open = msg.main_valve_open
        self.modbus_rtt_ms = msg.modbus_rtt_ms
        self._mark_rx("valve")

    def _cb_valve_output(self, msg) -> None:
        duties = msg.duty_cycles
        self.valve_cmd = (list(duties) if duties is not None and len(duties) == 8
                          and all(0 <= duty <= 1 for duty in duties) else None)
        self._mark_rx("valve_output")

    def fresh_valve_command(self):
        if self.source_age("valve_output") <= SOURCE_TIMEOUT_SEC:
            return self.valve_cmd
        return None

    def _cb_ref(self, msg) -> None:
        timestamp = self._relative_time(msg.header.stamp)
        self.t_r.append(timestamp)
        self.ref_x.append(msg.x_ref)
        self.ref_y.append(msg.y_ref)
        self.ref_th.append(math.degrees(msg.theta_ref))
        self.latest_ref = (msg.x_ref, msg.y_ref, msg.theta_ref)
        self._mark_rx("reference")

    def _cb_plans(self, entries) -> None:
        self.paper_plans = {}
        for key, plan_id, xy, times, *optional in entries:
            yaw = np.asarray(optional[0]) if optional and optional[0] is not None else None
            joints = np.asarray(optional[1]) if len(optional) >= 2 and optional[1] is not None else None
            geometry = optional[2] if len(optional) >= 3 else None
            self.paper_plans[key] = (plan_id, np.asarray(xy), np.asarray(times), yaw, joints, geometry)
        if self.plan_choice not in ("auto", "off") and self.plan_choice not in self.paper_plans:
            self.plan_choice = "auto"

    def _cb_paper_trial(self, status) -> None:
        self.paper_trial, self.paper_elapsed, *self.paper_match = status
        self._mark_rx("paper_trial")

    def _cb_plan_defaults(self, defaults):
        values = defaults if isinstance(defaults, dict) else getattr(defaults, "__dict__", {})
        self.plan_defaults = {method: key for method, key in values.items()
                              if method in ("Nominal", "Proposed", "JF_only") and isinstance(key, str)}

    def _cb_paper_status(self, status):
        data = status if isinstance(status, dict) else getattr(status, "__dict__", {})
        phase = data.get("phase")
        if not isinstance(phase, str) or not phase:
            return
        if phase != self.toss_phase:
            phases = {"IDLE": "대기", "WAITING_FOR_A": "시작 파지점 정렬 대기", "PREPARING": "준비 중",
                      "WAITING_FOR_MANUAL_CLOSE": "수동 파지 대기", "CLOSING_AT_A": "시작점 파지 중",
                      "A_HELD": "시작 파지점 고정",
                      "LAUNCH": "발사 구간", "COAST": "자유 이동", "CAPTURING": "캡처 진입",
                      "CLOSING": "파지 중", "LOCAL_BRAKE": "제동 중", "PLANNED_BRAKE": "계획 제동",
                      "HOLDING": "파지 유지", "STOPPING": "정지 중", "STOPPED": "정지",
                      "MISSED": "캡처 실패", "FAULT": "실험 오류"}
            level = 40 if phase == "FAULT" else 30 if phase == "MISSED" else 20
            reason = data.get("reason") if level >= 30 else None
            message = phases.get(phase, phase)
            self.toss_phase_label = message
            self.append_log(f"Self-tossing · {message}" + (f" · {reason}" if reason else ""), level)
        self.toss_phase = phase
        gripper = data.get("fss_gripper")
        if isinstance(gripper, str) and gripper and gripper != self.toss_gripper:
            self.toss_gripper = gripper
            self.append_log(f"그리퍼 · {gripper}")
        contact = data.get("contact_verified") is True
        if contact and not self.toss_contact:
            self.append_log("B 레일 접촉 확인")
        self.toss_contact = contact
        guide = data.get("grasp_guide")
        aligned = guide == "TCP within launch grasp tolerance"
        self.toss_alignment = ""
        if aligned:
            self.toss_alignment = "시작 파지점 정렬 완료"
            if not self.toss_aligned:
                self.append_log(self.toss_alignment)
        elif guide == "Waiting for fresh TCP pose":
            self.toss_alignment = "파지점 수신 대기"
        elif isinstance(guide, str) and guide.startswith("Move TCP (table):"):
            error = data.get("launch_pose_error")
            if (isinstance(error, (list, tuple)) and len(error) == 3 and
                    all(type(v) in (int, float) and math.isfinite(v) for v in error)):
                x, y, yaw = error
                self.toss_alignment = (f"파지점까지  X {-1000*x:+.1f} mm   Y {-1000*y:+.1f} mm"
                                       f"   Yaw {-math.degrees(yaw):+.1f}°")
        self.toss_aligned = aligned
        self._mark_rx("paper_status")
        directory = data.get("log_directory")
        if (data.get("frame_id") == "table" and data.get("plan_id")
                and directory and directory != self.trial_directory):
            self.trial_directory = directory
            self.plan_samples.clear()
            self.error_run = self.error_plan = ""

    def _cb_tracking_error(self, sample):
        data = sample if isinstance(sample, dict) else vars(sample)
        run, plan = data["run_id"], data["plan_key"]
        if (run, plan) != (self.error_run, self.error_plan):
            self.plan_samples.clear()
            self.error_run, self.error_plan = run, plan
        self.plan_samples.append(data)

    def active_plan_key(self):
        if not self.paper_trial or self.source_age("paper_trial") > SOURCE_TIMEOUT_SEC:
            return None
        if self.paper_match:
            key = self.paper_match[0]
            if key in self.paper_plans and self.paper_plans[key][0] == self.paper_trial:
                return key
            return None
        matches = [key for key, plan in self.paper_plans.items() if plan[0] == self.paper_trial]
        return matches[0] if len(matches) == 1 else None

    def preview_plan_key(self):
        try:
            options = shlex.split(self.toss_options)
        except ValueError:
            return None
        if any(option.split(":=", 1)[0] == "trial_file" for option in options):
            return None
        if self.plan_defaults is not None:
            return self.plan_defaults.get(self.toss_method)
        matches = [key for key in self.paper_plans if key.rsplit("/", 1)[-1] == self.toss_method]
        return matches[0] if len(matches) == 1 else None

    def plan_target(self, *, live_only=False):
        active = self.active_plan_key()
        if live_only and self.target_revisions["paper_trial"] <= self.recording_target_revisions.get("paper_trial", -1):
            active = None
        elapsed = self.paper_elapsed if self.source_age("paper_trial") <= SOURCE_TIMEOUT_SEC else None
        if (self.toss_phase and not (live_only and self.toss_phase in ("STOPPED", "MISSED", "FAULT"))
                and self.toss_phase not in (
                "LAUNCH", "COAST", "CAPTURING", "CLOSING", "LOCAL_BRAKE",
                "PLANNED_BRAKE", "HOLDING", "STOPPING")):
            elapsed = None
        key = active if live_only or self.plan_choice == "auto" else self.plan_choice
        if (not live_only and self.plan_choice == "auto" and self.control_view == "toss"
                and elapsed is None and (active is None or self.toss_phase in ("STOPPED", "MISSED", "FAULT"))):
            key = self.preview_plan_key()
        if key not in self.paper_plans:
            key = None
        following = key is not None and key == active and elapsed is not None
        if key is not None:
            _, xy, times, yaw, joints, geometry = self.paper_plans[key]
            t = elapsed if following else 0.0
            arm = None
            if self.arm_equipped and yaw is not None and joints is not None and geometry is not None:
                angles = [np.interp(t, times, column) for column in joints.T]
                arm = plan_arm_points(angles, *geometry)
            target = (np.interp(t, times, xy[:, 0]), np.interp(t, times, xy[:, 1]),
                    np.interp(t, times, yaw) if yaw is not None else None, arm)
            return key, target, following
        if (self.source_age("reference") < XY_TRAIL_SEC
                and (not live_only or self.target_revisions["reference"]
                     > self.recording_target_revisions.get("reference", -1))):
            # A base-only target preserves the latest measured articulation for comparison.
            arm = self.measured_arm_in_base() if self.arm_equipped else None
            return None, (*self.latest_ref, arm), False
        return None, None, False

    def recording_snapshot(self):
        pose = None
        if self.source_age("vicon") < XY_TRAIL_SEC and self.x_v:
            pose = (self.x_v[-1], self.y_v[-1], math.radians(self.th_v[-1]))
        key, target, _ = self.plan_target(live_only=True)
        snapshot = {"pose": pose, "arm": self.measured_arm_in_base() if self.arm_equipped else None,
                    "ghost": target[:3] if target else None, "ghost_arm": target[3] if target else None,
                    "plan": self.paper_plans[key][1] if key else None}
        for name in ("arm", "ghost_arm", "plan"):
            if snapshot[name] is not None:
                snapshot[name] = np.asarray(snapshot[name]).tolist()
        return snapshot

    def _cb_imu(self, msg) -> None:
        self._imu_count += 1
        self._mark_rx("imu")

    def _cb_mode(self, msg) -> None:
        self.mode_name = msg.mode_name or f"MODE {msg.mode}"
        self._mark_rx("mode")

    def _cb_health(self, msg) -> None:
        self.navigation_ok = msg.navigation_ok
        self.actuators_ok = msg.actuators_ok
        self._health_air_open = msg.main_valve_open
        self.navigation_degraded = msg.navigation_degraded
        self.navigation_age_sec = msg.navigation_age_sec
        self.control_loop_rate_hz = msg.control_loop_rate_hz
        self.modbus_rtt_ms = msg.modbus_rtt_ms
        self.warnings = list(msg.warnings)
        self.errors = list(msg.errors)
        self._mark_rx("health")

    def _cb_log(self, msg) -> None:
        self.append_log(f"{msg.name}: {msg.msg}", msg.level)

    def append_log(self, message, level=20):
        self.recorder.log(LOG_LEVELS.get(level, str(level)), message)
        stamp = time.strftime("%H:%M:%S")
        self.logs.append(LogEntry(stamp, level, message))
        self.log_revision += 1

    def set_command_result(self, message, level=20):
        self.recorder.record("command_result", {"level": level, "message": message})
        self.last_command_msg = message
        self.append_log(message, level)

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
        self._value.setFont(QFont(QApplication.font().family(), 22, QFont.Bold))
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
            "border-radius:18px; }"
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
        self._value.setFont(QFont(QApplication.font().family(), 13, QFont.Bold))
        self._value.setStyleSheet(f"color:{C_TEXT}; border:none;")
        layout.addWidget(name_label)
        layout.addWidget(self._value)
        self.set_state(C_IDLE, "—")

    def set_state(self, color: str, value: str) -> None:
        self._value.setText(value)
        self._value.setStyleSheet(f"color:{color}; border:none;")
        self.setStyleSheet(
            f"QFrame#readyLight {{ background:{C_PANEL}; border:1px solid {C_BORDER}; "
            "border-radius:11px; }"
        )


class AirBearingButton(QPushButton):
    def nextCheckState(self):
        # Only the telemetry update may select a state, including after a service reply.
        pass


class TargetCoordinate(QDoubleSpinBox):
    """Select the previous coordinate on the first click; later clicks move the cursor."""

    def __init__(self):
        super().__init__()
        self._first_click = False
        self.setLocale(QLocale.c())
        self.setKeyboardTracking(False)
        self.lineEdit().installEventFilter(self)

    def focusInEvent(self, event):
        self._first_click = event.reason() == Qt.MouseFocusReason
        super().focusInEvent(event)

    def eventFilter(self, watched, event):
        handled = super().eventFilter(watched, event)
        if (self._first_click and event.type() == QEvent.MouseButtonRelease
                and event.button() == Qt.LeftButton):
            self._first_click = False
            if not self.lineEdit().hasSelectedText():
                self.selectAll()
        return handled


class Dashboard(QMainWindow):
    """FSS 미션 관제용 실시간 대시보드."""

    def __init__(self, node: DashboardNode, fps: int = 30, history_sec: float = 30) -> None:
        app = QApplication.instance()
        if not app.property("fssFontLoaded"):
            # Official desktop variable font from orioncactus/pretendard v1.3.9.
            path = Path(__file__).parent / "assets" / "pretendard" / "PretendardVariable.ttf"
            families = QFontDatabase.applicationFontFamilies(QFontDatabase.addApplicationFont(str(path)))
            if families:
                font = QFont(app.font())
                font.setFamilies(families)
                app.setFont(font)
                app.setProperty("fssFontLoaded", True)
        super().__init__()
        self._node = self._default_node = node
        self._history_sec = history_sec
        self._experiment_root = Path(__file__).resolve().parents[1] / "experiments"
        self._links = {}
        self._recorded_runs = {}
        self._recorders = {node.recorder}
        self._vehicle_settings = QSettings("FSS", "Mission Control")
        self._trails = {}
        self._link_errors = {}
        self._last_rate = time.monotonic()
        self._series_bindings: list[tuple[pg.PlotDataItem, str, str]] = []
        self._log_view_key = None

        self.setWindowTitle("FSS Mission Control")
        self.resize(1200, 760)
        self.setMinimumSize(840, 520)
        self.setStyleSheet(
            f"""
            QWidget {{ color:{C_TEXT}; font-size:14px; }}
            QMainWindow, QDialog {{ background:{C_BG}; }}
            QLabel {{ background:transparent; }}
            QFrame#panel {{ background:{C_PANEL}; border:1px solid {C_BORDER};
                            border-radius:18px; }}
            QTabWidget::pane {{ background:{C_PANEL}; border:1px solid {C_BORDER};
                               border-radius:11px; top:2px; }}
            QTabBar::tab {{ color:{C_TEXT_DIM}; background:transparent; border:1px solid transparent;
                            padding:7px 14px; margin:0px 3px 4px 0px; border-radius:15px; }}
            QTabBar::tab:selected {{ color:{C_TEXT}; background:{C_PANEL}; border-color:{C_BORDER};
                                     font-weight:600; }}
            QTabBar::tab:hover:!selected {{ color:{C_INFO}; }}
            QTabBar#vehicleSelector::tab {{ background:{C_PANEL}; color:{C_TEXT};
                border:1px solid {C_BORDER}; border-radius:15px; margin:0 3px; padding:6px 13px; }}
            QTabBar#vehicleSelector::tab:selected {{ background:{C_INFO}; border-color:{C_INFO};
                color:{C_PANEL}; font-weight:600; }}
            QTabBar#vehicleSelector::tab:hover:!selected {{ border-color:{C_FOCUS}; color:{C_INFO}; }}
            QTextEdit {{ background:{C_PANEL_ALT}; color:{C_TEXT}; border:1px solid {C_BORDER};
                         border-radius:8px; font-size:13px; padding:10px; }}
            QPushButton, QLineEdit, QDoubleSpinBox, QComboBox {{
                color:{C_TEXT}; background:{C_PANEL}; border:1px solid {C_BORDER};
                border-radius:13px; min-height:22px; padding:4px 10px; }}
            QComboBox::drop-down {{ width:22px; border:none; }}
            QPushButton {{ color:{C_INFO}; border-radius:15px; padding:4px 13px; }}
            QPushButton:hover {{ border-color:{C_INFO}; }}
            QPushButton#bodyStart {{ background:{C_PANEL}; border:1px solid {C_BORDER};
                border-radius:10px; min-height:18px; padding:1px 7px; margin-right:6px; font-size:12px; }}
            QPushButton#bodyStart:hover {{ border-color:{C_INFO}; }}
            QPushButton:focus, QLineEdit:focus, QDoubleSpinBox:focus, QComboBox:focus {{
                border:2px solid {C_FOCUS}; }}
            QDoubleSpinBox QLineEdit {{
                background:transparent; border:none; padding:0; min-height:0; }}
            QDoubleSpinBox::up-button, QDoubleSpinBox::down-button {{
                width:0; height:0; border:none; }}
            QPushButton:pressed {{ background:{C_BG}; }}
            QPushButton:checked, QPushButton#sendTarget {{ background:{C_INFO}; border-color:{C_INFO};
                                         color:{C_PANEL}; font-weight:600; }}
            QPushButton#emergencyStop {{ color:{C_ERROR}; border-color:{C_ERROR}; font-weight:600; }}
            QPushButton#emergencyStop:pressed {{ background:{C_ERROR}; color:{C_PANEL}; }}
            QPushButton:disabled, QPushButton#sendTarget:disabled, QPushButton#emergencyStop:disabled {{
                background:{C_PANEL_ALT}; color:{C_IDLE}; border-color:{C_BORDER}; }}
            QPushButton#airOn:checked, QPushButton#airOn:checked:disabled {{
                background:{C_OK}; color:{C_PANEL}; border-color:{C_OK}; font-weight:600; }}
            QPushButton#airOff:checked, QPushButton#airOff:checked:disabled {{
                background:{C_TEXT_DIM}; color:{C_PANEL}; border-color:{C_TEXT_DIM}; font-weight:600; }}
            QAbstractItemView {{ background:{C_PANEL}; color:{C_TEXT};
                                 selection-background-color:{C_INFO}; selection-color:{C_PANEL}; }}
            """
        )

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(17, 12, 17, 17)
        root.setSpacing(8)

        root.addLayout(self._build_connections())
        views = QStackedWidget()
        overview = QWidget()
        panels = QHBoxLayout(overview)
        panels.setContentsMargins(12, 6, 12, 6)
        panels.setSpacing(12)
        panels.addWidget(self._build_trajectory_panel(), stretch=1)
        sidebar = QVBoxLayout()
        sidebar.setSpacing(6)
        view_row = QHBoxLayout()
        self._control_views = QButtonGroup(self)
        for index, label in enumerate(("위치 제어", "Self-tossing")):
            button = QPushButton(label)
            button.setCheckable(True)
            button.setStyleSheet("min-height:18px; padding:2px 10px; border-radius:12px;")
            self._control_views.addButton(button, index)
            view_row.addWidget(button)
        self._control_views.button(0).setChecked(True)
        self._control_views.idClicked.connect(self._select_control_view)
        sidebar.addLayout(view_row)
        self._controls = self._build_control_bar()
        self._controls.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        sidebar.addWidget(self._controls)
        self._toss_controls = self._build_toss_controls()
        sidebar.addWidget(self._toss_controls)
        self._toss_controls.hide()
        sidebar.addWidget(self._build_valve_plot())
        console_row = QHBoxLayout()
        self._console_tabs = QTabBar()
        self._console_tabs.setExpanding(False)
        self._console_tabs.setDrawBase(False)
        self._console_tabs.currentChanged.connect(self._select_console)
        console_row.addWidget(self._console_tabs, stretch=1)
        self._add_shell = QPushButton("+")
        self._add_shell.setToolTip("새 SSH 터미널")
        self._add_shell.setAccessibleName("새 SSH 터미널")
        self._add_shell.clicked.connect(self._new_terminal)
        console_row.addWidget(self._add_shell)
        self._close_shell = QPushButton("셸 닫기")
        self._close_shell.clicked.connect(self._close_terminal)
        console_row.addWidget(self._close_shell)
        sidebar.addLayout(console_row)
        self._console = QStackedWidget()
        self._terminal = TerminalView()
        self._console.addWidget(self._terminal)
        self._console.setMinimumHeight(90)
        sidebar.addWidget(self._console, stretch=1)
        panels.addLayout(sidebar, stretch=1)
        views.addWidget(overview)
        self._tabs = self._build_detail_tabs()
        self._log_panel = QWidget()
        log_layout = QVBoxLayout(self._log_panel)
        log_layout.setContentsMargins(0, 0, 0, 0)
        filter_row = QHBoxLayout()
        filter_row.addWidget(QLabel("최소 레벨"))
        self._log_level = QComboBox()
        self._log_level.setAccessibleName("로그 최소 레벨")
        for level, label in LOG_LEVELS.items():
            self._log_level.addItem(label, level)
        self._log_level.setCurrentIndex(self._log_level.findData(20))
        filter_row.addWidget(self._log_level)
        filter_row.addStretch()
        self._clear_logs = QPushButton("비우기")
        self._clear_logs.setToolTip("선택한 FSS의 화면 로그 비우기")
        self._clear_logs.clicked.connect(self._clear_visible_log)
        filter_row.addWidget(self._clear_logs)
        log_layout.addLayout(filter_row)
        self._log_text = QTextEdit()
        self._log_text.setReadOnly(True)
        log_layout.addWidget(self._log_text)
        self._log_level.currentIndexChanged.connect(self._update_logs)
        self._console.addWidget(self._log_panel)
        self._local_console = "log" if self._default_node.request else 1
        self._bind_console(None)
        diagnostics = QWidget()
        details = QVBoxLayout(diagnostics)
        details.addLayout(self._build_header())
        detail_row = QHBoxLayout()
        detail_row.addWidget(self._build_readiness_panel())
        detail_row.addWidget(self._build_summary_panel())
        details.addLayout(detail_row)
        self._arm_status = self._build_arm_status()
        details.addWidget(self._arm_status)
        details.addWidget(self._build_status_trends())
        self._tabs.addTab(diagnostics, "상태")
        views.addWidget(self._tabs)
        root.addWidget(views)

        footer = QHBoxLayout()
        self._position_summary = QLabel()
        footer.addWidget(self._position_summary)
        footer.addStretch()
        self._arm_notice = QLabel()
        self._arm_notice.setStyleSheet(f"color:{C_TEXT_DIM};")
        self._arm_notice.hide()
        footer.addWidget(self._arm_notice)
        self._experiment_button = QPushButton("기록 시작")
        self._experiment_button.setToolTip("선택한 FSS 데이터 기록")
        self._experiment_button.clicked.connect(self._toggle_experiment)
        footer.addWidget(self._experiment_button)
        detail_button = QPushButton("상세")
        detail_button.setCheckable(True)
        detail_button.toggled.connect(lambda show: views.setCurrentIndex(int(show)))
        detail_button.toggled.connect(lambda show: detail_button.setText("관제로" if show else "상세"))
        footer.addWidget(detail_button)
        root.addLayout(footer)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._update)
        self._timer.start(max(16, round(1000 / max(1, fps))))

    def _bind_console(self, link):
        self._console_link = link
        sessions = link.shells.sessions if link else {1: None}
        selected = link.shells.selected if link else self._local_console
        tabs = self._console_tabs
        tabs.blockSignals(True)
        while tabs.count():
            tabs.removeTab(0)
        for key in [*sessions, "log"]:
            title = "로그" if key == "log" else ("SSH" if key == 1 else f"SSH {key}")
            index = tabs.addTab(title)
            tabs.setTabData(index, key)
            if key == selected:
                tabs.setCurrentIndex(index)
        tabs.blockSignals(False)
        self._select_console(tabs.currentIndex())

    def _select_console(self, index):
        link = self._console_link
        key = self._console_tabs.tabData(index)
        if link:
            link.shells.selected = key
        else:
            self._local_console = key
        self._console.setCurrentWidget(self._log_panel if key == "log" else self._terminal)
        self._terminal.bind(link.shells.sessions.get(key) if link else None)
        self._add_shell.setEnabled(link is not None and not link.stopped.is_set())
        self._close_shell.setVisible(link is not None and isinstance(key, int) and key > 1)

    def _new_terminal(self):
        link = self._console_link
        if link and link.shells.add() is not None:
            self._bind_console(link)
            self._terminal.setFocus()

    def _close_terminal(self):
        link = self._console_link
        if link:
            link.shells.remove(link.shells.selected)
            self._bind_console(link)

    def _build_connections(self):
        row = QHBoxLayout()
        self._vehicles = QTabBar()
        self._vehicles.setObjectName("vehicleSelector")
        self._vehicles.setExpanding(False)
        self._vehicles.setDrawBase(False)
        self._vehicles.setElideMode(Qt.ElideNone)
        self._vehicles.setUsesScrollButtons(True)
        self._disconnect_button = QPushButton("연결 해제")
        self._disconnect_button.hide()
        self._disconnect_button.clicked.connect(lambda: self._disconnect_jetson(self._vehicles.currentIndex()))
        self._vehicles.currentChanged.connect(self._select_vehicle)
        if self._default_node.request:
            self._vehicles.addTab("로컬 ROS")
        add = QPushButton("+ FSS 연결")
        add.clicked.connect(self._connect_dialog)
        row.addWidget(add)
        row.addWidget(self._vehicles)
        row.addWidget(self._disconnect_button)
        row.addStretch()
        self._release_estop = QPushButton("정지 해제")
        self._release_estop.clicked.connect(self._reset_estop)
        self._release_estop.hide()
        row.addWidget(self._release_estop)
        self._btn_estop = QPushButton("비상 정지")
        self._btn_estop.setObjectName("emergencyStop")
        self._btn_estop.clicked.connect(lambda: self._node.request_mode(MODE_ESTOP))
        row.addWidget(self._btn_estop)
        return row

    def _shared_domain(self, link):
        return any(other is not link and not other.stopped.is_set()
                   and (other.observer in link.peers or link.observer in other.peers)
                   for other in self._links.values())

    def _can_start_body(self, endpoint, link):
        return (self._links.get(endpoint) is link and link.ready and not link.stopped.is_set()
                and link.data.control_view == "position"
                and link.profile in ("auto", "control")
                and not link.read_only and not link.bridge_error and link.can_start_body
                and not link.body_start_pending and not link.toss_pending
                and time.monotonic() - link.last_body_status_at <= SOURCE_TIMEOUT_SEC
                and not self._shared_domain(link))

    def _start_vehicle_body(self, endpoint, link):
        for connected in self._links.values():
            connected.drain()
        if self._can_start_body(endpoint, link):
            link.data.send_command("start_body", True)
        self._update()

    def _can_stop_body(self, endpoint, link):
        return (self._links.get(endpoint) is link and link.ready and not link.stopped.is_set()
                and link.profile != "monitor" and not link.bridge_error and link.can_stop_body
                and not link.body_start_pending
                and time.monotonic() - link.last_body_status_at <= SOURCE_TIMEOUT_SEC)

    def _stop_vehicle_body(self, endpoint, link):
        for connected in self._links.values():
            connected.drain()
        if self._can_stop_body(endpoint, link):
            link.data.send_command("stop_body", True)
        self._update()

    def _update_body_button(self, index, endpoint, link, label):
        side = QTabBar.RightSide
        button = self._vehicles.tabButton(index, side)
        stopping = self._can_stop_body(endpoint, link)
        if not stopping and not self._can_start_body(endpoint, link):
            if button is not None:
                button.setEnabled(False)
                self._vehicles.setTabButton(index, side, None)
                button.deleteLater()
            return
        if button is None:
            button = QPushButton("위치제어 시작", self._vehicles)
            button.setObjectName("bodyStart")
            button.adjustSize()
            button.clicked.connect(lambda: self._stop_vehicle_body(endpoint, link)
                                   if self._can_stop_body(endpoint, link) else self._start_vehicle_body(endpoint, link))
            self._vehicles.setTabButton(index, side, button)
        button.setText("FSS 프로세스 종료" if stopping else "위치제어 시작")
        button.adjustSize()
        action = f"{label} FSS 프로세스 종료" if stopping else f"{label} 위치제어 FSW 시작"
        button.setAccessibleName(action)
        button.setToolTip(action)

    def _reset_estop(self):
        if self._node.mode_name == "EMERGENCY_STOP" and self._node.source_age("mode") <= SOURCE_TIMEOUT_SEC:
            self._node.request_mode(MODE_IDLE)

    def _select_control_view(self, index):
        self._node.control_view = "position" if index == 0 else "toss"
        self._update()

    def _read_target(self):
        for editor in (self._target_x, self._target_y, self._target_yaw):
            editor.interpretText()
        return (self._controller.checkedButton().text(), self._target_x.value(),
                self._target_y.value(), self._target_yaw.value())

    def _send_target(self):
        if self._node.request and self._send_target_button.isEnabled():
            controller, x, y, yaw = self._read_target()
            self._node.send_command("target", [controller.lower(), x, y, math.radians(yaw)])

    def _selected_can_record(self):
        endpoint = self._vehicles.tabData(self._vehicles.currentIndex())
        link = self._links.get(endpoint)
        return link.ready and not link.stopped.is_set() if link else self._default_node.request is not None

    def _toggle_experiment(self):
        node = self._node
        endpoint = self._vehicles.tabData(self._vehicles.currentIndex())
        link = self._links.get(endpoint)
        if node.recorder.active:
            if link:
                link.drain()
            self._stop_experiment(node, "operator")
        elif self._selected_can_record():
            last = self._vehicle_settings.value("experiments/last_name", "default", type=str)
            name, accepted = QInputDialog.getText(self, "기록 시작", "실험 이름", text=last)
            if not accepted or node is not self._node:
                return
            if link:
                link.drain()
            if not self._selected_can_record():
                return
            vehicle = (link.display_id or endpoint) if link else "local"
            node.source_run = None
            node.recording_target_revisions = {}
            try:
                directory = node.recorder.start(self._experiment_root, name.strip(), vehicle)
            except (OSError, ValueError) as exc:
                QMessageBox.warning(self, "실험 기록", str(exc))
                return
            self._vehicle_settings.setValue("experiments/last_name", name.strip())
            node.reported_recording_error = ""
            node.reported_video_error = ""
            node.append_log(f"실험 기록 시작 · {directory}")
        self._update()

    @staticmethod
    def _stop_experiment(node, reason):
        if node.recorder.active:
            node.recorder.capture(node.recording_snapshot())
            try:
                directory = node.recorder.stop(reason)
            except OSError as exc:
                node.reported_recording_error = node.recorder.error
                node.set_command_result(f"실험 저장 실패 · {exc}", 40)
            else:
                node.append_log(f"실험 저장 · {directory}")
        node.auto_recording_run = None
        node.auto_recording_deadline = None

    def _finish_planned_recording(self, node):
        if not node.recorder.active or not node.auto_recording_run:
            return
        if node.auto_recording_deadline is None:
            if node.toss_phase not in ("LAUNCH", "COAST", "CAPTURING", "CLOSING", "LOCAL_BRAKE",
                                       "PLANNED_BRAKE", "HOLDING", "STOPPING"):
                return
            key = node.active_plan_key()
            elapsed = node.paper_elapsed
            if (key is None or node.paper_trial != node.auto_recording_run[0]
                    or node.target_revisions["paper_trial"] <= node.recording_target_revisions.get("paper_trial", -1)
                    or type(elapsed) not in (int, float) or not math.isfinite(elapsed) or elapsed < 0):
                return
            times = node.paper_plans[key][2]
            if not len(times) or not math.isfinite(times[-1]):
                return
            # Anchor to the running plan clock, not the potentially long preparation.
            node.auto_recording_deadline = time.monotonic() + max(0., float(times[-1]) + 10. - elapsed)
        if time.monotonic() >= node.auto_recording_deadline:
            self._stop_experiment(node, "plan_finished_plus_10s")

    def _auto_record_status(self, endpoint, link, message, after):
        data = message if isinstance(message, dict) else getattr(message, "__dict__", {})
        phase, plan, directory = (data.get(name) for name in ("phase", "plan_id", "log_directory"))
        if (data.get("frame_id") != "table" or not all(isinstance(value, str) and value
                                                     for value in (phase, plan, directory))):
            return
        node = link.data
        key = plan, directory
        if after:
            if node.auto_recording_run == key:
                node.source_run["phase"] = phase
                node.recorder.capture(node.recording_snapshot())
            if node.auto_recording_run == key and phase in ("STOPPED", "MISSED", "FAULT"):
                self._stop_experiment(node, phase.lower())
            return
        if phase not in ("PREPARING", "LAUNCH", "COAST", "CAPTURING", "CLOSING",
                         "LOCAL_BRAKE", "PLANNED_BRAKE", "HOLDING"):
            return
        handled = self._recorded_runs.setdefault(endpoint, set())
        if key in handled:
            return
        handled.add(key)
        if node.auto_recording_run:
            self._stop_experiment(node, "next_run")
        if node.recorder.active:
            return
        node.source_run = {"plan_id": plan, "log_directory": directory,
                           "phase": phase, "partial_join": phase != "PREPARING"}
        node.recording_target_revisions = dict(node.target_revisions)
        try:
            saved = node.recorder.start(self._experiment_root, "self-tossing", link.display_id or endpoint)
        except (OSError, ValueError) as exc:
            node.set_command_result(f"실험 자동 기록 실패 · {exc}", 40)
            return
        node.auto_recording_run = key
        node.auto_recording_deadline = None
        node.reported_recording_error = ""
        node.reported_video_error = ""
        node.append_log(f"실험 자동 기록 시작 · {saved}")

    @staticmethod
    def _report_recording_error(node):
        if node.recorder.error and node.reported_recording_error != node.recorder.error:
            node.reported_recording_error = node.recorder.error
            node.set_command_result(f"실험 저장 실패 · {node.recorder.error}", 40)
        if node.recorder.video_error and node.reported_video_error != node.recorder.video_error:
            node.reported_video_error = node.recorder.video_error
            node.append_log(f"실험 영상 저장 실패 · {node.recorder.video_error}", 40)

    def _connect_dialog(self):
        dialog = QDialog(self)
        dialog.setWindowTitle("FSS 연결")
        form = QFormLayout(dialog)
        host = QLineEdit("192.168.0.9")
        display_id = QLineEdit()
        display_id.setPlaceholderText("예: FSS-1")
        display_id.setMaxLength(32)
        arm = QCheckBox("로봇팔 장착")
        monitor = QCheckBox("모니터링만")
        monitor.setToolTip("선택한 기체의 데이터를 확인합니다.")

        def load_profile(address):
            endpoint = address.strip()
            display_id.setText(self._vehicle_settings.value(f"ids/{endpoint}", "", type=str))
            arm.setChecked(self._vehicle_settings.value(f"arm_equipped/{endpoint}", True, type=bool))

        host.textEdited.connect(load_profile)
        load_profile(host.text())
        user = QLineEdit("smrl")
        password = QLineEdit()
        password.setEchoMode(QLineEdit.Password)
        for label, field in (("SSH 주소", host), ("FSS ID (선택)", display_id),
                             ("SSH 계정", user), ("암호", password)):
            form.addRow(label, field)
        form.addRow("", arm)
        form.addRow("", monitor)
        buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.button(QDialogButtonBox.Ok).setText("연결")
        buttons.button(QDialogButtonBox.Cancel).setText("취소")
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        password.setFocus()
        if dialog.exec() and host.text().strip() and user.text().strip():
            self.add_jetson(host.text().strip(), user.text().strip(), None, password.text(),
                            display_id.text(), arm_equipped=arm.isChecked(), monitor_only=monitor.isChecked(),
                            prompt_existing=not monitor.isChecked())

    def _choose_existing_processes(self, endpoint, link):
        link.drain()
        if not self._can_stop_body(endpoint, link):
            return
        dialog = QMessageBox(self)
        dialog.setWindowTitle("FSS 연결")
        dialog.setText(f"{link.display_id or endpoint}에 실행 중인 FSS 프로세스가 있습니다.")
        keep = dialog.addButton("기존 프로세스에 연결", QMessageBox.AcceptRole)
        stop = dialog.addButton("종료 후 연결", QMessageBox.DestructiveRole)
        dialog.setDefaultButton(keep)
        dialog.exec()
        if dialog.clickedButton() is stop:
            self._stop_vehicle_body(endpoint, link)

    def add_jetson(self, host, user, domain, password, display_id=None, monitor_only=False, arm_equipped=None,
                   profile=None, prompt_existing=False):
        endpoint = host if domain is None else f"{host} / {domain}"
        profile = "monitor" if monitor_only else (profile or "auto")
        arm_key = f"arm_equipped/{endpoint}"
        if arm_equipped is None:
            arm_equipped = self._vehicle_settings.value(arm_key, True, type=bool)
        else:
            self._vehicle_settings.setValue(arm_key, arm_equipped)
        key = f"ids/{endpoint}"
        if display_id is None:
            display_id = self._vehicle_settings.value(key, "", type=str)
        else:
            display_id = display_id.strip()
            if display_id:
                self._vehicle_settings.setValue(key, display_id)
            else:
                self._vehicle_settings.remove(key)
        for i in range(self._vehicles.count()):
            if self._vehicles.tabData(i) == endpoint:
                self._vehicles.setCurrentIndex(i)
                existing = self._links[endpoint]
                profile_changed = existing.display_id != display_id or existing.data.arm_equipped != arm_equipped
                existing.display_id = display_id
                existing.data.arm_equipped = arm_equipped
                shells_finished = all(shell.finished for shell in existing.shells.sessions.values())
                if (existing.ssh_user == user and existing.profile == profile
                        and endpoint not in self._link_errors
                        and (profile_changed or not (existing.bridge_error or shells_finished
                                             or (existing.read_only and profile != "monitor")))):
                    self._update()
                    return
                self._disconnect_jetson(i)
                break
        link = JetsonLink(DashboardNode(self._history_sec), profile=profile)
        link.prompt_existing = prompt_existing
        self._recorders.add(link.data.recorder)
        if profile == "toss":
            link.shells.selected = "log"
        link.ssh_user = user
        link.display_id = display_id
        link.data.arm_equipped = arm_equipped
        link.data.append_log(f"{host} 연결 중")
        self._links[endpoint] = link
        link.data.request = link.request
        link.data.recording_status = lambda message, after: self._auto_record_status(
            endpoint, link, message, after)
        link.trust.connect(self._confirm_key)
        link.changed.connect(self._connection_error)
        index = self._vehicles.addTab(endpoint)
        self._vehicles.setTabData(index, endpoint)
        self._vehicles.setCurrentIndex(index)
        self._select_vehicle(index)
        colors = [C_EKF, C_WARN, C_VICON, "#7048a2", "#a7376d"]
        color = colors[(len(self._links) - 1) % len(colors)]
        trail = self._xy_plot.plot(pen=pg.mkPen(color, width=2))
        marker = VehicleBubble()
        self._xy_plot.addItem(marker)
        self._trails[endpoint] = trail, marker
        link.start(host, user, domain, password)

    def _confirm_key(self, link, host, fingerprint):
        if not link.stopped.is_set():
            link.accepted = QMessageBox.question(
                self, "처음 연결하는 FSS", f"{host}의 SSH 키를 신뢰할까요?\nSHA256:{fingerprint}"
            ) == QMessageBox.Yes
        link.decision.set()

    def _connection_error(self, message):
        link = self.sender()
        for endpoint, candidate in self._links.items():
            if candidate is link:
                self._link_errors[endpoint] = message
                link.drain()
                self._stop_experiment(link.data, "connection_error")
                link.close()
                for shell in link.shells.sessions.values():
                    if not shell.finished:
                        shell.status = f"SSH: {message}"
                if self._console_link is link:
                    self._add_shell.setEnabled(False)
                link.data.set_command_result(message, 40)
                break

    def _select_vehicle(self, index):
        link = self._links.get(self._vehicles.tabData(index)) if index >= 0 else None
        self._disconnect_button.setVisible(link is not None)
        chosen = link.data if link else self._default_node
        changed = chosen is not self._node
        if changed and hasattr(self, "_target_x"):
            self._node.target_draft = self._read_target()
            self._node.toss_options = self._toss_options.text()
        self._node = chosen
        if hasattr(self, "_log_panel"):
            self._bind_console(link)
        if changed and hasattr(self, "_target_x"):
            controller, x, y, yaw = self._node.target_draft
            self._control_views.button(1 if self._node.control_view == "toss" else 0).setChecked(True)
            self._controller.button(0 if controller == "PID" else 1).setChecked(True)
            self._toss_method.blockSignals(True)
            self._toss_method.setCurrentText(self._node.toss_method)
            self._toss_method.blockSignals(False)
            self._toss_options.setText(self._node.toss_options)
            for key, field in self._toss_tuning.items():
                field.setText(self._node.toss_tuning_draft.get(key, ""))
            for field, value in ((self._target_x, x), (self._target_y, y), (self._target_yaw, yaw)):
                field.setValue(value)
        self._log_view_key = None
        if hasattr(self, "_command_status"):
            self._log_text.clear()
            self._command_status.setText(self._node.last_command_msg)
            for curve in (self._vicon_curve, self._position_dot):
                curve.setData([], [])
            for curve, _, _ in self._series_bindings:
                curve.setData([], [])
            self._update()

    def _disconnect_jetson(self, index):
        endpoint = self._vehicles.tabData(index)
        link = self._links.pop(endpoint, None)
        if link:
            link.drain()
            self._stop_experiment(link.data, "disconnect")
            link.close()
            self._link_errors.pop(endpoint, None)
            for item in self._trails.pop(endpoint):
                self._xy_plot.removeItem(item)
            start = self._vehicles.tabButton(index, QTabBar.RightSide)
            self._vehicles.removeTab(index)
            if start is not None:
                start.deleteLater()
            self._select_vehicle(self._vehicles.currentIndex())

    def closeEvent(self, event):
        self._timer.stop()
        self._stop_experiment(self._default_node, "window_closed")
        for link in self._links.values():
            link.drain()
            self._stop_experiment(link.data, "window_closed")
            link.close()
        for recorder in self._recorders:
            recorder.finish_video()
        super().closeEvent(event)

    def _build_header(self) -> QHBoxLayout:
        layout = QHBoxLayout()
        layout.setSpacing(8)

        self._card_mode = StatusCard("Mode")
        self._card_nav = StatusCard("Navigation")
        self._card_act = StatusCard("Actuators")
        self._card_main = StatusCard("에어베어링")
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
        bar = QFrame()
        bar.setObjectName("panel")
        layout = QVBoxLayout(bar)
        layout.setSizeConstraint(QLayout.SetMinimumSize)
        layout.setContentsMargins(12, 9, 12, 10)
        layout.setSpacing(6)
        bar.setStyleSheet("QPushButton {border-radius:10px; min-height:22px; padding:3px 9px;}"
                          "QDoubleSpinBox {border-radius:10px; min-height:22px; padding:3px 6px;}")
        commands = QHBoxLayout()
        commands.setSpacing(6)
        commands.addWidget(QLabel("제어기"))
        self._controller = QButtonGroup(bar)
        for index, label in enumerate(("PID", "LQR")):
            button = QPushButton(label)
            button.setCheckable(True)
            self._controller.addButton(button, index)
            button.setChecked(index == 0)
            commands.addWidget(button)
        commands.addStretch()
        commands.addWidget(QLabel("에어베어링"))
        self._btn_air_on = AirBearingButton("ON")
        self._btn_air_off = AirBearingButton("OFF")
        self._btn_air_on.setObjectName("airOn")
        self._btn_air_off.setObjectName("airOff")
        self._btn_air_on.setCheckable(True)
        self._btn_air_off.setCheckable(True)
        self._btn_air_on.clicked.connect(lambda: self._node.request_main_valve(True))
        self._btn_air_off.clicked.connect(lambda: self._node.request_main_valve(False))
        commands.addWidget(self._btn_air_on)
        commands.addWidget(self._btn_air_off)
        layout.addLayout(commands)

        targets = QGridLayout()
        targets.setVerticalSpacing(2)
        targets.setHorizontalSpacing(6)
        self._target_x, self._target_y, self._target_yaw = (TargetCoordinate() for _ in range(3))
        for col, (field, label, low, high, decimals, step) in enumerate((
            (self._target_x, "X (m)", 0, 3, 3, 0.01),
            (self._target_y, "Y (m)", 0, 4, 3, 0.01),
            (self._target_yaw, "Yaw (°)", -180, 180, 1, 1),
        )):
            field.setRange(low, high)
            field.setDecimals(decimals)
            field.setSingleStep(step)
            field.lineEdit().setTextMargins(2, 0, 2, 0)
            field.setAccessibleName(label)
            caption = QLabel(label)
            caption.setBuddy(field)
            targets.addWidget(caption, 0, col)
            targets.addWidget(field, 1, col)
            targets.setColumnStretch(col, 1)
        self._send_target_button = QPushButton("목표 전송")
        self._send_target_button.setObjectName("sendTarget")
        self._send_target_button.clicked.connect(self._send_target)
        targets.addWidget(self._send_target_button, 1, 3)
        layout.addLayout(targets)
        self._command_status = QLabel("")
        self._command_status.setWordWrap(True)
        self._command_status.hide()
        layout.addWidget(self._command_status)
        return bar

    def _build_toss_controls(self):
        panel = QFrame()
        panel.setObjectName("panel")
        panel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        layout = QGridLayout(panel)
        layout.setContentsMargins(12, 9, 12, 10)
        layout.setSpacing(5)
        layout.addWidget(QLabel("경로"), 0, 0)
        self._toss_status = QLabel()
        self._toss_status.setTextFormat(Qt.PlainText)
        self._toss_status.setStyleSheet(f"color:{C_TEXT_DIM};")
        self._toss_status.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Preferred)
        self._toss_status.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
        layout.addWidget(self._toss_status, 0, 1, 1, 3)
        self._toss_method = QComboBox()
        self._toss_method.setAccessibleName("Self-tossing 경로")
        self._toss_method.addItems(("Nominal", "JF_only", "Proposed"))
        self._toss_method.currentTextChanged.connect(self._choose_toss_method)
        layout.addWidget(self._toss_method, 1, 0, 1, 2)
        self._toss_buttons = {}
        for col, action in enumerate(("toss_fss", "toss_run", "toss_close", "toss_start", "toss_stop")):
            button = QPushButton(action)
            button.setAccessibleName(action)
            if action == "toss_start":
                button.setObjectName("sendTarget")
            button.clicked.connect(lambda checked=False, name=action: self._send_toss(name))
            layout.addWidget(button, 2, col)
            self._toss_buttons[action] = button
        self._toss_options = QLineEdit()
        self._toss_options.setAccessibleName("toss_run 추가 옵션")
        self._toss_options.setPlaceholderText("추가 옵션 (선택)   이름:=값")
        self._toss_options.textChanged.connect(self._choose_toss_options)
        layout.addWidget(self._toss_options, 1, 2, 1, 3)
        self._toss_advanced = QPushButton("고급")
        self._toss_advanced.setCheckable(True)
        self._toss_advanced.setAccessibleName("Self-tossing 고급 설정")
        layout.addWidget(self._toss_advanced, 0, 4)
        self._toss_tuning_panel = QWidget()
        tuning = QHBoxLayout(self._toss_tuning_panel)
        tuning.setContentsMargins(0, 0, 0, 0)
        self._toss_tuning_panel.hide()
        self._toss_advanced.toggled.connect(self._toss_tuning_panel.setVisible)
        self._toss_tuning = {}
        for label, key, unit in (("change", "launch_position_gain_change_s", "s"),
                                 ("before pos", "launch_position_gain_before_per_s", "1/s")):
            tuning.addWidget(QLabel(label))
            field = QLineEdit()
            field.setAccessibleName(label)
            field.setPlaceholderText("YAML")
            field.setToolTip(f"{key} ({unit}) · 비우면 tuning.yaml 값 사용")
            field.textChanged.connect(lambda text, k=key: self._node.toss_tuning_draft.update({k: text}))
            self._toss_tuning[key] = field
            tuning.addWidget(field)
            tuning.addWidget(QLabel(unit))
        layout.addWidget(self._toss_tuning_panel, 4, 0, 1, 5)
        self._toss_gripper_position = QLabel("Gripper —")
        self._toss_gripper_position.setTextFormat(Qt.PlainText)
        layout.addWidget(self._toss_gripper_position, 3, 0, 1, 2)
        air = QHBoxLayout()
        air.addStretch()
        air.addWidget(QLabel("에어베어링"))
        self._toss_air_on = AirBearingButton("ON")
        self._toss_air_off = AirBearingButton("OFF")
        for button, value in ((self._toss_air_on, True), (self._toss_air_off, False)):
            button.setObjectName("airOn" if value else "airOff")
            button.setCheckable(True)
            button.clicked.connect(lambda checked=False, state=value: self._send_toss_air(state))
            air.addWidget(button)
        self._toss_standby = QPushButton("실험 대기")
        self._toss_standby.clicked.connect(lambda: self._send_toss("toss_reset"))
        layout.addWidget(self._toss_standby, 3, 2)
        layout.addLayout(air, 3, 3, 1, 2)
        self._toss_alignment = QLabel()
        self._toss_alignment.setTextFormat(Qt.PlainText)
        self._toss_alignment.setStyleSheet(f"color:{C_TEXT_DIM};")
        self._toss_alignment.hide()
        layout.addWidget(self._toss_alignment, 5, 0, 1, 5)
        return panel

    def _can_send_toss(self, link, action):
        return (self._node.control_view == "toss" and link and link.data is self._node
                and not self._shared_domain(link) and link.can_toss(action))

    def _send_toss_air(self, state):
        link = self._links.get(self._vehicles.tabData(self._vehicles.currentIndex()))
        if link:
            link.drain()
        if (self._node.control_view == "toss" and link and link.data is self._node
                and not self._shared_domain(link) and link.can_toss_air()):
            self._node.request_main_valve(state)

    def _choose_toss_method(self, method):
        self._node.toss_method = method
        self._node.plan_choice = "auto"
        self._plan_picker.setCurrentIndex(self._plan_picker.findData("auto"))
        self._update_trajectory()

    def _choose_toss_options(self, text):
        self._node.toss_options = text
        self._update_trajectory()

    def _send_toss(self, action):
        endpoint = self._vehicles.tabData(self._vehicles.currentIndex())
        link = self._links.get(endpoint)
        for connected in self._links.values():
            connected.drain()
        if self._can_send_toss(link, action):
            value = self._toss_method.currentText() if action == "toss_run" else True
            if action == "toss_run":
                try:
                    value = [value, *shlex.split(self._toss_options.text())]
                    for key, field in self._toss_tuning.items():
                        if field.text().strip():
                            number = float(field.text())
                            if not math.isfinite(number) or number < 0:
                                raise ValueError(f"{field.accessibleName()}는 0 이상의 유한한 값이어야 합니다.")
                            value.append(f"{key}:={number}")
                    toss_run_args(value)
                    if len(value) == 1:
                        value = value[0]
                except ValueError as exc:
                    message = f"추가 옵션 확인 · {exc}"
                    link.data.set_command_result(message, 30)
                    self._toss_options.setFocus()
                    QToolTip.showText(self._toss_options.mapToGlobal(self._toss_options.rect().bottomLeft()),
                                      message, self._toss_options)
                    return
            link.data.send_command(action, value)
        self._update()

    def _live_toss_alignment(self, link):
        node = self._node
        if (node.toss_alignment and node.source_age("paper_status") <= SOURCE_TIMEOUT_SEC
                and node.toss_phase in ("IDLE", "WAITING_FOR_A", "STOPPED", "MISSED")
                and (link is None or (link.ready and not link.stopped.is_set() and not link.bridge_error))):
            return node.toss_alignment
        return ""

    def _update_toss_controls(self, link):
        self._toss_standby.setEnabled(bool(self._can_send_toss(link, "toss_reset")))
        for action, button in self._toss_buttons.items():
            button.setEnabled(bool(self._can_send_toss(link, action)))
        active = self._node.active_plan_key()
        bound = bool(active and self._node.toss_phase not in ("STOPPED", "MISSED", "FAULT"))
        method = active.rsplit("/", 1)[-1] if bound else self._node.toss_method
        self._toss_method.blockSignals(True)
        self._toss_method.setCurrentText(method)
        self._toss_method.blockSignals(False)
        self._toss_method.setEnabled(not bound)
        self._toss_method.setToolTip("현재 실험의 경로" if bound else "실험 경로 미리보기")
        phase_fresh = bool(self._node.toss_phase) and self._node.source_age("paper_status") <= SOURCE_TIMEOUT_SEC
        self._toss_method.setVisible(True)
        self._toss_options.setEnabled(not bound)
        for field in self._toss_tuning.values():
            field.setEnabled(not bound)
        gripper = "Gripper —"
        if (self._node.arm_equipped and self._node.arm_tcp is not None
                and self._node.source_age("arm_tcp") < ARM_TIMEOUT_SEC):
            x, y = self._node.arm_tcp
            gripper = f"Gripper   X {x:.3f} m   Y {y:.3f} m"
        self._toss_gripper_position.setText(gripper)
        if not link or not link.ready or link.stopped.is_set() or link.bridge_error:
            status = "연결 대기"
        elif link.read_only:
            status = link.read_only
        elif (link.workflow != "toss"
              and time.monotonic() - link.last_body_status_at <= SOURCE_TIMEOUT_SEC):
            status = link.toss_state or "다른 본체에서 사용 중"
        elif phase_fresh:
            plan = self._node.active_plan_key()
            method = plan.rsplit("/", 1)[-1] if plan else ""
            phase = self._node.toss_phase_label
            status = f"{method} · {phase}" if method else phase
            if self._node.mode_name == "SAFE_HOLD" and self._node.source_age("mode") <= SOURCE_TIMEOUT_SEC:
                status = "SAFE_HOLD"
                if self._node.source_age("vicon") > SOURCE_TIMEOUT_SEC:
                    status += " · Vicon 미수신"
        elif time.monotonic() - link.last_body_status_at <= SOURCE_TIMEOUT_SEC:
            status = link.toss_state or "상태 미수신"
        else:
            status = "상태 미수신"
        self._toss_status.setText(status)
        self._toss_status.setToolTip((link.read_only or link.toss_state or status) if link else status)
        run = self._toss_buttons["toss_run"]
        reason = status
        if link and "toss_fss" in link.toss_actions:
            reason = "toss_fss 실행 후 사용 가능"
        elif link and any(action in link.toss_actions for action in ("toss_close", "toss_start", "toss_stop")):
            reason = "실험 실행기가 이미 켜져 있습니다."
        run.setToolTip("" if run.isEnabled() else reason)
        self._toss_buttons["toss_close"].setToolTip("수동 파지 요청 후 자세 준비·출발까지 진행")
        self._toss_buttons["toss_start"].setToolTip(
            "오류 복구 확인 후 파지·자세 준비부터 재시작" if self._node.toss_phase == "FAULT" else "")
        alignment = self._live_toss_alignment(link)
        self._toss_alignment.setText(alignment)
        self._toss_alignment.setVisible(bool(alignment))

    def _build_valve_plot(self):
        panel = QFrame()
        panel.setObjectName("panel")
        panel.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        layout = QVBoxLayout(panel)
        layout.setSizeConstraint(QLayout.SetMinimumSize)
        layout.setContentsMargins(12, 8, 12, 7)
        layout.setSpacing(2)
        self._valve_title = QLabel("밸브 명령 · 미수신")
        self._valve_title.setToolTip("밸브 드라이버가 전송한 채널별 명령 (0–1)")
        layout.addWidget(self._valve_title)
        plot = pg.PlotWidget()
        plot.setFixedHeight(max(88, 5 * self.fontMetrics().height()))
        plot.setXRange(0.5, 8.5, padding=0)
        plot.setYRange(0, 1, padding=0)
        plot.setMouseEnabled(x=False, y=False)
        plot.hideButtons()
        _style_plot(plot, "", grid_x=False, grid_y=False)
        plot.setTitle(None)
        plot.plotItem.layout.setContentsMargins(0, self.fontMetrics().height() / 2 + 1, 1, 1)
        plot.getAxis("bottom").setHeight(self.fontMetrics().height() + 8)
        plot.getAxis("left").setStyle(hideOverlappingLabels=False)
        for value in (0.5, 1):
            plot.addLine(y=value, pen=pg.mkPen(C_BORDER))
        plot.getAxis("left").setTicks([[(0, "0"), (0.5, "0.5"), (1, "1")]])
        channels = np.arange(1, 9)
        plot.getAxis("bottom").setTicks([[(int(ch), str(ch)) for ch in channels]])
        self._valve_cmd_bar = pg.BarGraphItem(x=channels, height=np.zeros(8), width=0.65, brush=C_INFO, pen=pg.mkPen(None))
        plot.addItem(self._valve_cmd_bar)
        self._valve_cmd_bar.hide()
        layout.addWidget(plot)
        return panel

    def _build_readiness_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(14, 10, 14, 10)
        layout.setSpacing(8)
        title = QLabel("서브시스템 준비도")
        title.setFont(QFont(QApplication.font().family(), 14, QFont.Bold))
        title.setStyleSheet(f"color:{C_TEXT}; border:none;")
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

        self._xy_plot = TablePlot()
        self._plan_picker = QComboBox()
        self._plan_picker.setAccessibleName("실험 기준 경로")
        self._plan_picker.setSizeAdjustPolicy(QComboBox.AdjustToMinimumContentsLengthWithIcon)
        self._plan_picker.setMinimumContentsLength(14)
        self._plan_picker.activated.connect(self._choose_plan)
        self._plan_picker.hide()
        layout.addWidget(self._plan_picker)
        self._picker_catalog = None
        self._drawn_plan = None
        _style_plot(
            self._xy_plot,
            "",
            x_label="X [m]",
            y_label="Y [m]",
            grid_x=False,
            grid_y=False,
        )
        grid_pen = pg.mkPen(C_BORDER)
        for x in (0.5, 1, 1.5, 2, 2.5):
            self._xy_plot.addLine(x=x, pen=grid_pen).setZValue(-2)
        for y in (0.5, 1, 1.5, 2, 2.5, 3, 3.5):
            self._xy_plot.addLine(y=y, pen=grid_pen).setZValue(-2)
        self._table_perimeter = pg.QtWidgets.QGraphicsRectItem(0, 0, 3, 4)
        self._table_perimeter.setPen(pg.mkPen(C_TEXT_DIM, width=8))
        self._table_perimeter.setZValue(-1)
        self._xy_plot.addItem(self._table_perimeter, ignoreBounds=True)
        self._xy_plot.setTitle(None)
        self._plan_curve = self._xy_plot.plot(pen=pg.mkPen("#b9b9c0", width=1.5, style=Qt.DashLine))

        self._vicon_curve = self._xy_plot.plot(
            pen=pg.mkPen(C_VICON, width=1.5),
            symbol="o",
            symbolSize=3,
            symbolBrush=C_VICON,
            symbolPen=None,
        )
        half = BODY_HALF_W
        self._ghost_outline = QPainterPath()
        self._ghost_outline.addRect(QRectF(-half, -half, 2 * half, 2 * half))
        self._ghost_ring = QPainterPath()
        self._ghost_ring.addEllipse(QRectF(-.04, -.04, .08, .08))
        self._ghost = pg.QtWidgets.QGraphicsPathItem()
        self._ghost.setPen(pg.mkPen(QColor(175, 111, 47, 126), width=1.5, style=Qt.DashLine))
        self._ghost.setBrush(QColor(232, 147, 56, 18))
        self._ghost_arm = pg.QtWidgets.QGraphicsPathItem(self._ghost)
        self._ghost_arm.setPen(pg.mkPen(QColor(175, 111, 47, 146), width=2))
        self._ghost.setZValue(1)
        self._ghost.hide()
        self._xy_plot.addItem(self._ghost, ignoreBounds=True)

        self._spacecraft_body = pg.QtWidgets.QGraphicsRectItem(
            -BODY_HALF_W, -BODY_HALF_W, BODY_HALF_W * 2, BODY_HALF_W * 2
        )
        self._spacecraft_body.setPen(pg.mkPen(C_TEXT_DIM, width=1.5))
        self._spacecraft_body.setBrush(QBrush(QColor(29, 29, 31, 45)))
        self._xy_plot.addItem(self._spacecraft_body)

        self._spacecraft_front = pg.QtWidgets.QGraphicsLineItem(0, 0, 0, 0)
        self._spacecraft_front.setPen(pg.mkPen(C_TEXT, width=3))
        self._xy_plot.addItem(self._spacecraft_front)
        self._position_dot = self._xy_plot.plot(
            pen=None,
            symbol="o",
            symbolSize=10,
            symbolBrush=C_VICON,
            symbolPen=pg.mkPen("#ffffff", width=1),
        )

        self._arm_links = self._arm_curve(self._xy_plot)
        self._gripper_link = self._xy_plot.plot(pen=pg.mkPen(C_ARM, width=3))
        self._arm_trace = self._xy_plot.plot(pen=pg.mkPen(C_ARM, width=2, style=Qt.DashLine), connect="finite")
        self._arm_tip = self._xy_plot.plot(pen=None, symbol="d", symbolSize=8, symbolBrush=C_ARM)
        self._local_bubble = VehicleBubble()
        self._xy_plot.addItem(self._local_bubble, ignoreBounds=True)
        layout.addWidget(self._xy_plot)
        return panel

    @staticmethod
    def _arm_curve(plot):
        return plot.plot(pen=pg.mkPen(C_ARM, width=3), symbol="o", symbolSize=6,
                         symbolBrush=C_ARM, symbolPen=pg.mkPen("#ffffff"))

    def _build_arm_status(self):
        panel = QFrame()
        panel.setObjectName("panel")
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(12, 8, 12, 8)
        self._arm_readout = QLabel("로봇팔 · 관절 데이터 대기")
        self._tcp_readout = QLabel()
        self._tcp_readout.setToolTip("기체가 /arm/end_effector_pose에 발행한 모델 파지점")
        self._arm_alignment = QLabel()
        self._arm_alignment.setTextFormat(Qt.PlainText)
        self._arm_alignment.hide()
        for label in (self._arm_readout, self._tcp_readout, self._arm_alignment):
            label.setWordWrap(True)
            layout.addWidget(label)
        return panel

    @staticmethod
    def _draw_arm(curve, shape, age):
        xs, ys = zip(*shape[0]) if shape else ([], [])
        color = C_IDLE if age > ARM_TIMEOUT_SEC else C_ARM
        curve.setData(xs, ys, pen=pg.mkPen(color, width=3), symbolBrush=color)

    def _update_arm(self, *, allow_overlay=True):
        node = self._node
        if node is self._default_node:
            self._local_bubble.show_vehicle(node)
        else:
            self._local_bubble.hide()
        age_table = node.source_age("arm_table")
        age_tcp = node.source_age("arm_tcp")
        age_joints = node.source_age("arm_joints")
        age_base = node.source_age("arm_base")
        joints = node.arm_joints if age_joints < XY_TRAIL_SEC else None
        table = node.arm_table if age_table < XY_TRAIL_SEC else None
        base = node.arm_base if age_base < XY_TRAIL_SEC else None
        tip = node.arm_tcp if age_tcp < XY_TRAIL_SEC else None
        shown = node.arm_equipped and bool(self._links or node.request or joints or base or table or tip)
        self._arm_status.setVisible(shown)
        trace = node.arm_tcp_trail.coordinates() if shown else ([], [])
        self._arm_trace.setData(*trace)
        if not shown:
            tip = table = joints = base = None
        paired = False
        if tip and table and node.arm_gripper_pair is not None:
            # The two standard ROS messages can straddle a GUI redraw.
            aligned = node.arm_tcp_stamp is not None and node.arm_tcp_stamp == node.arm_table_stamp
            paired = aligned or node.source_age("arm_pair") < 0.15
            if paired:
                table, tip = node.arm_gripper_pair
        projected = None
        if shown and allow_overlay and age_table > ARM_TIMEOUT_SEC:
            projected = node.arm_overlay.table_shape()
            if projected is not None:
                table, age_table, paired = projected, 0.0, False
        tip_color = C_IDLE if age_tcp > ARM_TIMEOUT_SEC else C_ARM
        self._arm_tip.setData([tip[0]] if tip else [], [tip[1]] if tip else [],
                              symbolBrush=tip_color, symbolPen=tip_color)
        self._draw_arm(self._arm_links, table, age_table)
        self._arm_links.setToolTip("Vicon과 본체 기준 링크로 계산한 지도 위치" if projected else "테이블 기준 링크 위치")
        xs, ys = zip(table[0][-1], tip) if paired else ([], [])
        self._gripper_link.setData(xs, ys, pen=pg.mkPen(tip_color, width=3))
        selected = self._links.get(self._vehicles.tabData(self._vehicles.currentIndex()))
        connected = getattr(selected, "ready", False) if selected else node.request is not None
        tcp_text = "테이블 파지점 · 미수신"
        if shown and node.arm_tcp is not None and age_tcp < ARM_TIMEOUT_SEC:
            x, y = node.arm_tcp
            tcp_text = f"테이블 파지점   X {x:.3f} m   Y {y:.3f} m"
            if node.arm_tcp_yaw_deg is not None:
                tcp_text += f"   Yaw {node.arm_tcp_yaw_deg:.1f}°"
        self._tcp_readout.setText(tcp_text)
        alignment = self._live_toss_alignment(selected) if shown else ""
        self._arm_alignment.setText(alignment)
        self._arm_alignment.setVisible(bool(alignment))
        if connected and node.arm_equipped and table is None:
            if base or joints or tip:
                self._arm_notice.setText("로봇팔 지도 위치 미수신")
                self._arm_notice.setToolTip("지도에는 테이블 기준 링크 또는 시각과 좌표가 정합된 Vicon·본체 기준 링크가 필요합니다.")
            else:
                self._arm_notice.setText("로봇팔 신호 미수신")
                self._arm_notice.setToolTip("선택한 기체에서 로봇팔 관절각이나 링크 위치가 수신되지 않았습니다.")
            self._arm_notice.show()
        else:
            self._arm_notice.hide()
        angles = "   ".join(f"J{i} {math.degrees(q):.1f}°" for i, q in enumerate(joints, 1)) if joints else "관절 데이터 대기"
        source = f" · {node.arm_source}" if joints else ""
        held = f" · 마지막 수신 {age_joints:.0f}초 전" if joints and age_joints > ARM_TIMEOUT_SEC else ""
        self._arm_readout.setText(f"로봇팔 · {angles}{source}{held}")

    def _build_summary_panel(self) -> QWidget:
        panel = QFrame()
        panel.setObjectName("panel")
        panel.setMinimumWidth(380)
        panel.setMaximumWidth(500)
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(8)

        title = QLabel("현재 / 목표 / 오차")
        title.setFont(QFont(QApplication.font().family(), 15, QFont.Bold))
        title.setStyleSheet(f"color:{C_TEXT}; border:none;")
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
        ]
        for row, (name, key) in enumerate(rows):
            name_label = QLabel(name)
            name_label.setStyleSheet(f"color:{C_TEXT_DIM}; font-size:12pt; border:none;")
            value_label = QLabel("—")
            value_label.setFont(QFont(QApplication.font().family(), 16, QFont.Bold))
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

    def _choose_comparison_source(self, index):
        self._node.comparison_source = index
        self._update_comparison()

    def _update_comparison(self):
        node = self._node
        source = node.comparison_source
        if source is None:
            source = int(not (node.paper_trial or node.plan_samples) and node.latest_ref is not None)
        self._comparison_source.setCurrentIndex(source)
        for plot in self._comparison_plots[3:]:
            plot.setVisible(node.arm_equipped)
        if source == 0:
            samples = node.plan_samples
            last = samples[-1] if samples else None
            signature = (node, node.error_run, node.error_plan, id(last), len(samples))
            if signature == self._comparison_signature:
                return
            self._comparison_signature = signature
            self._comparison_caption.setText(
                f"{node.error_plan.rsplit('/', 1)[-1]} · {last['plan_time_s']:.2f} s" if last else
                "실측 대기" if node.paper_trial else "실행 계획 대기")
            self._comparison_caption.setToolTip(node.error_run)
            times = [sample["plan_time_s"] for sample in samples]
            for (actual, planned), field in zip(self._comparison_curves,
                    ("x_m", "y_m", "yaw_rad", "shoulder_rad", "elbow_rad", "wrist_rad")):
                for curve, prefix in ((actual, "actual_"), (planned, "planned_")):
                    values = [sample.get(prefix + field) for sample in samples]
                    values = np.asarray([math.nan if value is None else value for value in values], dtype=float)
                    if field.endswith("_rad"):
                        values = np.degrees(values)
                    if field == "yaw_rad" and prefix == "actual_":
                        reference = np.degrees(np.asarray([
                            sample.get("planned_yaw_rad", math.nan) for sample in samples], dtype=float))
                        # Show equivalent headings on the plan's branch across ±180°.
                        values = np.where(np.isfinite(reference),
                                          reference + (values - reference + 180) % 360 - 180, values)
                    curve.setData(times, values, connect="finite")
        else:
            self._comparison_signature = None
            self._comparison_caption.clear()
            self._comparison_caption.setToolTip("")
            for (actual, planned), measured, target in zip(self._comparison_curves[:3],
                    (node.x_v, node.y_v, node.th_v), (node.ref_x, node.ref_y, node.ref_th)):
                actual.setData(list(node.t_v), list(measured), connect="finite")
                planned.setData(list(node.t_r), list(target), connect="finite")
            for (actual, planned), measured in zip(self._comparison_curves[3:], node.joint_history):
                actual.setData(list(node.t_j), list(measured), connect="finite")
                planned.clear()
        label = "계획 시간 [s]" if source == 0 else "시간 [s]"
        for plot in self._comparison_plots:
            plot.setLabel("bottom", label)

    def _build_detail_tabs(self) -> QTabWidget:
        tabs = QTabWidget()
        panel = QWidget()
        layout = QVBoxLayout(panel)
        layout.setContentsMargins(6, 6, 6, 6)
        row = QHBoxLayout()
        self._comparison_source = QComboBox()
        self._comparison_source.setAccessibleName("비교 기준")
        self._comparison_source.addItems(["실험 계획", "제어기 목표"])
        self._comparison_source.activated.connect(self._choose_comparison_source)
        row.addWidget(self._comparison_source)
        self._comparison_caption = QLabel()
        row.addWidget(self._comparison_caption)
        row.addStretch()
        for text, color in (("실측", C_VICON), ("계획", C_REFERENCE)):
            legend = QLabel(text)
            legend.setStyleSheet(f"color:{color}; padding:0 8px;")
            row.addWidget(legend)
        layout.addLayout(row)
        grid = QGridLayout()
        grid.setSpacing(8)
        self._comparison_plots, self._comparison_curves = [], []
        self._comparison_signature = None
        for i, title in enumerate(("X", "Y", "Yaw", "J1", "J2", "J3")):
            plot = pg.PlotWidget()
            _style_plot(plot, title, x_label="계획 시간 [s]", y_label="m" if i < 2 else "°")
            plot.getViewBox().setLimits(minYRange=.05 if i < 2 else 5.)
            actual = plot.plot(pen=pg.mkPen(C_VICON, width=2))
            planned = plot.plot(pen=pg.mkPen(C_REFERENCE, width=2, style=Qt.DashLine))
            if self._comparison_plots:
                plot.setXLink(self._comparison_plots[0])
            self._comparison_plots.append(plot)
            self._comparison_curves.append((actual, planned))
            grid.addWidget(plot, i // 3, i % 3)
        layout.addLayout(grid, 1)
        tabs.addTab(panel, "추종")
        return tabs

    def _build_status_trends(self):
        panel = QWidget()
        panel.setMaximumHeight(210)
        layout = QHBoxLayout(panel)
        layout.setContentsMargins(0, 0, 0, 0)
        self._status_trend_plots = []
        for title, units, span, series, stretch in (
            ("테이블 속도", "m/s", .1,
             [SeriesSpec("t_e", "vx_e", "Vx", C_VICON), SeriesSpec("t_e", "vy_e", "Vy", C_ARM)], 2),
            ("각속도", "°/s", 10., [SeriesSpec("t_e", "om_e", "ω", C_WARN)], 1),
            ("위치 · 1σ", "m", .02,
             [SeriesSpec("t_e", "std_x", "σx", C_VICON), SeriesSpec("t_e", "std_y", "σy", C_ARM)], 1),
            ("Yaw · 1σ", "°", 2., [SeriesSpec("t_e", "std_th", "σθ", C_WARN)], 1),
        ):
            plot = self._make_plot(title, series, units, min_y_span=span)
            plot.setMinimumHeight(130)
            self._status_trend_plots.append(plot)
            layout.addWidget(plot, stretch)
        return panel

    def _set_summary_value(self, key: str, text: str, color: str = C_TEXT) -> None:
        label = self._summary_values[key]
        label.setText(text)
        label.setStyleSheet(f"color:{color}; border:none;")

    def _update_status_cards(self, air_state) -> None:
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
            duties = node.fresh_valve_command()
            detail = f"최대 명령 {max(duties):.2f}" if duties is not None else "명령 미수신"
            self._card_act.set_status("OK", detail, C_OK)
        else:
            self._card_act.set_status("FAULT", "밸브/PLC 상태 확인", C_ERROR)

        if air_state is None:
            self._card_main.set_status("NO DATA", "밸브 상태 미수신", C_ERROR)
        elif air_state:
            self._card_main.set_status("ON", "에어베어링 공급", C_OK)
        else:
            color = C_ERROR if node.mode_name in ("READY", "ACTIVE") else C_IDLE
            self._card_main.set_status("OFF", "에어베어링 차단", color)

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

    def _choose_plan(self, index):
        self._node.plan_choice = self._plan_picker.itemData(index)
        self._update_trajectory()

    def _update_plan_and_get_target(self):
        node = self._node
        if self._picker_catalog is not node.paper_plans:
            self._picker_catalog = node.paper_plans
            self._drawn_plan = None
            self._plan_curve.clear()
            self._plan_picker.clear()
            self._plan_picker.addItem("계획 경로 · 자동", "auto")
            self._plan_picker.addItem("계획 경로 숨기기", "off")
            for key in node.paper_plans:
                experiment, method = key.rsplit("/", 1)
                self._plan_picker.addItem(f"{method}  ·  {experiment}", key)
                self._plan_picker.setItemData(self._plan_picker.count() - 1,
                                              f"저장 계획 미리보기 · {key}", Qt.ToolTipRole)
            self._plan_picker.setCurrentIndex(self._plan_picker.findData(node.plan_choice))
            self._plan_picker.setVisible(bool(node.paper_plans))
        key, target, following = node.plan_target()
        if key != self._drawn_plan:
            self._drawn_plan = key
            if key is None:
                self._plan_curve.clear()
            else:
                xy = node.paper_plans[key][1]
                self._plan_curve.setData(xy[:, 0], xy[:, 1])
        auto_label = "계획 경로 · 자동"
        if node.plan_choice == "auto" and key:
            auto_label += " · " + key.rsplit("/", 1)[-1]
        if self._plan_picker.itemText(0) != auto_label:
            self._plan_picker.setItemText(0, auto_label)
        self._plan_picker.setToolTip("실행 중인 실험의 계획" if following else
                                     f"계획 초기 자세 · {key}" if key else "계획 경로 숨기기")
        return target

    def _update_trajectory(self) -> None:
        node = self._node
        target = self._update_plan_and_get_target()
        visible = target is not None and all(math.isfinite(v) for v in target[:2])
        self._ghost.setVisible(visible)
        if visible:
            x, y, yaw, arm = target
            oriented = yaw is not None and math.isfinite(yaw)
            self._ghost.setPath(self._ghost_outline if oriented else self._ghost_ring)
            self._ghost.setPos(x, y)
            self._ghost.setRotation(math.degrees(yaw) if oriented else 0.)
            heading = f" · yaw {math.degrees(_wrap_angle_rad(yaw)):.1f}°" if oriented else ""
            path = QPainterPath()
            arm_label = ""
            if oriented and arm is not None:
                path.moveTo(*arm[0])
                for point in arm[1:]:
                    path.lineTo(*point)
                arm_label = " · 계획 관절각" if self._drawn_plan else " · 실측 팔 자세"
            self._ghost_arm.setPath(path)
            self._ghost.setToolTip(f"목표 ({x:.2f}, {y:.2f}) m{heading}{arm_label}")
        current = node.source_age("vicon") < XY_TRAIL_SEC
        self._spacecraft_body.setVisible(current)
        self._spacecraft_front.setVisible(current)
        self._position_dot.setVisible(current)
        self._vicon_curve.setData(*node.vicon_trail.coordinates())
        if current:
            x, y = node.x_v[-1], node.y_v[-1]
            heading = math.radians(node.th_v[-1])

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

    def _update_detail_plots(self) -> None:
        node = self._node
        for curve, time_attr, value_attr in self._series_bindings:
            times = getattr(node, time_attr)
            values = getattr(node, value_attr)
            if times and len(times) == len(values):
                curve.setData(list(times), list(values))
            elif curve.xData is not None:
                curve.clear()

        if self._tabs.isVisible() and self._tabs.currentIndex() == 0:
            self._update_comparison()

        duties = node.fresh_valve_command()
        if duties is not None:
            self._valve_cmd_bar.setOpts(height=duties)
        self._valve_cmd_bar.setVisible(duties is not None)
        self._valve_title.setText("밸브 명령" if duties is not None else "밸브 명령 · 미수신")

        self._update_logs()

    def _clear_visible_log(self):
        self._node.logs.clear()
        self._node.log_revision += 1
        self._update_logs()

    def _update_logs(self):
        minimum = self._log_level.currentData()
        key = (self._node.log_revision, minimum)
        if self._log_view_key == key:
            return
        self._log_view_key = key
        scrollbar = self._log_text.verticalScrollBar()
        old_position = scrollbar.value()
        follow = old_position >= scrollbar.maximum() - 1
        visible = (entry.to_html() for entry in self._node.logs if entry.level >= minimum)
        self._log_text.setHtml("<br>".join(visible))
        scrollbar.setValue(scrollbar.maximum() if follow else old_position)

    def _update_controls(self, writable: bool, air_state) -> None:
        """준비도 등 + 버튼 활성/비활성 + 명령 상태를 갱신한다."""
        node = self._node
        ready = node.readiness()
        mode_fresh = node.source_age("mode") <= SOURCE_TIMEOUT_SEC

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
        if air_state is None:
            self._lights["main_valve"].set_state(C_IDLE, "미수신")
        elif air_state:
            self._lights["main_valve"].set_state(C_OK, "ON")
        else:
            self._lights["main_valve"].set_state(
                C_ERROR if armed_mode else C_IDLE, "OFF"
            )

        self._btn_air_on.setEnabled(writable)
        self._btn_air_off.setEnabled(writable)
        self._btn_air_on.setChecked(air_state is True)
        self._btn_air_off.setChecked(air_state is False)

        if node.last_command_msg:
            self._command_status.setText(node.last_command_msg)

    def _update(self) -> None:
        """전체 대시보드를 갱신한다."""
        rate_due = time.monotonic() - self._last_rate >= 1
        if rate_due:
            self._last_rate = time.monotonic()
        names = Counter((link.display_id or endpoint).casefold() for endpoint, link in self._links.items())
        for connected in self._links.values():
            connected.drain()
        for i in range(self._vehicles.count()):
            endpoint = self._vehicles.tabData(i)
            link = self._links.get(endpoint)
            if not link:
                local = self._default_node
                self._vehicles.setTabText(i, f"로컬 ROS · {local.tracking_status()}")
                continue
            data = link.data
            if link.bridge_error:
                self._stop_experiment(data, "telemetry_disconnected")
            self._finish_planned_recording(data)
            self._report_recording_error(data)
            if data.recorder.active:
                data.recorder.capture(data.recording_snapshot())
            if rate_due:
                data._tick_hz()
            if endpoint in self._link_errors:
                status = "연결 끊김" if data.tracking_since is not None else "연결 오류"
            elif link.bridge_error:
                status = "ROS 미연결"
            elif not link.ready:
                status = "연결 중"
            else:
                status = data.tracking_status()
            label = link.display_id or endpoint
            if names[label.casefold()] > 1:
                label = f"{label} ({endpoint})"
            detail = self._link_errors.get(endpoint) or link.bridge_error or link.read_only
            tooltip = f"{link.ssh_user}@{endpoint}"
            if status == "Missing" and link.ready:
                tooltip += "\nSSH 연결됨 · 위치 데이터 미수신 (5초)"
            short_label = self._vehicles.fontMetrics().elidedText(label, Qt.ElideMiddle, 148)
            if label != short_label:
                tooltip = f"{label}\n{tooltip}"
            self._vehicles.setTabToolTip(i, f"{tooltip}\n{detail}" if detail else tooltip)
            self._vehicles.setTabText(i, f"{short_label} · {status}".replace("&", "&&"))
            self._update_body_button(i, endpoint, link, label)
            if (getattr(link, "prompt_existing", False) and link.ready
                    and time.monotonic() - link.last_body_status_at <= SOURCE_TIMEOUT_SEC):
                link.prompt_existing = False
                if self._can_stop_body(endpoint, link):
                    QTimer.singleShot(0, lambda key=endpoint, current=link: self._choose_existing_processes(key, current))
            if endpoint in self._trails:
                trail, marker = self._trails[endpoint]
                xs, ys = data.vicon_trail.coordinates()
                trail.setVisible(bool(xs) and not link.stopped.is_set() and data is not self._node)
                trail.setData(xs, ys)
                if link.stopped.is_set():
                    marker.hide()
                else:
                    marker.show_vehicle(data, label if len(self._links) > 1 else "")
        self._terminal.refresh()
        selected = self._links.get(self._vehicles.tabData(self._vehicles.currentIndex()))
        toss = self._node.control_view == "toss"
        toss_running = selected is not None and selected.workflow == "toss"
        if selected and self._console_tabs.tabData(self._console_tabs.currentIndex()) != selected.shells.selected:
            self._bind_console(selected)
        self._controls.setVisible(not toss)
        self._toss_controls.setVisible(toss)
        if toss:
            self._update_toss_controls(selected)
        enabled = (selected.ready and not selected.read_only and not selected.bridge_error
                   and not selected.stopped.is_set() and not toss_running) if selected else bool(self._node.request)
        shared_domain = selected and self._shared_domain(selected)
        if shared_domain:
            enabled = False
        self._btn_estop.setVisible(not toss_running)
        self._btn_estop.setEnabled(enabled)
        self._release_estop.setVisible(enabled and self._node.mode_name == "EMERGENCY_STOP"
                                       and self._node.source_age("mode") <= SOURCE_TIMEOUT_SEC)
        self._send_target_button.setEnabled(enabled and not toss)
        recorder = self._node.recorder
        self._report_recording_error(self._default_node)
        if self._default_node.recorder.active:
            self._default_node.recorder.capture(self._default_node.recording_snapshot())
        self._experiment_button.setEnabled(recorder.active or self._selected_can_record())
        verb = "종료" if recorder.active else "시작"
        self._experiment_button.setText(f"기록 {verb}")
        self._experiment_button.setToolTip(str(recorder.directory) if recorder.directory else "선택한 FSS 데이터 기록")
        node = self._node
        target = node.source_age("reference") < XY_TRAIL_SEC
        if target:
            summary = f"목표  {node.ref_x[-1]:.2f}, {node.ref_y[-1]:.2f} m"
            if node.source_age("vicon") < XY_TRAIL_SEC:
                error = math.hypot(node.x_v[-1] - node.ref_x[-1], node.y_v[-1] - node.ref_y[-1])
                summary += f"    거리 차  {error:.3f} m"
            self._position_summary.setText(summary)
        self._position_summary.setVisible(target)
        air_state = node.reported_air_state() if selected is None or (selected.ready and not selected.stopped.is_set()) else None
        self._update_status_cards(air_state)
        self._update_controls(enabled, air_state)
        toss_air_enabled = bool(toss and selected and not shared_domain and selected.can_toss_air())
        for button, state in ((self._toss_air_on, True), (self._toss_air_off, False)):
            button.setEnabled(toss_air_enabled)
            button.setChecked(air_state is state)
        if shared_domain:
            self._command_status.setText("공유 ROS 도메인 · 기체별 도메인을 지정하세요")
        elif not self._node.last_command_msg:
            self._command_status.clear()
        self._command_status.setVisible(bool(self._command_status.text()))
        self._update_summary()
        self._update_trajectory()
        self._update_arm(allow_overlay=not shared_domain)
        occupied = []
        if selected and (key := self._vehicles.tabData(self._vehicles.currentIndex())) in self._trails:
            self._trails[key][1].avoid_other_bubbles(occupied)
        self._local_bubble.avoid_other_bubbles(occupied)
        for key, (_, bubble) in self._trails.items():
            if self._links.get(key) is not selected:
                bubble.avoid_other_bubbles(occupied)
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

    app = QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    data = DashboardNode(args.history_sec)
    local = None
    try:
        import rclpy
    except ModuleNotFoundError as exc:
        if exc.name != "rclpy":
            raise
    else:
        local, data.request = open_node(
            data.receive,
            data.set_command_result,
            ros_args,
            reject=lambda text: data.set_command_result(text, 40),
        )
        local.create_timer(1.0, data._tick_hz)
    window = Dashboard(data, args.fps, args.history_sec)
    if local:
        ros_timer = QTimer(window)
        ros_timer.timeout.connect(lambda: rclpy.spin_once(local, timeout_sec=0.0))
        ros_timer.start(2)
    window._update()

    if args.fullscreen:
        window.showFullScreen()
    else:
        window.show()

    try:
        exit_code = app.exec()
    finally:
        window.deleteLater()
        QCoreApplication.sendPostedEvents(None, QEvent.DeferredDelete)
        if local:
            local.destroy_node()
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
