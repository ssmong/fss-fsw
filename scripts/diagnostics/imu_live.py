#!/usr/bin/env python3
"""IMU 실시간 진단 도구 — Xsens MTi-630 raw + FSS bridge 비교.

raw /imu/data (Xsens 센서 프레임)와 bridge /sensors/imu (FSS body) 토픽을 동시에 표시.
고정 장착 quaternion 변환과 축 방향이 올바른지 실시간으로 확인한다.

사용법:
  # 1) Xsens 드라이버 실행 중인 상태에서
  # 2) (선택) imu_bridge_node도 실행
  python3 scripts/diagnostics/imu_live.py

표시 항목:
  - 자이로 3축 (raw vs bridge)
  - 가속도 3축 (raw vs bridge)
  - 평면 자세 yaw (쿼터니언 → yaw)
"""

import math
import sys
from collections import deque

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QGridLayout,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

# 색상
C_BG = "#0d1117"
C_TEXT = "#e6edf3"
C_TEXT_DIM = "#a7b1bd"
C_BORDER = "#27313d"
C_RAW = "#58a6ff"      # raw (Xsens): 파란 점선
C_BRIDGE = "#f2cc60"   # bridge (FSS): 노랑 실선
C_ACCENT = "#00d4ff"

HISTORY = 500  # 샘플 수 (~5초 @ 100Hz)


def _style_plot(plot: pg.PlotWidget, title: str, *, legend: bool = False) -> None:
    """3×3 그리드에서도 축·범례·제목을 읽을 수 있게 만든다."""
    plot.setTitle(title, color=C_TEXT, size="12pt")
    plot.setLabel(
        "bottom",
        "시간 [s]",
        **{"color": C_TEXT_DIM, "font-size": "10pt"},
    )
    tick_font = QFont("sans-serif", 10)
    for axis_name in ("bottom", "left"):
        axis = plot.getAxis(axis_name)
        axis.setTickFont(tick_font)
        axis.setPen(C_BORDER)
        axis.setTextPen(C_TEXT_DIM)
        axis.enableAutoSIPrefix(False)
    plot.showGrid(x=True, y=True, alpha=0.2)
    if legend:
        plot.addLegend(
            offset=(8, 6),
            colCount=2,
            horSpacing=10,
            labelTextColor=C_TEXT,
            labelTextSize="10pt",
            brush=pg.mkBrush(13, 17, 23, 220),
            pen=pg.mkPen(C_BORDER),
        )


class IMUDiagNode(Node):
    """IMU 진단용 ROS2 노드."""

    def __init__(self) -> None:
        super().__init__("imu_diag_node")
        n = HISTORY

        # Raw (Xsens 드라이버 직접 출력)
        self.raw_t: deque[float] = deque(maxlen=n)
        self.raw_gx: deque[float] = deque(maxlen=n)
        self.raw_gy: deque[float] = deque(maxlen=n)
        self.raw_gz: deque[float] = deque(maxlen=n)
        self.raw_ax: deque[float] = deque(maxlen=n)
        self.raw_ay: deque[float] = deque(maxlen=n)
        self.raw_az: deque[float] = deque(maxlen=n)
        self.raw_yaw: deque[float] = deque(maxlen=n)

        # Bridge (FSS 좌표계 변환 후)
        self.brg_t: deque[float] = deque(maxlen=n)
        self.brg_gx: deque[float] = deque(maxlen=n)
        self.brg_gy: deque[float] = deque(maxlen=n)
        self.brg_gz: deque[float] = deque(maxlen=n)
        self.brg_ax: deque[float] = deque(maxlen=n)
        self.brg_ay: deque[float] = deque(maxlen=n)
        self.brg_az: deque[float] = deque(maxlen=n)

        self.raw_hz = 0.0
        self.brg_hz = 0.0
        self._rc = 0
        self._bc = 0
        self._t0: float | None = None

        self.create_subscription(Imu, "/imu/data", self._on_raw, 10)
        self.create_subscription(Imu, "/sensors/imu", self._on_bridge, 10)
        self.create_timer(1.0, self._tick)

    def _t(self, stamp) -> float:
        t = stamp.sec + stamp.nanosec * 1e-9
        if self._t0 is None:
            self._t0 = t
        return t - self._t0

    def _quat_to_yaw(self, q) -> float:
        """쿼터니언에서 평면 회전각 yaw [deg]만 추출한다."""
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.degrees(math.atan2(siny, cosy))

    def _on_raw(self, msg: Imu) -> None:
        t = self._t(msg.header.stamp)
        self.raw_t.append(t)
        g = msg.angular_velocity
        a = msg.linear_acceleration
        self.raw_gx.append(g.x); self.raw_gy.append(g.y); self.raw_gz.append(g.z)
        self.raw_ax.append(a.x); self.raw_ay.append(a.y); self.raw_az.append(a.z)
        self.raw_yaw.append(self._quat_to_yaw(msg.orientation))
        self._rc += 1

    def _on_bridge(self, msg: Imu) -> None:
        t = self._t(msg.header.stamp)
        self.brg_t.append(t)
        g = msg.angular_velocity
        a = msg.linear_acceleration
        self.brg_gx.append(g.x); self.brg_gy.append(g.y); self.brg_gz.append(g.z)
        self.brg_ax.append(a.x); self.brg_ay.append(a.y); self.brg_az.append(a.z)
        self._bc += 1

    def _tick(self) -> None:
        self.raw_hz = float(self._rc); self.brg_hz = float(self._bc)
        self._rc = 0; self._bc = 0


class IMUDiagWindow(QMainWindow):
    """IMU 실시간 진단 윈도우."""

    def __init__(self, node: IMUDiagNode) -> None:
        super().__init__()
        self._node = node
        self.setWindowTitle("IMU 진단 — Xsens MTi-630 (raw vs bridge)")
        self.resize(1400, 900)
        self.setStyleSheet(f"QMainWindow, QWidget {{ background: {C_BG}; color: {C_TEXT}; }}")

        pg.setConfigOptions(antialias=True, background=C_BG, foreground=C_TEXT)

        central = QWidget()
        self.setCentralWidget(central)
        grid = QGridLayout(central)
        grid.setSpacing(4)

        # 상태 라벨
        self._lbl = QLabel("대기 중...")
        self._lbl.setFont(QFont("monospace", 13, QFont.Bold))
        self._lbl.setStyleSheet(f"color: {C_ACCENT}; padding: 4px;")
        grid.addWidget(self._lbl, 0, 0, 1, 3)

        # 자이로 3축
        titles_g = ["자이로 X [rad/s]", "자이로 Y [rad/s]", "자이로 Z [rad/s]"]
        self._gyro_plots = []
        self._gyro_raw = []
        self._gyro_brg = []
        for i, t in enumerate(titles_g):
            p = pg.PlotWidget()
            _style_plot(p, t, legend=True)
            raw_c = p.plot(
                pen=pg.mkPen(C_RAW, width=2.0, style=Qt.DotLine), name="Raw"
            )
            brg_c = p.plot(pen=pg.mkPen(C_BRIDGE, width=2.4), name="Bridge")
            grid.addWidget(p, 1, i)
            self._gyro_plots.append(p)
            self._gyro_raw.append(raw_c)
            self._gyro_brg.append(brg_c)

        # 가속도 3축
        titles_a = ["가속도 X [m/s²]", "가속도 Y [m/s²]", "가속도 Z [m/s²]"]
        self._accel_plots = []
        self._accel_raw = []
        self._accel_brg = []
        for i, t in enumerate(titles_a):
            p = pg.PlotWidget()
            _style_plot(p, t, legend=True)
            raw_c = p.plot(
                pen=pg.mkPen(C_RAW, width=2.0, style=Qt.DotLine), name="Raw"
            )
            brg_c = p.plot(pen=pg.mkPen(C_BRIDGE, width=2.4), name="Bridge")
            grid.addWidget(p, 2, i)
            self._accel_plots.append(p)
            self._accel_raw.append(raw_c)
            self._accel_brg.append(brg_c)

        # 제어 상태에 포함되는 평면 자세 yaw만 표시한다.
        yaw_plot = pg.PlotWidget()
        _style_plot(yaw_plot, "평면 자세 Yaw [°]")
        self._yaw_curve = yaw_plot.plot(pen=pg.mkPen("#f0883e", width=2.2))
        grid.addWidget(yaw_plot, 3, 0, 1, 3)

        # 갱신 타이머 (30 FPS)
        self._timer = QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(33)

    def _update(self) -> None:
        nd = self._node
        rt = list(nd.raw_t)
        bt = list(nd.brg_t)

        # 상태
        brg_status = f"● {nd.brg_hz:.0f}Hz" if nd.brg_hz > 0 else "○ 미연결"
        status = (
            f"Raw /imu/data: ● {nd.raw_hz:.0f}Hz  |  "
            f"Bridge /sensors/imu: {brg_status}"
        )
        if nd.brg_hz <= 0:
            status += "  |  시작: ros2 launch fss_bringup sensors.launch.py"
        self._lbl.setText(status)

        # 자이로
        raw_g = [list(nd.raw_gx), list(nd.raw_gy), list(nd.raw_gz)]
        brg_g = [list(nd.brg_gx), list(nd.brg_gy), list(nd.brg_gz)]
        for i in range(3):
            if rt:
                self._gyro_raw[i].setData(rt, raw_g[i])
            if bt:
                self._gyro_brg[i].setData(bt, brg_g[i])

        # 가속도
        raw_a = [list(nd.raw_ax), list(nd.raw_ay), list(nd.raw_az)]
        brg_a = [list(nd.brg_ax), list(nd.brg_ay), list(nd.brg_az)]
        for i in range(3):
            if rt:
                self._accel_raw[i].setData(rt, raw_a[i])
            if bt:
                self._accel_brg[i].setData(bt, brg_a[i])

        # 평면 자세 Yaw
        if rt:
            self._yaw_curve.setData(rt, list(nd.raw_yaw))


def main() -> None:
    rclpy.init()
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    node = IMUDiagNode()
    win = IMUDiagWindow(node)

    # Qt와 ROS 콜백을 같은 스레드에서 처리해 plot 데이터 경합을 없앤다.
    ros_timer = QTimer(win)
    ros_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.0))
    ros_timer.start(2)

    win.show()
    try:
        exit_code = app.exec()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
