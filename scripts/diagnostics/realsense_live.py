#!/usr/bin/env python3
"""RealSense D435i 실시간 진단 도구 (PySide6 + pyqtgraph).

Jetson 직접 실행 (Unit test) 또는 노트북에서 원격 실행 (실험용) 모두 지원.
ROS2 DDS가 같은 네트워크/DOMAIN_ID에서 자동으로 토픽을 공유한다.

사용법:
  [Jetson 직접] ssh -X 또는 HDMI 모니터 연결 후:
    python3 scripts/diagnostics/realsense_live.py

  [노트북 원격] 같은 WiFi, 같은 ROS_DOMAIN_ID:
    python3 scripts/diagnostics/realsense_live.py --compressed

옵션:
  --compressed    압축 이미지 구독 (WiFi 대역폭 절약, ~1/20 트래픽)
  --depth-only    Depth만 표시 (Color 구독 안 함, 대역폭 추가 절약)

표시 항목:
  - Depth 이미지 (컬러맵 적용)
  - Color 이미지 (RGB)
  - 수신 Hz, 해상도, 전송 모드
"""

import argparse
import sys

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import QTimer, Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QApplication,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QVBoxLayout,
    QWidget,
)

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

C_BG = "#0d1117"
C_TEXT = "#e6edf3"
C_ACCENT = "#00d4ff"

# RealSense 토픽 경로 (realsense2_camera 기본)
DEPTH_RAW = "/camera/camera/depth/image_rect_raw"
COLOR_RAW = "/camera/camera/color/image_raw"
DEPTH_COMPRESSED = "/camera/camera/depth/image_rect_raw/compressedDepth"
COLOR_COMPRESSED = "/camera/camera/color/image_raw/compressed"


def _decode_raw_depth(msg: Image) -> np.ndarray | None:
    """sensor_msgs/Image (16UC1) → numpy 배열."""
    if msg.encoding not in ("16UC1", "mono16"):
        return None
    arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
    return arr


def _decode_raw_color(msg: Image) -> np.ndarray | None:
    """sensor_msgs/Image (rgb8/bgr8) → numpy RGB 배열."""
    if msg.encoding == "rgb8":
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
    if msg.encoding == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
        return arr[:, :, ::-1]  # BGR → RGB
    return None


def _decode_compressed_color(msg: CompressedImage) -> np.ndarray | None:
    """sensor_msgs/CompressedImage (jpeg) → numpy RGB 배열."""
    try:
        import cv2
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        return img[:, :, ::-1] if img is not None else None  # BGR → RGB
    except ImportError:
        return None


class RealSenseDiagNode(Node):
    """RealSense 진단 ROS2 노드."""

    def __init__(self, use_compressed: bool = False, depth_only: bool = False) -> None:
        super().__init__("realsense_diag_node")

        self.depth_img: np.ndarray | None = None
        self.color_img: np.ndarray | None = None
        self.depth_hz = 0.0
        self.color_hz = 0.0
        self.depth_res = ""
        self.color_res = ""
        self._dc = 0
        self._cc = 0
        self._mode = "compressed" if use_compressed else "raw"

        # Depth 구독 (항상 raw — compressed depth는 특수 포맷이라 raw가 낫다)
        self.create_subscription(Image, DEPTH_RAW, self._on_depth_raw, 5)

        # Color 구독
        if not depth_only:
            if use_compressed:
                self.create_subscription(
                    CompressedImage, COLOR_COMPRESSED, self._on_color_compressed, 5
                )
            else:
                self.create_subscription(Image, COLOR_RAW, self._on_color_raw, 5)

        self.create_timer(1.0, self._tick)
        self.get_logger().info(
            f"RealSense 진단 시작 (모드: {self._mode}, depth_only: {depth_only})"
        )

    def _on_depth_raw(self, msg: Image) -> None:
        img = _decode_raw_depth(msg)
        if img is not None:
            self.depth_img = img
            self.depth_res = f"{msg.width}×{msg.height}"
            self._dc += 1

    def _on_color_raw(self, msg: Image) -> None:
        img = _decode_raw_color(msg)
        if img is not None:
            self.color_img = img
            self.color_res = f"{msg.width}×{msg.height}"
            self._cc += 1

    def _on_color_compressed(self, msg: CompressedImage) -> None:
        img = _decode_compressed_color(msg)
        if img is not None:
            self.color_img = img
            h, w = img.shape[:2]
            self.color_res = f"{w}×{h}"
            self._cc += 1

    def _tick(self) -> None:
        self.depth_hz = float(self._dc)
        self.color_hz = float(self._cc)
        self._dc = 0
        self._cc = 0


class RealSenseDiagWindow(QMainWindow):
    """RealSense 실시간 진단 윈도우."""

    def __init__(self, node: RealSenseDiagNode) -> None:
        super().__init__()
        self._node = node
        self.setWindowTitle("RealSense D435i 진단")
        self.resize(1200, 650)
        self.setStyleSheet(f"""
            QMainWindow, QWidget {{ background: {C_BG}; color: {C_TEXT}; }}
            QLabel {{ color: {C_TEXT}; }}
        """)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout(central)
        layout.setContentsMargins(6, 6, 6, 6)

        # 상태 라벨
        self._lbl = QLabel("대기 중...")
        self._lbl.setFont(QFont("monospace", 13, QFont.Bold))
        self._lbl.setStyleSheet(f"color: {C_ACCENT}; padding: 4px;")
        layout.addWidget(self._lbl)

        # 이미지 뷰
        img_layout = QHBoxLayout()

        # Depth 뷰
        self._depth_widget = pg.GraphicsLayoutWidget()
        self._depth_vb = self._depth_widget.addViewBox(row=0, col=0)
        self._depth_vb.setAspectLocked(True)
        self._depth_item = pg.ImageItem()
        self._depth_vb.addItem(self._depth_item)
        # 컬러맵 (Jet)
        cmap = pg.colormap.get("CET-L4")
        self._depth_item.setLookupTable(cmap.getLookupTable(nPts=256))
        lbl_d = QLabel("Depth")
        lbl_d.setFont(QFont("sans-serif", 13, QFont.Bold))
        lbl_d.setAlignment(Qt.AlignCenter)
        lbl_d.setStyleSheet(f"color: {C_ACCENT};")

        depth_container = QVBoxLayout()
        depth_container.addWidget(lbl_d)
        depth_container.addWidget(self._depth_widget)
        img_layout.addLayout(depth_container)

        # Color 뷰
        self._color_widget = pg.GraphicsLayoutWidget()
        self._color_vb = self._color_widget.addViewBox(row=0, col=0)
        self._color_vb.setAspectLocked(True)
        self._color_item = pg.ImageItem()
        self._color_vb.addItem(self._color_item)
        lbl_c = QLabel("Color")
        lbl_c.setFont(QFont("sans-serif", 13, QFont.Bold))
        lbl_c.setAlignment(Qt.AlignCenter)
        lbl_c.setStyleSheet(f"color: {C_ACCENT};")

        color_container = QVBoxLayout()
        color_container.addWidget(lbl_c)
        color_container.addWidget(self._color_widget)
        img_layout.addLayout(color_container)

        layout.addLayout(img_layout)

        # 갱신 타이머 (15 FPS)
        self._timer = QTimer()
        self._timer.timeout.connect(self._update)
        self._timer.start(66)

    def _update(self) -> None:
        nd = self._node

        # 상태 표시
        d_info = f"● {nd.depth_hz:.0f}Hz {nd.depth_res}" if nd.depth_hz > 0 else "○ 미수신"
        c_info = f"● {nd.color_hz:.0f}Hz {nd.color_res}" if nd.color_hz > 0 else "○ 미수신"
        self._lbl.setText(
            f"Depth: {d_info}  |  Color: {c_info}  |  모드: {nd._mode}"
        )

        # Depth 이미지 갱신
        if nd.depth_img is not None:
            d = nd.depth_img.astype(np.float32)
            d = np.clip(d / 4000.0, 0, 1)  # 4m 범위 정규화
            self._depth_item.setImage(d.T, levels=(0, 1))

        # Color 이미지 갱신
        if nd.color_img is not None:
            # pyqtgraph: (width, height, 3) 순서
            self._color_item.setImage(
                np.transpose(nd.color_img, (1, 0, 2)),
                levels=(0, 255),
            )


def main() -> None:
    parser = argparse.ArgumentParser(description="RealSense D435i 실시간 진단")
    parser.add_argument("--compressed", action="store_true",
                        help="압축 이미지 구독 (WiFi 대역폭 절약)")
    parser.add_argument("--depth-only", action="store_true",
                        help="Depth만 표시 (Color 미구독)")
    args = parser.parse_args()

    # --compressed는 cv2로 디코드 — 미설치면 컬러가 조용히 "미수신"이 되므로 시작 시 명시 실패
    if args.compressed and not args.depth_only:
        import importlib.util
        if importlib.util.find_spec("cv2") is None:
            print(
                "오류: --compressed 모드는 opencv-python(cv2)이 필요합니다. "
                "설치: pip install opencv-python (또는 --compressed 없이 실행)",
                file=sys.stderr,
            )
            sys.exit(1)

    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    rclpy.init()
    node = RealSenseDiagNode(
        use_compressed=args.compressed,
        depth_only=args.depth_only,
    )
    win = RealSenseDiagWindow(node)

    # Qt와 ROS 콜백을 같은 스레드에서 처리해 이미지 데이터 경합을 없앤다.
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
