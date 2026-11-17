"""Fixed-table trajectory movie, rendered with Qt and encoded as H.264 MP4."""

from collections import deque
import math
import os
from pathlib import Path
import sys

from PySide6.QtCore import QObject, QPointF, QRectF, QTimer, QUrl, Qt, Signal
from PySide6.QtGui import QColor, QFont, QGuiApplication, QImage, QPainter, QPainterPath, QPen


def _finite(values, count):
    return (isinstance(values, (tuple, list)) and len(values) == count
            and all(isinstance(v, (int, float)) and math.isfinite(v) for v in values))


class TrajectoryRenderer:
    WIDTH, HEIGHT = 640, 840
    LEFT, BOTTOM, SCALE = 60, 770, 180

    def __init__(self, fps=10):
        self.trail = deque(maxlen=max(2, round(fps * 5) + 2))

    @classmethod
    def screen(cls, x, y):
        return QPointF(cls.LEFT + cls.SCALE * x, cls.BOTTOM - cls.SCALE * y)

    def render(self, snapshot, elapsed):
        pose = snapshot.get("pose")
        if _finite(pose, 3):
            self.trail.append((elapsed, pose[0], pose[1]))
        while self.trail and self.trail[0][0] <= elapsed - 5 + 1e-9:
            self.trail.popleft()
        image = QImage(self.WIDTH, self.HEIGHT, QImage.Format_RGB32)
        image.fill(QColor("#fafafc"))
        painter = QPainter(image)
        try:
            painter.setRenderHint(QPainter.Antialiasing)
            font = QFont(QGuiApplication.font())
            font.setPointSize(11)
            painter.setFont(font)
            painter.setPen(QColor("#636368"))
            painter.drawText(60, 29, f"{elapsed:07.2f} s")
            painter.drawText(288, 822, "X [m]")
            painter.drawText(14, 35, "Y [m]")
            for n in range(9):
                y = n / 2
                p = self.screen(0, y)
                painter.setPen(QColor("#e0e0e0"))
                painter.drawLine(p, self.screen(3, y))
                painter.setPen(QColor("#636368"))
                painter.drawText(QPointF(24, p.y() + 4), f"{y:g}")
            for n in range(7):
                x = n / 2
                p = self.screen(x, 0)
                painter.setPen(QColor("#e0e0e0"))
                painter.drawLine(p, self.screen(x, 4))
                painter.setPen(QColor("#636368"))
                painter.drawText(QPointF(p.x() - 5, 793), f"{x:g}")
            table_bounds = QRectF(self.LEFT, self.BOTTOM - 4 * self.SCALE,
                                 3 * self.SCALE, 4 * self.SCALE)
            painter.setPen(QPen(QColor("#636368"), 8))
            painter.setBrush(Qt.NoBrush)
            painter.drawRect(table_bounds)
            painter.setClipRect(table_bounds)
            plan = snapshot.get("plan")
            if isinstance(plan, (tuple, list)):
                self._polyline(painter, plan, QPen(QColor(130, 135, 142, 65), 1, Qt.DashLine))
            for earlier, later in zip(self.trail, list(self.trail)[1:]):
                if later[0] - earlier[0] <= 0.5:
                    alpha = round(200 * max(0, 1 - (elapsed - later[0]) / 5))
                    painter.setPen(QPen(QColor(0, 102, 204, alpha), 2))
                    painter.drawLine(self.screen(*earlier[1:]), self.screen(*later[1:]))
            self._body(painter, snapshot.get("ghost"), snapshot.get("ghost_arm"), ghost=True)
            self._body(painter, pose, snapshot.get("arm"), ghost=False)
        finally:
            painter.end()
        return image

    def _polyline(self, painter, points, pen):
        path = QPainterPath()
        connected = False
        for point in points:
            if not _finite(point, 2):
                connected = False
                continue
            pixel = self.screen(*point)
            if connected:
                path.lineTo(pixel)
            else:
                path.moveTo(pixel)
            connected = True
        painter.setPen(pen)
        painter.setBrush(Qt.NoBrush)
        painter.drawPath(path)

    def _body(self, painter, pose, arm, ghost):
        if not isinstance(pose, (tuple, list)) or len(pose) != 3 or not _finite(pose[:2], 2):
            return
        x, y, yaw = pose
        color = QColor(175, 111, 47, 126) if ghost else QColor("#39424d")
        pen = QPen(color, 2, Qt.DashLine if ghost else Qt.SolidLine)
        painter.setPen(pen)
        if not isinstance(yaw, (float, int)) or not math.isfinite(yaw):
            painter.setBrush(Qt.NoBrush)
            painter.drawEllipse(self.screen(x, y), 7, 7)
            return
        cosine, sine = math.cos(yaw), math.sin(yaw)
        def table(point):
            a, b = point
            return x + cosine * a - sine * b, y + sine * a + cosine * b
        corners = [table(p) for p in ((-.15, -.15), (.15, -.15), (.15, .15), (-.15, .15), (-.15, -.15))]
        path = QPainterPath(self.screen(*corners[0]))
        for point in corners[1:]:
            path.lineTo(self.screen(*point))
        painter.setBrush(QColor(232, 147, 56, 18) if ghost else QColor(57, 66, 77, 48))
        painter.drawPath(path)
        if isinstance(arm, (tuple, list)) and all(_finite(p, 2) for p in arm):
            arm_color = QColor(175, 111, 47, 146) if ghost else QColor("#00877d")
            self._polyline(painter, [table(p) for p in arm],
                           QPen(arm_color, 2 if ghost else 3, Qt.DashLine if ghost else Qt.SolidLine))
        painter.setBrush(color)
        painter.drawEllipse(self.screen(x, y), 3, 3)


class VideoRecorder(QObject):
    """Call on the Qt GUI thread; retain until finalized after stop()."""

    finalized = Signal(str)

    def __init__(self, directory, fps=10):
        super().__init__()
        self.path = Path(directory) / "trajectory.mp4"
        self.fps = fps
        self.error = ""
        self.finished = False
        self.frames_written = 0
        self.frames_dropped = 0
        self._stopping = self._eof = self._pumping = False
        self._last_index = -1
        self._pending = deque()
        self._renderer = TrajectoryRenderer(fps)
        self._timeout = QTimer(self)
        self._timeout.setSingleShot(True)
        self._timeout.timeout.connect(lambda: self._fail("MP4 마무리 시간 초과"))
        try:
            if fps <= 0 or fps > 60:
                raise ValueError("Video fps must be between 1 and 60")
            if self.path.exists():
                raise FileExistsError(self.path)
            # Qt's FFmpeg plugin also needs Conda's DLLs when Python is launched directly.
            dlls = Path(sys.prefix) / "Library" / "bin"
            if sys.platform == "win32" and dlls.is_dir() and str(dlls) not in os.environ.get("PATH", "").split(os.pathsep):
                os.environ["PATH"] = str(dlls) + os.pathsep + os.environ.get("PATH", "")
            os.environ.setdefault("QT_MEDIA_BACKEND", "ffmpeg")
            from PySide6.QtMultimedia import QMediaCaptureSession, QMediaFormat, QMediaRecorder, QVideoFrameInput
            self._session = QMediaCaptureSession(self)
            self._input = QVideoFrameInput(self)
            self._recorder = QMediaRecorder(self)
            self._session.setVideoFrameInput(self._input)
            self._session.setRecorder(self._recorder)
            media_format = QMediaFormat(QMediaFormat.MPEG4)
            media_format.setVideoCodec(QMediaFormat.H264)
            self._recorder.setMediaFormat(media_format)
            self._recorder.setVideoResolution(self._renderer.WIDTH, self._renderer.HEIGHT)
            self._recorder.setVideoFrameRate(fps)
            self._recorder.setVideoBitRate(1_500_000)
            self._recorder.setAutoStop(True)
            self._recorder.setOutputLocation(QUrl.fromLocalFile(str(self.path.resolve())))
            self._recorder.errorOccurred.connect(lambda code, text: self._fail(text))
            self._recorder.recorderStateChanged.connect(self._state_changed)
            self._input.readyToSendVideoFrame.connect(self._pump)
            self._recorder.record()
        except (ImportError, OSError, RuntimeError, ValueError) as exc:
            self._fail(str(exc))

    def capture(self, snapshot, elapsed_seconds):
        if self.finished or self._stopping or not math.isfinite(elapsed_seconds) or elapsed_seconds < 0:
            return
        index = math.floor(elapsed_seconds * self.fps + 1e-6)
        if index <= self._last_index:
            return
        self._last_index = index
        from PySide6.QtMultimedia import QVideoFrame
        frame = QVideoFrame(self._renderer.render(snapshot, elapsed_seconds))
        frame.setStartTime(round(elapsed_seconds * 1_000_000))
        frame.setEndTime(round((elapsed_seconds + 1 / self.fps) * 1_000_000))
        if len(self._pending) == 2:
            self._pending.popleft()
            self.frames_dropped += 1
        self._pending.append(frame)
        self._pump()

    def _pump(self):
        if self._pumping or self.finished:
            return
        self._pumping = True
        try:
            while self._pending:
                if not self._input.sendVideoFrame(self._pending[0]):
                    return
                self._pending.popleft()
                self.frames_written += 1
            if self._stopping and not self._eof:
                from PySide6.QtMultimedia import QVideoFrame
                self._eof = self._input.sendVideoFrame(QVideoFrame())
        finally:
            self._pumping = False

    def stop(self):
        if self.finished or self._stopping:
            return
        if not self.frames_written and not self._pending:
            self.capture({}, 0)
        self._stopping = True
        self._timeout.start(15000)
        self._pump()

    def _state_changed(self, state):
        from PySide6.QtMultimedia import QMediaRecorder
        if self._stopping and state == QMediaRecorder.StoppedState and not self.finished:
            self._timeout.stop()
            self.finished = True
            self.finalized.emit(str(self.path))

    def _fail(self, message):
        if self.finished:
            return
        self.error = message or "MP4 인코딩 오류"
        self.finished = True
        self._timeout.stop()
        self._pending.clear()
        if hasattr(self, "_recorder"):
            self._recorder.stop()
        self.finalized.emit(str(self.path))
