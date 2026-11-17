"""Per-vehicle Mission Control experiment archives on the ground station."""

from __future__ import annotations

from collections import Counter
from contextlib import ExitStack
from datetime import datetime
import gzip
import json
import math
from pathlib import Path, PureWindowsPath
import re
import time
from types import SimpleNamespace

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen


def _plain(value):
    if isinstance(value, SimpleNamespace):
        return vars(value)
    if hasattr(value, "get_fields_and_field_types"):
        from rosidl_runtime_py.convert import message_to_ordereddict
        return message_to_ordereddict(value)
    if hasattr(value, "tolist"):
        return value.tolist()
    raise TypeError(f"Unsupported payload: {type(value).__name__}")


def _json(value):
    # ROS health messages can use infinity for an unknown measurement.
    text = json.dumps(value, ensure_ascii=False, separators=(",", ":"), default=_plain)
    if any(word in text for word in ("Infinity", "NaN")):
        text = json.dumps(json.loads(text, parse_constant=lambda _: None),
                          ensure_ascii=False, separators=(",", ":"))
    return text


def _run_directory(root, name, now):
    name = name.strip() or "default"
    if (name in (".", "..") or name.endswith(".") or len(name) > 100
            or re.search(r'[<>:"/\\|?*\x00-\x1f]', name) or PureWindowsPath(name).is_reserved()):
        raise ValueError("실험 이름에는 폴더 이름으로 쓸 수 있는 문자만 입력하세요.")
    root = Path(root).resolve()
    day = (root / name / now.date().isoformat()).resolve()
    if not day.is_relative_to(root):
        raise ValueError("실험 폴더가 기록 경로를 벗어납니다.")
    day.mkdir(parents=True, exist_ok=True)
    clock = now.strftime("%H-%M-%S")
    index = 1
    while True:
        run = day / (clock if index == 1 else f"{clock}-{index:02d}")
        try:
            run.mkdir()
            return run
        except FileExistsError:
            index += 1


class ExperimentRecorder:
    def __init__(self, context=None):
        self.context = context
        self.directory = None
        self.active = False
        self.error = ""

    def start(self, root, name, vehicle):
        if self.active:
            raise ValueError("이 기체의 실험을 이미 기록 중입니다.")
        now = datetime.now().astimezone()
        self.directory = _run_directory(root, name, now)
        self._files = ExitStack()
        try:
            self._data = self._files.enter_context((self.directory / "telemetry.jsonl.gz").open("xb"))
            self._logs = self._files.enter_context((self.directory / "logs.txt").open("x", encoding="utf-8", buffering=1))
            self._buffer = bytearray()
            self._start_ns = time.monotonic_ns()
            self._last_flush_ns = 0
            self._counts = Counter()
            self._paths = {}
            self._path_counts = Counter()
            self._strides = Counter()
            self._summary = {"format_version": 1, "experiment": name.strip() or "default", "vehicle": vehicle,
                             "started_at": now.isoformat(), "status": "recording"}
            self._save_summary()
            self.active = True
            self.error = ""
            if self.context:
                self.record("session_context", self.context())
                if self.error:
                    raise OSError(self.error)
        except Exception:
            self.active = False
            self._files.close()
            raise
        return self.directory

    def _save_summary(self):
        tmp = self.directory / "session.tmp"
        tmp.write_text(json.dumps(self._summary, ensure_ascii=False, indent=2), encoding="utf-8")
        tmp.replace(self.directory / "session.json")

    def _flush(self):
        if self._buffer:
            # Each chunk is a complete gzip member, so earlier chunks survive a process crash.
            self._data.write(gzip.compress(self._buffer, compresslevel=1, mtime=0))
            self._data.flush()
            self._buffer.clear()

    def _fail(self, exc):
        self.active = False
        self.error = f"실험 기록 오류 · {exc}"
        try:
            self._files.close()
        except OSError:
            pass
        self._summary.update(status="error", error=str(exc), counts=dict(self._counts))
        try:
            self._save_summary()
        except OSError:
            pass

    def record(self, kind, value):
        if not self.active:
            return
        now = time.monotonic_ns()
        try:
            row = {"received_unix_ns": time.time_ns(), "elapsed_s": (now - self._start_ns) * 1e-9,
                   "kind": kind, "data": value}
            self._buffer.extend((_json(row) + "\n").encode("utf-8"))
            self._counts[kind] += 1
            if now - self._last_flush_ns >= 250_000_000 or len(self._buffer) >= 262144:
                self._flush()
                self._last_flush_ns = now
            if kind in ("vicon", "ekf"):
                self._point(kind, value.x, value.y)
            elif kind == "ref":
                self._point(kind, value.x_ref, value.y_ref)
            elif kind == "arm_tcp" and value.header.frame_id == "table":
                self._point(kind, value.pose.position.x, value.pose.position.y)
        except OSError as exc:
            self._fail(exc)

    def log(self, level, text):
        if self.active:
            try:
                self._logs.write(f"{datetime.now().astimezone().isoformat(timespec='milliseconds')} [{level}] {text}\n")
            except OSError as exc:
                self._fail(exc)

    def _point(self, kind, x, y):
        if not math.isfinite(x) or not math.isfinite(y):
            return
        self._path_counts[kind] += 1
        stride = self._strides[kind] or 1
        if (self._path_counts[kind] - 1) % stride:
            return
        points = self._paths.setdefault(kind, [])
        points.append((x, y))
        if len(points) >= 8192:
            self._paths[kind] = points[::2]
            self._strides[kind] = stride * 2

    def stop(self, reason="operator"):
        if not self.active:
            return self.directory
        self.active = False
        self._summary.update(stopped_at=datetime.now().astimezone().isoformat(), stop_reason=reason,
                             duration_s=(time.monotonic_ns() - self._start_ns) * 1e-9, counts=dict(self._counts))
        try:
            self._flush()
            if not self._data.tell():
                self._data.write(gzip.compress(b""))
            self._files.close()
            self._plot()
            self._summary["status"] = "finished"
            self._save_summary()
        except OSError as exc:
            self._fail(exc)
            raise
        return self.directory

    def _plot(self):
        image = QImage(840, 720, QImage.Format_RGB32)
        image.fill(QColor("white"))
        painter = QPainter(image)
        painter.setRenderHint(QPainter.Antialiasing)
        try:
            painter.setFont(QFont("sans-serif", 15))
            painter.drawText(54, 40, f"{self._summary['experiment']}   /   {self._summary['vehicle']}")
            painter.setFont(QFont("sans-serif", 10))
            # One metre has the same screen length on both axes.
            left, bottom, scale = 70, 644, 138
            box = QRectF(left, bottom - 4 * scale, 3 * scale, 4 * scale)
            for tick in range(9):
                y = bottom - tick * scale / 2
                painter.setPen(QColor("#d8dbe0"))
                painter.drawLine(QPointF(left, y), QPointF(box.right(), y))
                painter.setPen(QColor("#24282c"))
                painter.drawText(QPointF(left - 32, y + 4), f"{tick / 2:g}")
            for tick in range(7):
                x = left + tick * scale / 2
                painter.setPen(QColor("#d8dbe0"))
                painter.drawLine(QPointF(x, bottom), QPointF(x, box.top()))
                painter.setPen(QColor("#24282c"))
                painter.drawText(QPointF(x - 6, bottom + 22), f"{tick / 2:g}")
            painter.drawText(40, 77, "Y [m]")
            painter.drawText(241, 699, "X [m]")
            painter.drawText(526, 107, self._summary["started_at"][:19].replace("T", " "))
            painter.drawText(526, 135, f"Duration   {self._summary['duration_s']:.1f} s")
            y_legend = 190
            for kind, label, color in (("ref", "GNC reference", "#be451e"), ("ekf", "EKF", "#68628d"),
                                       ("vicon", "Vicon", "#0066cc"), ("arm_tcp", "Grasp point", "#007b70")):
                points = self._paths.get(kind)
                if not points:
                    continue
                pen = QPen(QColor(color), 2)
                if kind == "ref":
                    pen.setStyle(Qt.DashLine)
                painter.setPen(pen)
                painter.drawLine(526, y_legend - 4, 558, y_legend - 4)
                painter.drawText(569, y_legend, label)
                y_legend += 30
                painter.save()
                painter.setClipRect(box.adjusted(-1, -1, 1, 1))
                path = QPainterPath()
                for i, (x, y) in enumerate(points):
                    point = QPointF(left + x * scale, bottom - y * scale)
                    if i:
                        path.lineTo(point)
                    else:
                        path.moveTo(point)
                painter.drawPath(path)
                painter.setBrush(QColor(color))
                painter.drawEllipse(point, 3, 3)
                painter.restore()
            if not self._paths:
                painter.drawText(526, y_legend, "No XY samples received")
        finally:
            painter.end()
        if not image.save(str(self.directory / "trajectory.png"), "PNG"):
            raise OSError("trajectory.png를 저장할 수 없습니다.")
