"""Per-vehicle Mission Control experiment archives on the ground station."""

from __future__ import annotations

from collections import Counter
from contextlib import ExitStack
import csv
from datetime import datetime
import gzip
import json
import math
from numbers import Real
from pathlib import Path, PureWindowsPath
import re
import time
from types import SimpleNamespace

from PySide6.QtCore import QPointF, QRectF, Qt
from PySide6.QtGui import QColor, QFont, QImage, QPainter, QPainterPath, QPen
from mission_video import VideoRecorder


TRACKING_IDS = ("run_id", "plan_key", "plan_id", "phase")
TRACKING_VALUES = (
    "sample_stamp_s", "control_stamp_s", "feedback_age_s", "plan_time_s",
    "actual_x_m", "actual_y_m", "actual_yaw_rad", "planned_x_m", "planned_y_m", "planned_yaw_rad",
    "error_x_m", "error_y_m", "error_xy_m", "error_yaw_rad",
    *(f"{source}_{joint}_rad" for joint in ("shoulder", "elbow", "wrist")
      for source in ("actual", "planned", "error")),
)
TRACKING_COLUMNS = ("received_unix_ns", "elapsed_s", *TRACKING_IDS, *TRACKING_VALUES)
POSE_COLUMNS = ("received_unix_ns", "elapsed_s", "sample_stamp_ns", "x_m", "y_m", "yaw_rad")


def _tracking_sample(value):
    if isinstance(value, SimpleNamespace):
        value = vars(value)
    if not isinstance(value, dict):
        return None
    result = {name: value.get(name) for name in TRACKING_IDS}
    for name in TRACKING_VALUES:
        v = value.get(name)
        result[name] = v if isinstance(v, Real) and not isinstance(v, bool) and math.isfinite(v) else None
    required = ("sample_stamp_s", "control_stamp_s", "plan_time_s", "actual_x_m", "actual_y_m",
                "planned_x_m", "planned_y_m", "error_x_m", "error_y_m", "error_xy_m")
    if any(result[name] is None for name in required):
        return None
    return result


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


def _save_session(directory, summary, *, preserve_memo=True):
    # Video finalization may arrive after the operator saved a memo.
    session = directory / "session.json"
    if preserve_memo and session.is_file():
        existing = json.loads(session.read_text(encoding="utf-8"))
        if "memo" in existing:
            summary["memo"] = existing["memo"]
    tmp = directory / "session.tmp"
    tmp.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(directory / "session.json")


def experiment_method(plan_id):
    match = re.search(r"(?:^|[-/])(Proposed|Nominal|JF_only)(?:$|[-/])", str(plan_id or ""))
    return match.group(1) if match else None


def save_experiment_memo(directory, memo):
    directory = Path(directory)
    summary = json.loads((directory / "session.json").read_text(encoding="utf-8"))
    summary["memo"] = memo
    _save_session(directory, summary, preserve_memo=False)


class ExperimentRecorder:
    def __init__(self, context=None):
        self.context = context
        self.directory = None
        self.active = False
        self.error = ""
        self.video_error = ""
        self._videos = []

    def start(self, root, name, vehicle):
        if self.active:
            raise ValueError("이 기체의 실험을 이미 기록 중입니다.")
        now = datetime.now().astimezone()
        self.directory = _run_directory(root, name, now)
        self._files = ExitStack()
        try:
            self._data = self._files.enter_context((self.directory / "telemetry.jsonl.gz").open("xb"))
            self._logs = self._files.enter_context((self.directory / "logs.txt").open("x", encoding="utf-8", buffering=1))
            tracking_file = self._files.enter_context((self.directory / "tracking_error.csv").open(
                "x", encoding="utf-8", newline="", buffering=1))
            self._tracking = csv.DictWriter(tracking_file, fieldnames=TRACKING_COLUMNS)
            self._tracking.writeheader()
            pose_file = self._files.enter_context((self.directory / "pose.csv").open(
                "x", encoding="utf-8", newline="", buffering=1))
            self._pose = csv.DictWriter(pose_file, fieldnames=POSE_COLUMNS)
            self._pose.writeheader()
            self._buffer = bytearray()
            self._start_ns = time.monotonic_ns()
            self._last_flush_ns = 0
            self._counts = Counter()
            self._paths = {}
            self._path_counts = Counter()
            self._strides = Counter()
            self._summary = {"format_version": 2, "experiment": name.strip() or "default", "vehicle": vehicle,
                             "started_at": now.isoformat(), "status": "recording", "pose_source": "vicon"}
            context = self.context() if self.context else {}
            self._summary["method"] = experiment_method((context.get("source_run") or {}).get("plan_id"))
            self._video = None
            self.video_error = ""
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
        _save_session(self.directory, self._summary)

    def capture(self, snapshot):
        if not self.active or self.video_error:
            return
        elapsed = (time.monotonic_ns() - self._start_ns) * 1e-9
        try:
            if self._video is None:
                video = self._video = VideoRecorder(self.directory, fps=10)
                self._videos = [v for v in self._videos if not v.finished] + [video]
                directory, summary = self.directory, self._summary
                summary["video"] = {"file": "trajectory.mp4", "status": "recording"}
                video.finalized.connect(lambda *args: self._video_done(video, directory, summary))
                if video.finished:
                    self._video_done(video, directory, summary)
            self._video.capture(snapshot, elapsed)
        except (ImportError, OSError, RuntimeError) as exc:
            self.video_error = f"궤적 동영상 저장 오류 · {exc}"
            self._summary["video"] = {"file": "trajectory.mp4", "status": "error", "error": str(exc)}
            self.log("ERROR", self.video_error)
            try:
                self._save_summary()
            except OSError:
                pass

    def _video_done(self, video, directory, summary):
        summary["video"] = {"file": "trajectory.mp4", "status": "error" if video.error else "finished",
                            "frames": video.frames_written, "dropped_frames": video.frames_dropped,
                            "error": video.error}
        if video.error and directory == self.directory:
            self.video_error = f"궤적 동영상 저장 오류 · {video.error}"
            self.log("ERROR", self.video_error)
        try:
            _save_session(directory, summary)
        except OSError as exc:
            if directory == self.directory:
                self.video_error = f"궤적 동영상 상태 저장 오류 · {exc}"

    def finish_video(self, timeout=16.0):
        """Drain encoder output when closing the GUI, without stopping any FSS node."""
        from PySide6.QtCore import QCoreApplication
        deadline = time.monotonic() + timeout
        for video in self._videos:
            video.stop()
        while any(not video.finished for video in self._videos) and time.monotonic() < deadline:
            QCoreApplication.processEvents()
            time.sleep(.005)
        return all(video.finished for video in self._videos)

    def _flush(self):
        if self._buffer:
            # Each chunk is a complete gzip member, so earlier chunks survive a process crash.
            self._data.write(gzip.compress(self._buffer, compresslevel=1, mtime=0))
            self._data.flush()
            self._buffer.clear()

    def _fail(self, exc):
        self.active = False
        self.error = f"실험 기록 오류 · {exc}"
        if self._video is not None:
            self._video.stop()
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
        if not self.active or kind == "ekf":
            return
        if kind == "vicon":
            kind = "pose"
        now = time.monotonic_ns()
        try:
            row = {"received_unix_ns": time.time_ns(), "elapsed_s": (now - self._start_ns) * 1e-9,
                   "kind": kind, "data": value}
            if kind == "pose":
                header = getattr(value, "header", None)
                stamp = getattr(header, "stamp", None)
                self._pose.writerow({"received_unix_ns": row["received_unix_ns"], "elapsed_s": row["elapsed_s"],
                                     "sample_stamp_ns": stamp.sec * 10**9 + stamp.nanosec if stamp else None,
                                     "x_m": value.x, "y_m": value.y, "yaw_rad": value.theta})
            if kind == "tracking_error":
                sample = _tracking_sample(value)
                if sample is None:
                    return
                row["data"] = sample
                self._tracking.writerow({"received_unix_ns": row["received_unix_ns"],
                                         "elapsed_s": row["elapsed_s"], **sample})
            if kind == "paper_status":
                status = vars(value) if isinstance(value, SimpleNamespace) else value
                method = experiment_method(status.get("plan_id")) if isinstance(status, dict) else None
                if method and method != self._summary.get("method"):
                    self._summary["method"] = method
                    self._save_summary()
                tuning = status.get("tuning") if isinstance(status, dict) else None
                if isinstance(tuning, dict) and tuning != self._summary.get("tuning"):
                    self._summary["tuning"] = tuning
                    (self.directory / "tuning.json").write_text(_json(tuning), encoding="utf-8")
                    self._save_summary()
            self._buffer.extend((_json(row) + "\n").encode("utf-8"))
            self._counts[kind] += 1
            if now - self._last_flush_ns >= 250_000_000 or len(self._buffer) >= 262144:
                self._flush()
                self._last_flush_ns = now
            if kind == "pose":
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
        if self._video is not None:
            self._video.stop()
            if not self._video.finished:
                self._summary["video"]["status"] = "encoding"
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
            for kind, label, color in (("ref", "Target", "#be451e"),
                                       ("pose", "Measured", "#0066cc"), ("arm_tcp", "Grasp point", "#007b70")):
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
