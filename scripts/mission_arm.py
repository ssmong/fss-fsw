"""Planar arm feedback used by Mission Control. All angles are radians internally."""

import math
import time
from bisect import bisect_left
from collections import deque


JOINTS = ("shoulder", "elbow", "wrist")
JOINT_LABELS = ("어깨", "팔꿈치", "손목")


class ViconArmOverlay:
    """Align measured base-frame links with Vicon in ROS time for display only."""

    def __init__(self):
        self.config = None
        self.vicon = deque(maxlen=64)
        self.links = deque(maxlen=16)

    def configure(self, values):
        self.config = None
        self.vicon.clear()
        self.links.clear()
        if (isinstance(values, (list, tuple)) and len(values) == 6
                and all(type(v) in (int, float) and math.isfinite(v) for v in values)):
            x, y, yaw, delay_ms, skew, timeout = values
            if delay_ms >= 0 and skew > 0 and timeout > 0:
                self.config = x, y, yaw, delay_ms / 1000, skew, min(timeout, 0.5)

    def add(self, frame, header, sample):
        if self.config is None:
            return
        buffer = self.vicon if frame == "table" else self.links
        stamp = getattr(header, "stamp", None)
        if not stamp or getattr(header, "frame_id", None) != frame:
            return
        t = stamp.sec + stamp.nanosec * 1e-9
        if buffer is self.vicon:
            t -= self.config[3]
        if buffer and t < buffer[-1][0] - self.config[5]:
            self.vicon.clear()
            self.links.clear()
        if not buffer or t > buffer[-1][0]:
            buffer.append((t, time.monotonic(), sample))

    def table_shape(self):
        if self.config is None or not self.vicon or not self.links:
            return None
        ox, oy, angle_offset, _, max_skew, timeout = self.config
        now = time.monotonic()
        # Only interpolate between measured poses. A faster encoder stream can
        # be ahead of the camera, so consider the most recent overlapping sample.
        for t, received, shape in reversed(self.links):
            if now - received > timeout or t < max(self.vicon[-1][0], self.links[-1][0]) - timeout:
                break
            i = bisect_left(self.vicon, t, key=lambda sample: sample[0])
            if i == len(self.vicon):
                continue
            right = self.vicon[i]
            if i == 0 and right[0] != t:
                break
            left = right if right[0] == t else self.vicon[i - 1]
            span = right[0] - left[0]
            if span > max_skew or min(left[1], right[1]) < now - timeout:
                continue
            ratio = (t - left[0]) / span if span else 0
            x, y, yaw = left[2]
            nx, ny, nyaw = right[2]
            x += ratio * (nx - x)
            y += ratio * (ny - y)
            yaw += ratio * math.remainder(nyaw - yaw, math.tau)
            c, s = math.cos(yaw), math.sin(yaw)
            x += c * ox - s * oy
            y += s * ox + c * oy
            yaw += angle_offset
            c, s = math.cos(yaw), math.sin(yaw)
            points, wrist = shape
            return tuple((x + c * px - s * py, y + s * px + c * py) for px, py in points), yaw + wrist
        return None


def joint_angles(message):
    names, positions = message.name, message.position
    if len(names) != len(positions) or len(names) != len(set(names)):
        return None
    values = dict(zip(names, positions))
    if not all(name in values and math.isfinite(values[name]) for name in JOINTS):
        return None
    return tuple(values[name] for name in JOINTS)


def pose_yaw_deg(q):
    components = tuple(getattr(q, key, None) for key in ("x", "y", "z", "w"))
    if any(v is None or not math.isfinite(v) for v in components):
        return None
    x, y, z, w = components
    if not math.isclose(math.hypot(x, y, z, w), 1.0, abs_tol=1e-3):
        return None
    # The end effector's +X axis projected onto the table.
    forward_x, forward_y = w*w + x*x - y*y - z*z, 2 * (x*y + w*z)
    if math.hypot(forward_x, forward_y) > 1e-8:
        return math.degrees(math.atan2(forward_y, forward_x))
    return None


def link_points(message, frame):
    # hardware_runtime publishes [base_link, link1, link2, gripper_link].
    if message.header.frame_id != frame or len(message.poses) != 4:
        return None
    points = tuple((p.position.x, p.position.y) for p in message.poses)
    q = message.poses[-1].orientation
    if not all(math.isfinite(v) for xy in points for v in xy):
        return None
    if not all(math.isfinite(v) for v in (q.z, q.w)) or math.hypot(q.z, q.w) < 1e-9:
        return None
    return points, 2 * math.atan2(q.z, q.w)


def encoder_model(angles):
    # docs/hardware/014_Mass_Properties.md, joint origins in the base frame (m).
    x, y = 0.190250, -0.000250
    points = [(0.0, 0.0), (x, y)]
    shoulder, elbow, wrist = angles
    for heading in (shoulder, shoulder + elbow):
        x += 0.199021 * math.cos(heading)
        y += 0.199021 * math.sin(heading)
        points.append((x, y))
    return points, shoulder + elbow + wrist


def plan_arm_points(joints, offsets, grasp_offset=None):
    """Base-frame XY in metres, ordered base, shoulder, elbow, wrist, optional grasp.

    Each offset is expressed in its parent link's frame. Relative joint angles
    turn about +Z after reaching that joint. The plan supplies its own geometry.
    """
    try:
        if len(joints) != 3 or len(offsets) != 3 or not all(math.isfinite(q) for q in joints):
            return None
        segments = tuple(offsets) if grasp_offset is None else (*offsets, grasp_offset)
        points = [(0.0, 0.0)]
        x = y = heading = 0.0
        for turn, (dx, dy) in zip((0.0, *joints), segments):
            heading += turn
            c, s = math.cos(heading), math.sin(heading)
            x += c * dx - s * dy
            y += s * dx + c * dy
            if not math.isfinite(x) or not math.isfinite(y):
                return None
            points.append((x, y))
        return tuple(points)
    except (TypeError, ValueError, OverflowError):
        return None


def feedback_source(status):
    fields = dict(part.strip().split("=", 1) for part in status.split(";") if "=" in part)
    return {"True": "모의", "False": "엔코더"}.get(fields.get("dry_run"), "출처 미확인")
