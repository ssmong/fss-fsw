"""Planar arm feedback used by Mission Control. All angles are radians internally."""

import math


JOINTS = ("shoulder", "elbow", "wrist")
JOINT_LABELS = ("어깨", "팔꿈치", "손목")


def joint_angles(message):
    names, positions = message.name, message.position
    if len(names) != len(positions) or len(names) != len(set(names)):
        return None
    values = dict(zip(names, positions))
    if not all(name in values and math.isfinite(values[name]) for name in JOINTS):
        return None
    return tuple(values[name] for name in JOINTS)


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
