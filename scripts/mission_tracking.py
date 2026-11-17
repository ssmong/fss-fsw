"""Tracking of the registered spacecraft plan against timestamped executor feedback."""

import bisect
import json
import math
from pathlib import Path


_TRACKED_PHASES = ("LAUNCH", "COAST", "CAPTURING", "CLOSING")
_FINISH_PHASES = ("LOCAL_BRAKE", "PLANNED_BRAKE", "HOLDING", "STOPPING", "STOPPED", "MISSED", "FAULT")


def _finite(value):
    return type(value) in (float, int) and math.isfinite(value)


class _Nominal:
    def __init__(self, entry):
        import numpy as np

        self.key, self.plan_id, xy, times, *optional = entry
        self.entry = entry
        self.t = np.asarray(times, dtype=float)
        self.xy = np.asarray(xy, dtype=float)
        if (self.t.ndim != 1 or len(self.t) < 2 or self.xy.shape != (len(self.t), 2)
                or not np.isfinite(self.t).all() or not np.isfinite(self.xy).all()
                or not (np.diff(self.t) > 0).all()):
            raise ValueError("invalid nominal time grid or XY")
        self.yaw = self.joints = None
        if optional and optional[0] is not None:
            yaw = np.asarray(optional[0], dtype=float)
            if yaw.shape == self.t.shape and np.isfinite(yaw).all():
                self.yaw = np.unwrap(yaw) if np.all(np.abs(yaw) <= math.pi) else yaw
        if len(optional) > 1 and optional[1] is not None:
            joints = np.asarray(optional[1], dtype=float)
            if joints.shape == (len(self.t), 3) and np.isfinite(joints).all():
                # Legacy principal-angle series cross ±π. Preserve explicit multi-turn winding.
                principal = np.all(np.abs(joints) <= math.pi, axis=0)
                self.joints = np.where(principal, np.unwrap(joints, axis=0), joints)

    def compare(self, row, run, max_age):
        if not isinstance(row, dict) or row.get("phase") not in _TRACKED_PHASES:
            return None
        stamp, control, elapsed = (row.get(k) for k in ("sample_stamp", "control_stamp", "plan_time_s"))
        if not all(_finite(x) for x in (stamp, control, elapsed)):
            return None
        age = control - stamp
        # The executor records elapsed at control_stamp; q is stamped at sample_stamp.
        t = elapsed - age
        if stamp <= 0 or not 0 <= age <= max_age or not self.t[0] <= t <= self.t[-1]:
            return None
        q = row.get("q")
        if not isinstance(q, list) or len(q) not in (3, 6) or not all(_finite(v) for v in q[:2]):
            return None
        right = min(bisect.bisect_right(self.t, t), len(self.t) - 1)
        left = right - 1
        fraction = (t - self.t[left]) / (self.t[right] - self.t[left])

        def interpolate(series):
            return series[left] + fraction * (series[right] - series[left])

        output = dict(run_id=run, plan_key=self.key, plan_id=self.plan_id, phase=row["phase"],
                      sample_stamp_s=stamp, control_stamp_s=control, feedback_age_s=age, plan_time_s=t)
        for name, actual, nominal in zip(("x", "y"), q[:2], interpolate(self.xy)):
            output[f"actual_{name}_m"] = actual
            output[f"planned_{name}_m"] = float(nominal)
            output[f"error_{name}_m"] = float(actual - nominal)
        output["error_xy_m"] = math.hypot(output["error_x_m"], output["error_y_m"])

        measured_joints = q[3:] if len(q) == 6 else (None, None, None)
        planned_joints = interpolate(self.joints) if self.joints is not None else (None, None, None)
        yaw = interpolate(self.yaw) if self.yaw is not None else None
        for name, actual, nominal in zip(("yaw", "shoulder", "elbow", "wrist"), [q[2], *measured_joints],
                                         [yaw, *planned_joints]):
            actual = actual if _finite(actual) else None
            output[f"actual_{name}_rad"] = actual
            output[f"planned_{name}_rad"] = float(nominal) if nominal is not None else None
            delta = (actual - nominal) if actual is not None and nominal is not None else None
            output[f"error_{name}_rad"] = (float(delta) + math.pi) % math.tau - math.pi if delta is not None else None
        return output


def _registered_entry(entry, original, executed):
    """Move the loaded table-frame samples back through source and into this run's table."""
    import numpy as np

    key, plan_id, xy, times, *optional = entry
    _, a, offset = original
    _, b, target = executed
    angle = b - a
    c, s = math.cos(angle), math.sin(angle)
    xy = (np.asarray(xy) - offset) @ np.array([[c, s], [-s, c]]) + target
    if optional and optional[0] is not None:
        optional[0] = (np.asarray(optional[0]) + angle).tolist()
    experiment, method = key.rsplit("/", 1)
    return [f"{experiment} (실행 기록)/{method}", plan_id, xy.tolist(), times, *optional]


class PaperTracking:
    """Read newly recorded samples from the run identified by the live status topic."""

    def __init__(self, plans, signatures, joined_at):
        self.joined_at = joined_at
        self.latest_stamp = -math.inf
        self.set_catalog(plans, signatures)

    def set_catalog(self, plans, signatures):
        self.plans = plans
        self.signatures = signatures
        self.run = None
        self.plan = self.active_key = self.extra_plan = self.bound_extra = None
        self.offset = 0
        self.last_t = -math.inf

    def refresh_catalog(self, plans, signatures):
        """Refresh idle choices while retaining the bound run and its read cursor."""
        self.plans, self.signatures = plans, signatures
        if self.plan is None:
            return
        old = self.plan
        current = next((p for p in plans if p[0] == old.key), None)
        if current == old.entry:
            return
        if self.bound_extra is None:
            experiment, method = old.key.rsplit("/", 1)
            key = f"{experiment} (실행 기록)/{method}"
            while any(p[0] == key for p in plans):
                key = key.rsplit("/", 1)[0] + " (기록)/" + method
            self.bound_extra = [key, *old.entry[1:]]
        self.extra_plan = self.bound_extra
        self.active_key = self.bound_extra[0]

    def _bind(self, plan_id, meta, manifest):
        self.plan = self.active_key = self.extra_plan = None
        registration = meta.get("registration")
        if not isinstance(registration, dict):
            return
        digest = meta.get("plan_sha256")
        angle, offset = registration.get("rotation_rad"), registration.get("translation_m")
        if not _finite(angle) or not isinstance(offset, list) or len(offset) != 2 or not all(map(_finite, offset)):
            return
        signature = (digest, angle, offset)
        candidates = [p for p in self.plans if p[1] == plan_id and p[0] in self.signatures
                      and self.signatures[p[0]][0] == digest]
        if isinstance(manifest, str) and manifest:
            path = Path(manifest)
            if path.name != "trial.yaml" or not path.parent.parts or ".." in path.parts:
                return
            suffix = path.parent.parts[-2:]
            candidates = [p for p in candidates if tuple(p[0].split("/")[-len(suffix):]) == suffix]
        elif len(candidates) > 1:
            candidates = [p for p in candidates if self.signatures[p[0]] == signature]
        if len(candidates) != 1:
            return
        entry = candidates[0]
        original = self.signatures[entry[0]]
        try:
            if original != signature:
                entry = _registered_entry(entry, original, signature)
                if any(p[0] == entry[0] for p in self.plans):
                    return
                self.extra_plan = entry
            self.plan = _Nominal(entry)
            self.active_key = self.plan.key
        except (ImportError, ValueError, TypeError):
            self.extra_plan = None
        self.bound_extra = self.extra_plan

    def _initial_offset(self, stream, end):
        # Seek close to the first eligible timestamp when joining a large, ongoing log.
        # The lower bound always follows a complete record older than our connection.
        low, high = 0, end
        threshold = max(self.joined_at, self.latest_stamp)
        while high - low > 65536:
            middle = (low + high) // 2
            stream.seek(middle)
            stream.readline(65536)
            line = stream.readline(65536)
            try:
                stamp = json.loads(line)["control_stamp"]
            except (ValueError, KeyError, TypeError):
                break
            if not _finite(stamp) or not line.endswith(b"\n"):
                break
            if stamp < threshold:
                low = stream.tell()
            else:
                high = middle
        return low

    def poll(self, status, now, source):
        if not isinstance(status, dict) or status.get("frame_id") != "table" or not _finite(now):
            return []
        plan_id, phase = status.get("plan_id"), status.get("phase")
        if source is None or phase not in ("PREPARING", *_TRACKED_PHASES, *_FINISH_PHASES):
            self.extra_plan = self.active_key = None
            if not status.get("log_directory"):
                matches = [p[0] for p in self.plans if p[1] == plan_id]
                self.active_key = matches[0] if len(matches) == 1 else None
            return []
        run, samples, config = source
        experiment = config.get("experiment")
        if not isinstance(experiment, dict) or experiment.get("frame_id") != "table":
            self.active_key = self.extra_plan = None
            return []
        cap = experiment.get("feedback_timeout_s")
        if not _finite(cap) or cap <= 0:
            self.active_key = self.extra_plan = None
            return []
        identity = (run, plan_id)
        try:
            with samples.open("rb") as stream:
                end = stream.seek(0, 2)
                if self.run != identity:
                    self.bound_extra = None
                    meta = config["paper_plan"]
                    manifest = (meta.get("paper_trial_file") or meta.get("trial_file")
                                or experiment.get("paper_trial_file") or status.get("paper_trial_file"))
                    self._bind(plan_id, meta, manifest)
                    self.run = identity
                    self.last_t = -math.inf
                    self.offset = self._initial_offset(stream, end)
                if self.plan is None:
                    return []
                self.active_key = self.bound_extra[0] if self.bound_extra else self.plan.key
                self.extra_plan = self.bound_extra
                if end < self.offset:
                    self.offset = end
                    return []
                # Normal 5 Hz status batches contain roughly 20 records. Bound work on a slow or reconnected link.
                if self.offset == end:
                    return []
                stream.seek(self.offset)
                data = stream.read(min(end - self.offset, 512 * 1024))
                last = data.rfind(b"\n") + 1
                if not last:
                    if len(data) == 512 * 1024:
                        self.offset += len(data)  # an invalid oversized row cannot block future records
                    return []
                lines = data[:last].splitlines(keepends=True)[:128]
                self.offset += sum(map(len, lines))
        except OSError:
            return []

        results = []
        for line in lines:
            try:
                row = json.loads(line)
            except (UnicodeError, ValueError):
                continue
            sample = self.plan.compare(row, run, cap)
            if sample is None:
                continue
            stamp, t = sample["sample_stamp_s"], sample["plan_time_s"]
            if (stamp < self.joined_at or stamp <= self.latest_stamp or t <= self.last_t
                    or sample["control_stamp_s"] > now):
                continue
            results.append(sample)
            self.latest_stamp, self.last_t = stamp, t
        return results
