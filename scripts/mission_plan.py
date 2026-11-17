"""Read spacecraft body poses from registered self_tossing_hardware paper trials."""

import hashlib
import io
import json
import math
from pathlib import Path
import zipfile


def _plan_joint_offsets(model_path, np):
    if model_path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            model_path = Path(get_package_share_directory("hardware_runtime")) / "config/planar_model.json"
        except (ImportError, LookupError):
            model_path = Path.home() / "FSS_FSW/src/hardware/hardware_runtime/config/planar_model.json"
    try:
        model_path = Path(model_path)
        if model_path.stat().st_size > 65536:
            return None
        model = json.loads(model_path.read_bytes())
        source = model["provenance"]
        if (model["schema_version"] != 1 or source["urdf_commit"] != "1aa9864b55c8a3032085244ab4e365785faf354c"
                or source["urdf_sha256"] != "23ad70e02d040ea482776261baaae14fbf1de2fc1da3770742b2a54bbab449f3"):
            return None
        offsets = np.asarray(model["joint_offsets_m"], dtype=float)
        if offsets.shape == (3, 2) and np.isfinite(offsets).all():
            return offsets.tolist()
    except (OSError, TypeError, KeyError, ValueError):
        pass
    return None


def _matches_vector(values, actual, tolerance):
    return (isinstance(values, list) and len(values) == len(actual)
            and all(type(value) in (int, float) and math.isclose(value, sample, rel_tol=0, abs_tol=tolerance)
                    for value, sample in zip(values, actual)))


def load_table_plans(root=None, model_path=None, *, signatures=None):
    """Return key, ID, table XY, seconds, body yaw, relative joints, and arm geometry."""
    try:
        import numpy as np
        import yaml
    except ImportError:
        return []

    root = (Path(root) if root else Path.home() / "self_tossing_hardware" / "experiments").resolve()
    joint_offsets = _plan_joint_offsets(model_path, np)
    plans = []
    for manifest in sorted(root.glob("*/*/trial.yaml")):
        try:
            setup = manifest.with_name("initial_setup.json")
            archive = manifest.with_name("plan.npz")
            if any(not f.resolve().is_relative_to(root) or f.stat().st_size > limit
                   for f, limit in ((manifest, 65536), (setup, 65536), (archive, 2_000_000))):
                continue
            trial = yaml.safe_load(manifest.read_text())
            registration = trial["registration"]
            angle = float(registration["rotation_rad"])
            offset = np.asarray(registration["translation_m"], dtype=float)
            if offset.shape != (2,) or not np.isfinite([angle, *offset]).all():
                continue
            blob = archive.read_bytes()
            if hashlib.sha256(blob).hexdigest() != trial["plan_sha256"]:
                continue
            with zipfile.ZipFile(io.BytesIO(blob)) as z:
                if sum(f.file_size for f in z.infolist()) > 8_000_000:
                    continue
            with np.load(io.BytesIO(blob), allow_pickle=False) as plan:
                if str(plan["frame"].item()) != "fss_sim_1aa9864" or list(plan["phase_names"]) != ["push", "flight"]:
                    continue
                push, flight = plan["X_push"], plan["X_flight"]
                if any(a.ndim != 2 or len(a) < 2 or a.shape[1] not in (2, 9) for a in (push, flight)):
                    continue
                initial = json.loads(setup.read_text())
                base = initial["base_pose"]
                if np.shape(base) not in ((2,), (3,)) or not np.allclose(push[0, :2], base[:2], atol=1e-6, rtol=0):
                    continue
                durations = np.asarray(plan["T"], dtype=float)
                if durations.shape != (2,) or not np.isfinite(durations).all() or np.any(durations <= 0):
                    continue
                if not np.allclose(push[-1, :2], flight[0, :2], atol=1e-6, rtol=0):
                    continue
                # Both shooting grids include the shared release node.
                times = np.r_[np.linspace(0, durations[0], len(push)),
                              np.linspace(durations[0], durations.sum(), len(flight))[1:]]
                xy = np.vstack((push[:, :2], flight[1:, :2]))
                yaw = None
                joints = None
                geometry = None
                if push.shape[1] == flight.shape[1] == 9:
                    start = initial.get("joint_rad")
                    joint_samples = np.vstack((push[:, 3:6], flight[1:, 3:6]))
                    if (np.isfinite(joint_samples).all() and _matches_vector(start, push[0, 3:6], 1e-6)
                            and np.allclose(push[-1, 3:6], flight[0, 3:6], atol=1e-6, rtol=0)):
                        # The export already uses calibrated relative shoulder, elbow, wrist radians.
                        joints = joint_samples
                        if joint_offsets is not None:
                            geometry = [joint_offsets, None]
                            expected = trial.get("expected_grasp_offset_m")
                            grasp = plan.get("grasp_offset_m")
                            if (np.shape(grasp) == (2,) and grasp.dtype.kind in "ifu" and np.isfinite(grasp).all()
                                    and _matches_vector(expected, grasp, 1e-10)):
                                geometry[1] = grasp.tolist()
                if push.shape[1] == flight.shape[1] == 9 and len(base) == 3:
                    # State columns start with body x, y, yaw, then the three joint angles.
                    headings = np.r_[push[:, 2], flight[:, 2]]
                    if np.isfinite(headings).all() and math.isfinite(base[2]):
                        if abs(math.remainder(push[0, 2] - base[2], math.tau)) > 1e-6:
                            continue
                        if abs(math.remainder(push[-1, 2] - flight[0, 2], math.tau)) <= 1e-6:
                            registered = np.r_[push[:, 2], flight[1:, 2]] + angle
                            yaw = np.unwrap(np.arctan2(np.sin(registered), np.cos(registered)))
            c, s = np.cos(angle), np.sin(angle)
            xy = xy @ np.array([[c, s], [-s, c]]) + offset
            if not np.isfinite(xy).all() or not isinstance(trial["plan_id"], str) or not trial["plan_id"]:
                continue
            if len(xy) > 512:
                indices = np.linspace(0, len(xy) - 1, 512, dtype=int)
                xy, times = xy[indices], times[indices]
                if yaw is not None:
                    yaw = yaw[indices]
                if joints is not None:
                    joints = joints[indices]
            key = manifest.parent.relative_to(root).as_posix()
            plans.append((key, trial["plan_id"], xy.tolist(),
                          times.tolist(), yaw.tolist() if yaw is not None else None,
                          joints.tolist() if joints is not None else None, geometry))
            if signatures is not None:
                signatures[key] = (trial["plan_sha256"], angle, offset.tolist())
        except (OSError, RuntimeError, KeyError, ValueError, TypeError, IndexError,
                EOFError, zipfile.BadZipFile, yaml.YAMLError):
            continue
    return plans


def paper_run_files(status, root=None):
    directory = status.get("log_directory")
    if not isinstance(directory, str) or not Path(directory).is_absolute():
        return None
    try:
        root = (Path(root) if root else Path.home() / "self_tossing_hardware" / "outputs").resolve()
        run = Path(directory).resolve()
        if not run.is_relative_to(root) or run == root:
            return None
        config, samples = run / "configuration.json", run / "samples.jsonl"
        if any(p.resolve().parent != run or not p.is_file() for p in (config, samples)):
            return None
        if config.stat().st_size > 262144:
            return None
        metadata = json.loads(config.read_bytes())
        trial = metadata.get("paper_plan")
        if not isinstance(trial, dict) or trial.get("plan_id") != status.get("plan_id"):
            return None
        return run.relative_to(root).as_posix(), samples, metadata
    except (OSError, RuntimeError, ValueError, TypeError, AttributeError):
        return None


def paper_plan_elapsed(status, root=None, *, source=None):
    """Read the executor's plan clock from the last complete record of this run."""
    if status.get("phase") not in ("LAUNCH", "COAST", "CAPTURING", "CLOSING", "LOCAL_BRAKE",
                                   "PLANNED_BRAKE", "HOLDING", "STOPPING", "STOPPED", "MISSED", "FAULT"):
        return None
    if source is None:
        source = paper_run_files(status, root)
    if source is None:
        return None
    _, samples, _ = source
    try:
        with samples.open("rb") as stream:
            offset = max(0, stream.seek(0, 2) - 65536)
            stream.seek(offset)
            lines = stream.read(65536).split(b"\n")
        # The asynchronous writer can leave a partial row at either end of this window.
        complete = lines[1 if offset else 0:-1]
        if complete:
            elapsed = json.loads(complete[-1])["plan_time_s"]
            if type(elapsed) in (int, float) and math.isfinite(elapsed) and elapsed >= 0:
                return elapsed
    except (OSError, RuntimeError, KeyError, ValueError, TypeError, AttributeError):
        pass
    return None
