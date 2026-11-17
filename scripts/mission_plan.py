"""Read spacecraft XY samples from registered self_tossing_hardware paper trials."""

import hashlib
import io
import json
import math
from pathlib import Path
import zipfile


def load_table_plans(root=None):
    """Return (experiment/method, plan ID, table XY, sample seconds) per bundle."""
    try:
        import numpy as np
        import yaml
    except ImportError:
        return []

    root = (Path(root) if root else Path.home() / "self_tossing_hardware" / "experiments").resolve()
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
                if any(a.ndim != 2 or len(a) < 2 or a.shape[1] != 9 for a in (push, flight)):
                    continue
                base = json.loads(setup.read_text())["base_pose"]
                if np.shape(base) != (3,) or not np.allclose(push[0, :3], base, atol=1e-6, rtol=0):
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
            c, s = np.cos(angle), np.sin(angle)
            xy = xy @ np.array([[c, s], [-s, c]]) + offset
            if not np.isfinite(xy).all() or not isinstance(trial["plan_id"], str) or not trial["plan_id"]:
                continue
            if len(xy) > 512:
                indices = np.linspace(0, len(xy) - 1, 512, dtype=int)
                xy, times = xy[indices], times[indices]
            plans.append((manifest.parent.relative_to(root).as_posix(), trial["plan_id"], xy.tolist(), times.tolist()))
        except (OSError, RuntimeError, KeyError, ValueError, TypeError, IndexError,
                EOFError, zipfile.BadZipFile, yaml.YAMLError):
            continue
    return plans


def paper_plan_elapsed(status, root=None):
    """Read the executor's plan clock from the last complete record of this run."""
    if status.get("phase") not in ("LAUNCH", "COAST", "CAPTURING", "CLOSING", "LOCAL_BRAKE",
                                   "PLANNED_BRAKE", "HOLDING", "STOPPING", "STOPPED", "MISSED", "FAULT"):
        return None
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
        trial = json.loads(config.read_bytes()).get("paper_plan")
        if not isinstance(trial, dict) or trial.get("plan_id") != status.get("plan_id"):
            return None
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
