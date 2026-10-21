"""Capture provenance and bounded offline alignment; no ROS dependency."""
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import socket
import subprocess
import sys
import time

import numpy as np


ACQUISITION_SCHEMA_VERSION = 1
STREAMS = ("joints", "pose", "camera")
TOPICS = {
    "joints": "/joint_states",
    "pose": "/arm/end_pose",
    "camera": "/eye_in_hand/eye_in_hand/image_raw/image_compressed",
}


@dataclass(frozen=True)
class AlignmentConfig:
    clock: str = "receive"
    header_clock_domain: str = ""
    max_match_ms: float = 50.0
    max_age_ms: float = 100.0
    max_gap_ms: float = 250.0
    max_clock_skew_ms: float = 100.0
    require_camera: bool = True

    def validate(self):
        if self.clock not in ("receive", "header"):
            raise ValueError("alignment clock must be receive or header")
        if self.clock == "header" and not self.header_clock_domain.strip():
            raise ValueError("header alignment requires an explicit common header clock domain")
        for name in ("max_match_ms", "max_age_ms", "max_gap_ms", "max_clock_skew_ms"):
            value = getattr(self, name)
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be positive and finite")


class AlignmentError(ValueError):
    def __init__(self, message, report=None):
        super().__init__(message)
        self.report = report or {}


def alignment_from_args(args):
    cfg = AlignmentConfig(
        clock=args.sync_clock, header_clock_domain=args.header_clock_domain,
        max_match_ms=args.sync_max_match_ms, max_age_ms=args.sync_max_age_ms,
        max_gap_ms=args.sync_max_gap_ms, max_clock_skew_ms=args.sync_max_clock_skew_ms,
        require_camera=not args.allow_no_camera,
    )
    cfg.validate()
    return cfg


def header_values(msg):
    """Preserve integer ROS sec/nanosec without a float-seconds round trip."""
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    sec, nsec = getattr(stamp, "sec", 0), getattr(stamp, "nanosec", 0)
    valid = isinstance(sec, int) and isinstance(nsec, int) and sec >= 0 and 0 <= nsec < 10**9
    total = sec * 10**9 + nsec if valid else 0
    if total > np.iinfo(np.int64).max:
        total = 0
    return int(total), str(getattr(header, "frame_id", ""))


class EpisodeBuffer:
    """One episode, with values and original receipt/header provenance."""

    def __init__(self, metadata=None):
        self.state, self.state_ts = [], []
        self.pose, self.pose_ts = [], []
        self.imgs, self.img_ts = [], []
        self.aj, self.aj_ts = [], []  # retained for command diagnostics, not action labels
        self.ag, self.ag_ts = [], []
        self.trace = {s: {"header_ns": [], "receive_ns": [], "frame_id": []} for s in STREAMS}
        self.metadata = metadata or {}

    def append(self, stream, value, msg=None, receive_ns=None, header_ns=None, frame_id=None):
        receive_ns = time.monotonic_ns() if receive_ns is None else int(receive_ns)
        recorded_header, recorded_frame = header_values(msg)
        header_ns = recorded_header if header_ns is None else int(header_ns)
        frame_id = recorded_frame if frame_id is None else str(frame_id)
        values, times = {
            "joints": (self.state, self.state_ts),
            "pose": (self.pose, self.pose_ts),
            "camera": (self.imgs, self.img_ts),
        }[stream]
        values.append(value)
        times.append(receive_ns / 1e9)  # compatibility only; alignment uses integer trace
        self.trace[stream]["header_ns"].append(header_ns)
        self.trace[stream]["receive_ns"].append(receive_ns)
        self.trace[stream]["frame_id"].append(frame_id)

    def has_data(self):
        return len(self.state) > 10


def capture_provenance(args, repo_dir=None):
    repo = Path(repo_dir or Path(__file__).parent)
    def git(*arguments):
        try:
            return subprocess.check_output(
                ["git", "-C", str(repo), *arguments], text=True, stderr=subprocess.DEVNULL
            ).strip()
        except (OSError, subprocess.CalledProcessError):
            return None
    status = git("status", "--porcelain")
    hashes = {}
    for path in sorted(repo.glob("*.py")):
        hashes[path.name] = hashlib.sha256(path.read_bytes()).hexdigest()
    return {
        "schema_version": ACQUISITION_SCHEMA_VERSION,
        "recording_started_utc": datetime.now(timezone.utc).isoformat(),
        "host": socket.gethostname(),
        "git_sha": git("rev-parse", "HEAD"),
        "git_dirty": bool(status) if status is not None else None,
        "source_sha256": hashes,
        "runtime": {"python": sys.version, "numpy": np.__version__,
                    "lerobot_adapter_version": "0.4.4"},
        "actual_arguments": vars(args).copy(),
        "topics": TOPICS,
        "units": {"position": "m", "joint": "rad", "gripper": "measured_rad / gripper_max"},
        "state_layout": ["xyz", "rotation6d_first_two_matrix_rows", "gripper_fraction"],
        "raw_pose_layout": ["x", "y", "z", "qx", "qy", "qz", "qw"],
        "coordinate_transform_applied": None,
        "action_definition": "next resampled measured state; last observation dropped",
        "coordinate_calibration": {
            "id": getattr(args, "calibration_id", ""),
            "physical_axes_and_tool_zero_verified": False,
        },
    }


def diagnostic_features(has_camera=True):
    keys = ["acq.schema_version", "acq.grid_time_ns", "acq.action_grid_time_ns", "acq.valid"]
    names = ["joints", "pose", "action_joints", "action_pose"]
    if has_camera:
        names.append("camera")
    for stream in names:
        keys.extend(f"acq.{stream}.{field}" for field in (
            "header_time_ns", "receive_mono_ns", "source_index", "match_error_ns",
            "source_age_ns", "source_gap_ns", "valid",
        ))
    return {key: {"dtype": "int64", "shape": (1,), "names": None} for key in keys}


def schema_errors(info, fps, action_mode="ee", require_camera=True):
    """Fail before recording when appending would change the stored schema."""
    features = info.get("features", {})
    expected = {
        "observation.state": {"dtype": "float32", "shape": (10,)},
        "observation.joints": {"dtype": "float32", "shape": (7,)},
        "action": {"dtype": "float32", "shape": (10 if action_mode == "ee" else 7,)},
        **diagnostic_features(require_camera or "observation.images.wrist" in features),
    }
    errors = []
    for key, spec in expected.items():
        got = features.get(key, {})
        if got.get("dtype") != spec["dtype"] or tuple(got.get("shape", ())) != spec["shape"]:
            errors.append(f"{key}: expected {spec['dtype']}{spec['shape']}, got {got or 'missing'}")
    if {k for k in features if k.startswith("acq.")} != {
            k for k in expected if k.startswith("acq.")}:
        errors.append("acq diagnostic columns differ from the supported schema")
    if require_camera and features.get("observation.images.wrist", {}).get("dtype") != "video":
        errors.append("observation.images.wrist: video required")
    if float(info.get("fps", -1)) != float(fps):
        errors.append(f"fps: expected {fps}, got {info.get('fps')}")
    return errors


def nearest_indices(source, target):
    source = np.asarray(source)
    if len(source) == 0:
        raise ValueError("empty source stream")
    if len(source) == 1:
        return np.zeros(len(target), dtype=np.int64)
    right = np.clip(np.searchsorted(source, target), 1, len(source) - 1)
    left = right - 1
    return np.where(target - source[left] <= source[right] - target, left, right)


def align_episode(buf, fps=20, trim=0.5, config=None):
    """Reject invalid episodes; never bridge removed intervals into a fake timeline."""
    cfg = config or AlignmentConfig()
    cfg.validate()
    if not math.isfinite(fps) or fps <= 0 or fps != int(fps):
        raise AlignmentError("fps must be a positive integer")
    if not math.isfinite(trim) or trim < 0:
        raise AlignmentError("trim must be nonnegative and finite")
    counts = {"joints": len(buf.state), "pose": len(buf.pose), "camera": len(buf.imgs)}
    streams = ["joints", "pose"] + (["camera"] if counts["camera"] else [])
    if cfg.require_camera and not counts["camera"]:
        raise AlignmentError("missing camera stream; no VLA episode saved")
    report = {
        "alignment": asdict(cfg),
        "clock_domain": "host_monotonic" if cfg.clock == "receive" else cfg.header_clock_domain,
        "hardware_synchronization_established": False,
        "timing_limit": "receipt alignment is not exposure/state acquisition synchronization",
        "streams": {},
    }
    timestamps = {}
    offsets = {}
    for stream in streams:
        trace = buf.trace[stream]
        if counts[stream] < 2 or any(len(trace[k]) != counts[stream] for k in trace):
            raise AlignmentError(f"{stream}: missing values or complete timestamp provenance", report)
        receive = np.asarray(trace["receive_ns"], dtype=np.int64)
        header = np.asarray(trace["header_ns"], dtype=np.int64)
        if np.any(receive <= 0) or np.any(np.diff(receive) <= 0):
            raise AlignmentError(f"{stream}: receipt timestamps must be positive and strictly increasing", report)
        ids = sorted(set(trace["frame_id"]))
        if len(ids) > 1:
            raise AlignmentError(f"{stream}: frame_id changed within episode: {ids}", report)
        header_valid = bool(np.all(header > 0) and np.all(np.diff(header) > 0))
        # Positive but repeated/reversed stamps are evidence of stale/reset data,
        # even when the chosen alignment clock is receipt time.
        positive = header[header > 0]
        stale = int(np.count_nonzero(np.diff(positive) <= 0))
        report["streams"][stream] = {
            "sample_count": counts[stream], "frame_id": ids[0],
            "valid_header_stamps": header_valid, "missing_header_count": int(np.sum(header <= 0)),
            "nonincreasing_positive_header_count": stale,
        }
        if stale:
            raise AlignmentError(f"{stream}: repeated or reversed source header stamps", report)
        if cfg.clock == "header":
            if not header_valid:
                raise AlignmentError(f"{stream}: header alignment requires valid increasing source stamps", report)
            # Integer centering avoids losing ns precision at wall-clock magnitudes.
            offsets[stream] = int(np.median(receive - header))
            timestamps[stream] = header
        else:
            timestamps[stream] = receive
    if offsets and max(offsets.values()) - min(offsets.values()) > cfg.max_clock_skew_ms * 1e6:
        raise AlignmentError("declared header clocks have inconsistent receipt offsets", report)
    report["header_to_receive_median_offsets_ns"] = offsets
    # Intersection of all stream coverage, then trim both ends.
    start = max(int(t[0]) for t in timestamps.values()) + round(trim * 1e9)
    stop = min(int(t[-1]) for t in timestamps.values()) - round(trim * 1e9)
    if stop <= start:
        raise AlignmentError("streams have no common coverage after trimming", report)
    count = int((stop - start) * fps // 10**9) + 1
    axis = start + np.rint(np.arange(count, dtype=np.float64) * (1e9 / fps)).astype(np.int64)
    if len(axis) < 11:
        raise AlignmentError("fewer than 10 observation/action pairs after trimming", report)
    matches = {}
    all_valid = np.ones(len(axis), dtype=bool)
    for stream, ts in timestamps.items():
        indices = nearest_indices(ts, axis)
        error = ts[indices] - axis
        age = -error
        left = np.clip(np.searchsorted(ts, axis, side="right") - 1, 0, len(ts) - 1)
        right = np.minimum(left + 1, len(ts) - 1)
        gap = ts[right] - ts[left]
        valid = ((np.abs(error) <= cfg.max_match_ms * 1e6)
                 & (age <= cfg.max_age_ms * 1e6)
                 & (gap <= cfg.max_gap_ms * 1e6))
        all_valid &= valid
        matches[stream] = {
            "source_index": indices, "match_error_ns": error, "source_age_ns": age,
            "source_gap_ns": gap, "valid": valid.astype(np.int64),
            "header_time_ns": np.asarray(buf.trace[stream]["header_ns"], dtype=np.int64)[indices],
            "receive_mono_ns": np.asarray(buf.trace[stream]["receive_ns"], dtype=np.int64)[indices],
        }
        report["streams"][stream].update({
            "maximum_match_error_ms": float(np.max(np.abs(error)) / 1e6),
            "maximum_selected_gap_ms": float(np.max(gap) / 1e6),
            "invalid_grid_samples": int(np.count_nonzero(~valid)),
        })
    report.update({"grid_samples": len(axis), "saved_frames": len(axis) - 1,
                   "grid_start_ns": int(axis[0]), "grid_end_ns": int(axis[-1]),
                   "invalid_grid_indices": np.flatnonzero(~all_valid).tolist()})
    if not all_valid.all():
        raise AlignmentError("alignment quality limits failed; episode rejected", report)
    diagnostics = {
        "acq.schema_version": np.full(len(axis) - 1, ACQUISITION_SCHEMA_VERSION, dtype=np.int64),
        "acq.grid_time_ns": axis[:-1], "acq.action_grid_time_ns": axis[1:],
        "acq.valid": (all_valid[:-1] & all_valid[1:]).astype(np.int64),
    }
    for stream, values in matches.items():
        for key, value in values.items():
            diagnostics[f"acq.{stream}.{key}"] = value[:-1]
            if stream in ("pose", "joints"):
                diagnostics[f"acq.action_{stream}.{key}"] = value[1:]
    return axis, matches, diagnostics, report


def write_json_atomic(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n")
    temporary.replace(path)
