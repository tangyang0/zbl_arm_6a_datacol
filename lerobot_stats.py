"""Scoped LeRobot 0.4.4 statistics adapter; never edits installed packages."""
from contextlib import contextmanager
import inspect
import threading

import numpy as np


_STATS_LOCK = threading.RLock()


def check_lerobot_version():
    """Lightweight preflight, without importing torch/video modules."""
    from importlib.metadata import version
    installed = version("lerobot")
    if installed != "0.4.4":
        raise RuntimeError(
            f"Capture statistics adapter requires tested lerobot==0.4.4, got {installed}; "
            "validate the new API before recording."
        )


def safe_episode_stats(episode_data, features, quantile_list=None):
    """Skip acquisition/command diagnostics; promote pixels before squaring."""
    from lerobot.datasets.compute_stats import get_feature_stats, sample_images

    stats = {}
    for key, data in episode_data.items():
        spec = features[key]
        if (key.startswith(("acq.", "command.")) or key == "observation.master_ee_pose"
                or spec["dtype"] == "string"):
            continue
        image = spec["dtype"] in ("image", "video")
        values = sample_images(data) if image else np.asarray(data)
        values = values.astype(np.float64)
        if not np.isfinite(values).all():
            raise ValueError(f"Nonfinite data in {key}")
        item = get_feature_stats(
            values, axis=(0, 2, 3) if image else 0,
            keepdims=True if image else values.ndim == 1, quantile_list=quantile_list,
        )
        if image:
            item = {k: v if k == "count" else np.squeeze(v / 255.0, axis=0)
                    for k, v in item.items()}
        stats[key] = item
    return stats


@contextmanager
def safe_lerobot_statistics():
    """0.4.4 has no public stats callback; adapt this one save under a lock.

    The module function is restored even if encoding/save raises. This collector
    serializes all saves and disables parallel encoding; unrelated LeRobot users
    should not share the same process while this scoped adapter is active.
    """
    import lerobot.datasets.lerobot_dataset as module
    check_lerobot_version()
    if "compute_episode_stats" not in module.LeRobotDataset.save_episode.__code__.co_names:
        raise RuntimeError("Unsupported LeRobot save_episode statistics API")
    if list(inspect.signature(module.compute_episode_stats).parameters)[:2] != [
        "episode_data", "features"
    ]:
        raise RuntimeError("Unsupported LeRobot compute_episode_stats signature")
    with _STATS_LOCK:
        original = module.compute_episode_stats
        module.compute_episode_stats = safe_episode_stats
        try:
            yield
        finally:
            module.compute_episode_stats = original
