#!/usr/bin/env python3
"""LeRobot v3 capture with bounded alignment and per-frame acquisition columns."""
from datetime import datetime, timezone
import errno
import json
import math
import os
from pathlib import Path
import re
import subprocess
import warnings

import numpy as np

from acquisition import (
    ACQUISITION_SCHEMA_VERSION, AlignmentConfig, AlignmentError, EpisodeBuffer,
    align_episode, diagnostic_features, nearest_indices, schema_errors, write_json_atomic,
)
from lerobot_stats import safe_lerobot_statistics


def quat_to_6d(x, y, z, w):
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("invalid end-pose quaternion")
    x, y, z, w = x/norm, y/norm, z/norm, w/norm
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return R[:2, :].flatten()


def task_dirname(task):
    """任务文本 -> 数据集子目录名(非法字符与空白转下划线, 截断80字符)"""
    name = re.sub(r"[^\w-]+", "_", task.strip())
    return name.strip("_")[:80] or "task"


def acquisition_contract(fps, action_mode, grip_max, alignment, calibration_id=""):
    return {
        "schema_version": ACQUISITION_SCHEMA_VERSION, "fps": int(fps),
        "action_mode": action_mode, "gripper_max_rad": float(grip_max),
        "alignment_clock": alignment.clock,
        "header_clock_domain": alignment.header_clock_domain,
        "require_camera": alignment.require_camera,
        "calibration_id": calibration_id,
    }


def dataset_compatibility_errors(ds_dir, fps, action_mode, grip_max, alignment,
                                 calibration_id=""):
    """Read-only preflight, also called before recording starts."""
    ds_dir = Path(ds_dir)
    info_path = ds_dir / "meta/info.json"
    if not info_path.exists():
        if ds_dir.exists() and any(ds_dir.iterdir()):
            return ["目标目录存在但缺少有效meta/info.json；请检查上次写入或换--dataset"]
        return []
    try:
        info = json.loads(info_path.read_text())
        errors = schema_errors(info, fps, action_mode, alignment.require_camera)
        contract_path = ds_dir / "meta/acquisition/dataset_contract.json"
        if not contract_path.is_file():
            errors.append("缺少新版采集契约；旧数据请保留并换一个--dataset名称")
        else:
            actual = json.loads(contract_path.read_text())
            expected = acquisition_contract(fps, action_mode, grip_max, alignment, calibration_id)
            if actual != expected:
                errors.append(f"采集契约变化：已保存{actual}，当前{expected}；请换--dataset")
        if any((ds_dir / "meta/acquisition").glob("episode_*.pending.json")):
            errors.append("发现未完成的写入记录(.pending.json)，请检查数据后再追加")
        return errors
    except (OSError, ValueError, TypeError) as exc:
        return [f"无法验证数据集schema/采集契约: {exc}"]


def cleanup_empty_images(ds_dir, camera_keys=("observation.images.wrist",)):
    """Only remove empty parent directories after a completed episode save."""
    ds_dir = Path(ds_dir)
    if ds_dir.is_symlink():
        return []
    images = ds_dir / "images"
    removed = []
    if images.is_symlink():
        return removed
    for directory in [*(images / key for key in camera_keys), images]:
        if directory.is_symlink():
            continue
        try:
            directory.rmdir()
            removed.append(str(directory))
        except OSError as exc:
            if exc.errno not in (errno.ENOENT, errno.ENOTEMPTY, errno.EEXIST):
                warnings.warn(f"保留images目录，无法清理空目录{directory}: {exc}")
    return removed


def write_episode(buf, root, task, action_mode="ee", fps=20,
                  trim=0.5, grip_max=1.70, alignment=None, calibration_id=""):
    """Save a new-schema episode; refuse unsafe alignment or legacy-schema append.

    State/action semantics stay unchanged: absolute measured EE xyz+rotation6D+
    gripper fraction; action[i] is the next resampled measurement. acq.* values
    are diagnostics, never appended to the model's 10-D vectors.
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    alignment = alignment or AlignmentConfig()
    alignment.validate()
    if action_mode not in ("ee", "joints"):
        raise ValueError("action_mode must be ee or joints")
    if not math.isfinite(grip_max) or grip_max <= 0:
        raise ValueError("grip_max must be positive and finite")
    tdir = task_dirname(task)
    ds_dir = Path(root) / tdir
    errors = dataset_compatibility_errors(
        ds_dir, fps, action_mode, grip_max, alignment, calibration_id)
    if errors:
        raise ValueError("拒绝追加: " + "; ".join(errors))
    axis, matches, diagnostics, quality = align_episode(buf, fps, trim, alignment)
    state, pose = np.asarray(buf.state), np.asarray(buf.pose)
    if (state.shape != (len(buf.state), 7) or pose.shape != (len(buf.pose), 7)
            or not np.isfinite(state).all() or not np.isfinite(pose).all()):
        raise ValueError("joint/pose samples must be finite 7-D vectors")
    has_img = len(buf.imgs) > 0
    if has_img:
        from PIL import Image
        import io
        # Decode selected frames only. Explicit RGB avoids grayscale/RGBA schema drift.
        cache = {}
        imgs = []
        for index in matches["camera"]["source_index"][:-1]:
            if int(index) not in cache:
                with Image.open(io.BytesIO(buf.imgs[index])) as image:
                    cache[int(index)] = np.asarray(image.convert("RGB"))
            imgs.append(cache[int(index)])
        if len({image.shape for image in imgs}) != 1:
            raise ValueError("camera resolution changed within episode")
    existing_info = ds_dir / "meta/info.json"
    if existing_info.is_file():
        video_present = "observation.images.wrist" in json.loads(existing_info.read_text())["features"]
        if video_present != has_img:
            raise ValueError("camera presence differs from the existing dataset; use a new --dataset")
    qi = matches["joints"]["source_index"]
    joints = state[qi].copy()
    joints[:, 6] = np.clip(joints[:, 6] / grip_max, 0.0, 1.0)
    p = pose[matches["pose"]["source_index"]]
    rot6 = np.stack([quat_to_6d(*q) for q in p[:, 3:7]])
    ee = np.concatenate([p[:, :3], rot6, joints[:, 6:7]], axis=1)

    # 动作与状态同源(官方x2robot2lerobot语义: 同一条follow_*从臂流,
    # action[i] ≡ state[i+1]全维成立, 不用下发的指令流):
    # ee模式 = 末端位姿流 + 实测夹爪; joints模式 = 实测关节流整体
    if action_mode == "joints":
        act = joints.copy()
    else:
        act = np.concatenate([p[:, :3], rot6, joints[:, 6:7]], axis=1)

    # 第i帧行动=第i+1帧动作(末帧丢弃)
    n = len(axis) - 1
    ee, joints = ee[:n].astype(np.float32), joints[:n].astype(np.float32)
    act = act[1:].astype(np.float32)

    feats = {
        "observation.state": {"dtype": "float32", "shape": (10,),
                              "names": None},
        "observation.joints": {"dtype": "float32", "shape": (7,),
                               "names": None},
        "action": {"dtype": "float32",
                   "shape": (10 if action_mode == "ee" else 7,),
                   "names": None},
        **diagnostic_features(has_img),
    }
    # 任务层目录: <root>/<任务名>/ 本身即一个LeRobot数据集
    # (lerobot 0.4.4中create的root参数就是数据集目录本身, repo_id仅是标签)
    with safe_lerobot_statistics():
        if (ds_dir / "meta/info.json").exists():
            ds = LeRobotDataset(tdir, root=ds_dir)
            ds.vcodec = "h264"
            if has_img and tuple(ds.features["observation.images.wrist"]["shape"]) != imgs[0].shape:
                raise ValueError("camera resolution differs from the existing dataset")
        else:
            if has_img:
                h, w = imgs[0].shape[:2]
                feats["observation.images.wrist"] = {
                    "dtype": "video", "shape": (h, w, 3),
                    "names": ["height", "width", "channels"]}
            ds = LeRobotDataset.create(
                repo_id=tdir, root=ds_dir, fps=int(fps), features=feats, vcodec="h264")
        contract = acquisition_contract(fps, action_mode, grip_max, alignment, calibration_id)
        write_json_atomic(ds_dir / "meta/acquisition/dataset_contract.json", contract)
        episode_index = int(ds.meta.total_episodes)
        sidecar = ds_dir / f"meta/acquisition/episode_{episode_index:06d}.json"
        pending = sidecar.with_suffix(".pending.json")
        provenance = {
            **buf.metadata, "schema_version": ACQUISITION_SCHEMA_VERSION,
            "episode_index": episode_index, "task": task, "contract": contract,
            "alignment_report": quality, "status": "pending",
            "acq_columns_are_model_inputs": False, "acq_columns_have_normalization_stats": False,
            "image_statistics_adapter": "float64 before squaring; scoped lerobot 0.4.4 adapter",
        }
        write_json_atomic(pending, provenance)
        try:
            for i in range(n):
                frame = {"observation.state": ee[i], "observation.joints": joints[i],
                         "action": act[i], "task": task}
                frame.update({key: np.asarray([value[i]], dtype=np.int64)
                              for key, value in diagnostics.items()})
                if has_img:
                    frame["observation.images.wrist"] = imgs[i]
                ds.add_frame(frame)
            ds.save_episode(parallel_encoding=False)
            ds.finalize()
        except Exception:
            # Do not clean images or mark completion after a partial write.
            # Closing writers preserves diagnostic files for explicit recovery.
            try:
                ds.finalize()
            except Exception as close_error:
                warnings.warn(f"Failed closing partial dataset: {close_error}")
            raise
        provenance.update(status="complete", finalized_at_utc=datetime.now(timezone.utc).isoformat())
        write_json_atomic(sidecar, provenance)
        pending.unlink()
        cleanup_empty_images(ds_dir, ds.meta.video_keys)
    # 去锁: 容器写入进程的uid与本机用户不同, 开放读写避免本机文件管理器显示锁
    subprocess.run(["chmod", "-R", "a+rwX", str(ds_dir)], check=False)
    return n
