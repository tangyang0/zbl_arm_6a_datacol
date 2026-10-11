#!/usr/bin/env python3
"""episode.h5 -> LeRobot v3 数据集(wall-x训练格式), 宿主机pytorch环境运行

依赖: pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
      pip install "lerobot>=0.3" h5py

用法:
    python h5_to_lerobot.py <episode.h5 | 任务目录 | recordings根目录> \
        --repo-id zbl_pick_cube [--root ~/lerobot_data] [--fps 30] \
        [--action-mode ee|joints] [--trim 0.5] [--grip-max 1.70]

帧对齐: 有图像时以相机时间戳为主轴, qpos/action/end_pose按时间戳最近邻
取样; 无图像时以qpos时间戳降采样到--fps。
动作约定(同厂家x2robot2lerobot): 第i帧行动 = 第i+1帧动作, 末帧丢弃。
  ee模式:     action = [末端xyz(3), 姿态6D(6), 夹爪开度0~1(1)] = 10维
  joints模式: action = [6关节角, 夹爪开度0~1] = 7维
observation.state = [6关节角, 夹爪开度0~1] = 7维(与动作模式无关)
"""
import argparse
import glob
import os

import h5py
import numpy as np


def quat_to_6d(x, y, z, w):
    """四元数 -> 6D旋转表示(旋转矩阵前两行展平), 数值上与厂家转换一致"""
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return R[:2, :].flatten()


def nearest_indices(src_ts, dst_ts):
    """对每个dst_ts返回src中最近邻索引(要求src_ts升序)"""
    idx = np.searchsorted(src_ts, dst_ts)
    idx = np.clip(idx, 1, len(src_ts) - 1)
    left = src_ts[idx - 1]
    right = src_ts[idx]
    return np.where(dst_ts - left <= right - dst_ts, idx - 1, idx)


def load_episode(h5_path, args):
    """读取一个episode.h5, 返回按主轴对齐后的(state, action, images, task)"""
    with h5py.File(h5_path, "r") as f:
        qpos = f["qpos"][:]                  # (N,7) 关节+夹爪(rad)
        qpos_ts = f["qpos_ts"][:]
        task = f.attrs.get("task", "demo")
        end_pose = (f["end_pose"][:] if "end_pose" in f else None)
        end_ts = (f["end_pose_ts"][:] if "end_pose" in f else None)
        action_joint = (f["action_joint"][:] if "action_joint" in f
                        and len(f["action_joint"]) > 0 else None)
        action_grip = (f["action_gripper"][:] if "action_gripper" in f
                       and len(f["action_gripper"]) > 0 else None)
        ag_ts = (f["action_gripper_ts"][:] if action_grip is not None
                 and "action_gripper_ts" in f else None)
        has_img = "image_jpg" in f and len(f["image_jpg"]) > 0
        if has_img:
            from PIL import Image
            imgs = [np.asarray(Image.open(
                __import__("io").BytesIO(bytes(b))))
                for b in f["image_jpg"][:]]
            img_ts = f["image_ts"][:]

    if len(qpos) < 10:
        raise ValueError(f"数据太短: {h5_path}")

    # 主轴选择: 相机优先, 否则qpos降采样
    if has_img:
        axis_ts = img_ts
    else:
        t0, t1 = qpos_ts[0], qpos_ts[-1]
        axis_ts = np.arange(t0, t1, 1.0 / args.fps)
    # 掐头去尾(秒)
    if args.trim > 0:
        m = (axis_ts >= axis_ts[0] + args.trim) & \
            (axis_ts <= axis_ts[-1] - args.trim)
        axis_ts = axis_ts[m]
        if has_img:
            imgs = [im for im, k in zip(imgs, m) if k]
    rel_ts = axis_ts - axis_ts[0]                 # 相对时间戳(0起)

    qi = nearest_indices(qpos_ts, axis_ts)
    state = qpos[qi].copy()
    state[:, 6] = np.clip(state[:, 6] / args.grip_max, 0.0, 1.0)

    if args.action_mode == "joints":
        if action_joint is None:
            action_joint = qpos[:, :6]        # 干跑时以实际关节角兜底
        ai = nearest_indices(qpos_ts, axis_ts)
        act = action_joint[ai]
    else:                                     # ee模式
        if end_pose is None:
            raise ValueError("ee动作模式需要end_pose数据")
        ei = nearest_indices(end_ts, axis_ts)
        pose = end_pose[ei]
        rot6 = np.stack([quat_to_6d(*q) for q in pose[:, 3:7]])
        # 夹爪动作: 指令流优先(按时间戳最近邻), 否则用状态夹爪角
        if action_grip is not None and ag_ts is not None and len(ag_ts) > 1:
            grip = np.clip(
                action_grip[nearest_indices(ag_ts, axis_ts), 0]
                / args.grip_max, 0.0, 1.0)
        else:
            grip = np.clip(qpos[qi, 6] / args.grip_max, 0.0, 1.0)
        act = np.concatenate([pose[:, :3], rot6, grip[:, None]], axis=1)

    # 第i帧行动=第i+1帧动作(末帧丢弃)
    n = len(axis_ts) - 1
    return state[:n].astype(np.float32), act[1:].astype(np.float32), \
        (imgs[:n] if has_img else None), rel_ts[:n], task


def main():
    p = argparse.ArgumentParser()
    p.add_argument("src", help="episode.h5 / 任务目录 / recordings根目录")
    p.add_argument("--repo-id", required=True, help="数据集名(如 zbl_pick)")
    p.add_argument("--root", default=None, help="输出根目录(默认~/lerobot_data)")
    p.add_argument("--fps", type=float, default=30.0)
    p.add_argument("--action-mode", choices=["ee", "joints"], default="ee")
    p.add_argument("--trim", type=float, default=0.5,
                   help="掐头去尾秒数(0关闭, 默认0.5)")
    p.add_argument("--grip-max", type=float, default=1.70,
                   help="夹爪物理满开rad, 用于归一化0~1")
    args = p.parse_args()

    if os.path.isfile(args.src):
        h5_files = [args.src]
    else:
        h5_files = sorted(glob.glob(
            os.path.join(args.src, "**", "episode.h5"), recursive=True))
    if not h5_files:
        raise SystemExit(f"未找到episode.h5: {args.src}")

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    # root即数据集目录本身(lerobot约定, repo_id仅作标签)
    root = args.root or os.path.expanduser(
        os.path.join("~/lerobot_data", args.repo_id))
    feats = {
        "observation.state": {"dtype": "float32", "shape": (7,),
                              "names": None},
        "action": {"dtype": "float32",
                   "shape": (10 if args.action_mode == "ee" else 7,),
                   "names": None},
    }

    ds = None
    # 累积数据集: 已存在则追加(增量), 否则新建
    append_mode = os.path.exists(os.path.join(root, "meta", "info.json"))
    for h5p in h5_files:
        if os.path.exists(h5p + ".converted"):
            continue                  # 已转换过的episode, 增量跳过
        state, act, imgs, ts, task = load_episode(h5p, args)
        if imgs is not None and "observation.images.wrist" not in feats:
            h, w = imgs[0].shape[:2]
            feats["observation.images.wrist"] = {
                "dtype": "video", "shape": (h, w, 3),
                "names": ["height", "width", "channels"]}
        if ds is None:
            if append_mode:
                ds = LeRobotDataset(args.repo_id, root=root)
                ds.vcodec = "h264"   # 追加模式显式指定, 否则默认AV1多数播放器不兼容
                print(f"追加模式: {root}")
            else:
                ds = LeRobotDataset.create(
                    repo_id=args.repo_id, root=root, fps=int(args.fps),
                    features=feats, vcodec="h264")
        for i in range(len(state)):
            frame = {
                "observation.state": state[i],
                "action": act[i],
                "task": task,
            }
            if imgs is not None:
                frame["observation.images.wrist"] = imgs[i]  # (h,w,3) uint8
            ds.add_frame(frame)
        ds.save_episode()
        open(h5p + ".converted", "w").close()   # 增量标记
        print(f"{h5p}: {len(state)}帧 task={task!r} 已写入")
    if ds is not None:
        ds.finalize()
        print(f"数据集完成: {root}")
    else:
        print("没有新episode需要转换(均已转换或无数据)")


if __name__ == "__main__":
    main()
