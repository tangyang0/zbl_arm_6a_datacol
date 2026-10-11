#!/usr/bin/env python3
"""录制时直接写LeRobot v3数据集(wall-x训练格式), SDK容器内由master_bridge调用

数据流: bridge把各路原始流(统一perf_counter时间戳)喂进EpisodeBuffer,
episode结束时 write_episode(): 以相机时间戳为主轴(无相机按fps重采样)
-> 掐头去尾 -> 最近邻对齐 -> 第i帧行动=第i+1帧 -> 追加写入数据集+编码视频。
对齐与映射逻辑与 h5_to_lerobot.py 保持同源一致。
"""
import math
import os

import numpy as np


class EpisodeBuffer:
    """一个episode的原始流缓冲(生产者: 桥接回调; 消费者: write_episode)"""

    def __init__(self):
        self.state, self.state_ts = [], []       # (7,) 6关节+夹爪(rad)
        self.aj, self.aj_ts = [], []             # (6,) 关节指令
        self.ag, self.ag_ts = [], []             # (1,) 夹爪指令
        self.pose, self.pose_ts = [], []         # (7,) 末端xyz+quat
        self.imgs, self.img_ts = [], []          # jpg原始字节

    def has_data(self):
        return len(self.state_ts) > 10


def nearest_indices(src_ts, dst_ts):
    idx = np.searchsorted(src_ts, dst_ts)
    idx = np.clip(idx, 1, len(src_ts) - 1)
    left = src_ts[idx - 1]
    right = src_ts[idx]
    return np.where(dst_ts - left <= right - dst_ts, idx - 1, idx)


def quat_to_6d(x, y, z, w):
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return R[:2, :].flatten()


def write_episode(buf, root, repo_id, task, action_mode="ee", fps=30,
                  trim=0.5, grip_max=1.70):
    """把buffer写为数据集的一个episode(自动新建或追加), 返回帧数"""
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    state = np.array(buf.state)
    state_ts = np.array(buf.state_ts)
    has_img = len(buf.imgs) > 0
    if has_img:
        from PIL import Image
        import io
        img_ts = np.array(buf.img_ts)
        imgs = [np.asarray(Image.open(io.BytesIO(b))) for b in buf.imgs]
        axis_ts = img_ts
    else:
        axis_ts = np.arange(state_ts[0], state_ts[-1], 1.0 / fps)
    if trim > 0:
        m = (axis_ts >= axis_ts[0] + trim) & (axis_ts <= axis_ts[-1] - trim)
        axis_ts = axis_ts[m]
        if has_img:
            imgs = [im for im, k in zip(imgs, m) if k]
    if len(axis_ts) < 10:
        raise ValueError("掐头去尾后数据过短")

    qi = nearest_indices(state_ts, axis_ts)
    st = state[qi].copy()
    st[:, 6] = np.clip(st[:, 6] / grip_max, 0.0, 1.0)

    if action_mode == "joints":
        aj = np.array(buf.aj) if buf.aj else state[:, :6]
        aj_ts = np.array(buf.aj_ts) if buf.aj else state_ts
        act = aj[nearest_indices(aj_ts, axis_ts)]
    else:
        pose = np.array(buf.pose) if buf.pose else None
        if pose is None:
            raise ValueError("ee动作模式需要end_pose数据")
        pose_ts = np.array(buf.pose_ts)
        p = pose[nearest_indices(pose_ts, axis_ts)]
        rot6 = np.stack([quat_to_6d(*q) for q in p[:, 3:7]])
        if buf.ag:
            ag = np.array(buf.ag)
            ag_ts = np.array(buf.ag_ts)
            grip = np.clip(
                ag[nearest_indices(ag_ts, axis_ts), 0] / grip_max, 0.0, 1.0)
        else:
            grip = st[:, 6]
        act = np.concatenate([p[:, :3], rot6, grip[:, None]], axis=1)

    # 第i帧行动=第i+1帧动作(末帧丢弃)
    n = len(axis_ts) - 1
    st, act = st[:n].astype(np.float32), act[1:].astype(np.float32)

    feats = {
        "observation.state": {"dtype": "float32", "shape": (7,),
                              "names": None},
        "action": {"dtype": "float32",
                   "shape": (10 if action_mode == "ee" else 7,),
                   "names": None},
    }
    append_mode = os.path.exists(os.path.join(root, "meta", "info.json"))
    if append_mode:
        ds = LeRobotDataset(repo_id, root=root)
        ds.vcodec = "h264"   # 追加模式不会沿用create时的编码器, 必须显式指定,
        # 否则默认libsvtav1(AV1), 多数播放器解不了
    else:
        if has_img:
            h, w = imgs[0].shape[:2]
            feats["observation.images.wrist"] = {
                "dtype": "video", "shape": (h, w, 3),
                "names": ["height", "width", "channels"]}
        ds = LeRobotDataset.create(
            repo_id=repo_id, root=root, fps=int(fps),
            features=feats, vcodec="h264")

    for i in range(n):
        frame = {"observation.state": st[i], "action": act[i], "task": task}
        if has_img:
            frame["observation.images.wrist"] = imgs[i]
        ds.add_frame(frame)
    ds.save_episode()
    ds.finalize()
    return n
