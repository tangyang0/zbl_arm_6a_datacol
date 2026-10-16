#!/usr/bin/env python3
"""录制时直接写LeRobot v3数据集(wall-x训练格式), SDK容器内由master_bridge调用

数据流: bridge把各路原始流(统一perf_counter时间戳)喂进EpisodeBuffer,
episode结束时 write_episode(): 以相机时间戳为主轴(无相机按fps重采样)
-> 掐头去尾 -> 最近邻对齐 -> 第i帧行动=第i+1帧 -> 追加写入数据集+编码视频。
对齐与映射逻辑与 h5_to_lerobot.py 保持同源一致。
"""
import math
import os
import re
import subprocess

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


def task_dirname(task):
    """任务文本 -> 数据集子目录名(非法字符与空白转下划线, 截断80字符)"""
    name = re.sub(r"[^\w-]+", "_", task.strip())
    return name.strip("_")[:80] or "task"


def write_episode(buf, root, task, action_mode="ee", fps=20,
                  trim=0.5, grip_max=1.70):
    """把buffer写为数据集的一个episode(自动新建或追加), 返回帧数

    目录层级: <root>/<任务名>/ 为一个独立完整的LeRobot v3数据集
    (同一任务增量追加; 换任务自动另起目录), 落盘后全目录开放读写权限
    (容器uid与本机用户不同, 不chmod会在本机文件管理器里显示锁)。

    时间轴: 统一重采样到目标fps网格(与模型预训练帧率对齐, 默认20Hz),
    各路原始流(200Hz关节/位姿, 相机原生帧率)最近邻对齐到网格。
    状态: observation.state = 末端10维[x,y,z,rot6d,grip]
          (与wall-x预训练agent_pos同构: pos3+rotation_6D+gripper),
          observation.joints = 7维[6关节角+夹爪归一化](备用)。
    动作: 与状态同源(同一条从臂流, 官方x2robot2lerobot语义),
    第i帧行动=第i+1帧实测位姿, 即 action[i] ≡ state[i+1](末帧丢弃);
    ee模式为绝对末端位姿(训练侧loader自动转delta, 与预训练relative布局对齐)。
    """
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    state = np.array(buf.state)
    state_ts = np.array(buf.state_ts)
    pose = np.array(buf.pose) if buf.pose else None
    if pose is None:
        raise ValueError("缺少end_pose数据(末端状态必需)")
    pose_ts = np.array(buf.pose_ts)
    has_img = len(buf.imgs) > 0
    if has_img:
        from PIL import Image
        import io
        img_ts = np.array(buf.img_ts)
        imgs = [np.asarray(Image.open(io.BytesIO(b))) for b in buf.imgs]
        t0, t1 = img_ts[0], img_ts[-1]   # 相机是覆盖最窄的流, 用它定界
    else:
        t0, t1 = state_ts[0], state_ts[-1]
    axis_ts = np.arange(t0, t1, 1.0 / fps)
    if trim > 0:
        axis_ts = axis_ts[(axis_ts >= t0 + trim) & (axis_ts <= t1 - trim)]
    if len(axis_ts) < 10:
        raise ValueError("掐头去尾后数据过短")
    if has_img:
        imgs = [imgs[k] for k in nearest_indices(img_ts, axis_ts)]

    qi = nearest_indices(state_ts, axis_ts)
    joints = state[qi].copy()
    joints[:, 6] = np.clip(joints[:, 6] / grip_max, 0.0, 1.0)
    p = pose[nearest_indices(pose_ts, axis_ts)]
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
    n = len(axis_ts) - 1
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
    }
    # 任务层目录: <root>/<任务名>/ 本身即一个LeRobot数据集
    # (lerobot 0.4.4中create的root参数就是数据集目录本身, repo_id仅是标签)
    tdir = task_dirname(task)
    ds_dir = os.path.join(root, tdir)
    append_mode = os.path.exists(os.path.join(ds_dir, "meta", "info.json"))
    if append_mode:
        ds = LeRobotDataset(tdir, root=ds_dir)
        ds.vcodec = "h264"   # 追加模式不会沿用create时的编码器, 必须显式指定,
        # 否则默认libsvtav1(AV1), 多数播放器解不了
    else:
        if has_img:
            h, w = imgs[0].shape[:2]
            feats["observation.images.wrist"] = {
                "dtype": "video", "shape": (h, w, 3),
                "names": ["height", "width", "channels"]}
        ds = LeRobotDataset.create(
            repo_id=tdir, root=ds_dir, fps=int(fps),
            features=feats, vcodec="h264")

    for i in range(n):
        frame = {"observation.state": ee[i], "observation.joints": joints[i],
                 "action": act[i], "task": task}
        if has_img:
            frame["observation.images.wrist"] = imgs[i]
        ds.add_frame(frame)
    ds.save_episode()
    ds.finalize()
    # 去锁: 容器写入进程的uid与本机用户不同, 开放读写避免本机文件管理器显示锁
    subprocess.run(["chmod", "-R", "a+rwX", ds_dir], check=False)
    return n
