#!/usr/bin/env python3
"""rosbag2 -> HDF5 转换(数采episode -> 训练数据), 在SDK容器内运行

用法:
    python3 bag2hdf5.py <episode目录>          # 转换单个episode
    python3 bag2hdf5.py <recordings根目录>     # 递归转换所有episode

输出: 每个episode目录下生成 episode.h5
  /qpos            (N,7)  从臂关节角arm_joint1-6 + 夹爪arm_gripper (rad)
  /qpos_ts         (N,)   bag时间戳(s)
  /action_joint    (M,6)  /joint_position_controller/commands
  /action_joint_ts (M,)
  /action_gripper  (K,1)  /gripper_controller/commands
  /action_gripper_ts (K,)
  /end_pose        (P,7)  末端位姿 x y z qx qy qz qw
  /end_pose_ts     (P,)
  /image_jpg       (I,)   压缩图像原始字节(vlen uint8, 相机未开则无此组)
  /image_ts        (I,)
"""
import os
import sys

import h5py
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import JointState, CompressedImage
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped

ARM_JOINTS = [f"arm_joint{i}" for i in range(1, 7)] + ["arm_gripper"]


def bag_to_h5(ep_dir):
    meta = os.path.join(ep_dir, "metadata.yaml")
    if not os.path.exists(meta):
        print(f"跳过(非episode目录): {ep_dir}")
        return False
    if os.path.exists(os.path.join(ep_dir, "episode.h5")):
        return True                  # 已转换, 增量跳过
    # rosbag2存储后缀自动识别(mcap为容器内默认, 旧版本可能是sqlite3)
    if any(f.endswith(".mcap") for f in os.listdir(ep_dir)):
        storage = "mcap"
    elif any(f.endswith(".db3") for f in os.listdir(ep_dir)):
        storage = "sqlite3"
    else:
        print(f"跳过(无bag数据文件): {ep_dir}")
        return False

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=ep_dir, storage_id=storage),
        rosbag2_py.ConverterOptions("", ""))

    qpos, qpos_ts = [], []
    aj, aj_ts = [], []
    ag, ag_ts = [], []
    ep, ep_ts = [], []
    img, img_ts = [], []

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        if topic == "/joint_states":
            m = deserialize_message(data, JointState)
            try:
                idx = [m.name.index(n) for n in ARM_JOINTS]
                qpos.append([m.position[i] for i in idx])
                qpos_ts.append(t)
            except ValueError:
                pass
        elif topic == "/joint_position_controller/commands":
            m = deserialize_message(data, Float64MultiArray)
            if len(m.data) == 6:
                aj.append(list(m.data))
                aj_ts.append(t)
        elif topic == "/gripper_controller/commands":
            m = deserialize_message(data, Float64MultiArray)
            ag.append(list(m.data))
            ag_ts.append(t)
        elif topic == "/arm/end_pose":
            m = deserialize_message(data, PoseStamped)
            p, q = m.pose.position, m.pose.orientation
            ep.append([p.x, p.y, p.z, q.x, q.y, q.z, q.w])
            ep_ts.append(t)
        elif topic.endswith("image_compressed"):
            m = deserialize_message(data, CompressedImage)
            img.append(bytes(m.data))
            img_ts.append(t)

    dur = (qpos_ts[-1] - qpos_ts[0]) if len(qpos_ts) > 1 else 0.0
    # VLA任务标签: 桥接录制时写入的自然语言指令
    task = "demo"
    try:
        with open(os.path.join(ep_dir, "instruction.txt"),
                  encoding="utf-8") as tf:
            task = tf.read().strip() or "demo"
    except OSError:
        pass
    out = os.path.join(ep_dir, "episode.h5")
    with h5py.File(out, "w") as f:
        f.create_dataset("qpos", data=np.array(qpos))
        f.create_dataset("qpos_ts", data=np.array(qpos_ts))
        f.create_dataset("action_joint", data=np.array(aj))
        f.create_dataset("action_joint_ts", data=np.array(aj_ts))
        f.create_dataset("action_gripper", data=np.array(ag))
        f.create_dataset("action_gripper_ts", data=np.array(ag_ts))
        f.create_dataset("end_pose", data=np.array(ep))
        f.create_dataset("end_pose_ts", data=np.array(ep_ts))
        if img:
            dt = h5py.vlen_dtype(np.uint8)
            d = f.create_dataset("image_jpg", (len(img),), dtype=dt)
            for i, b in enumerate(img):
                d[i] = np.frombuffer(b, dtype=np.uint8)
            f.create_dataset("image_ts", data=np.array(img_ts))
        f.attrs["duration_s"] = dur
        f.attrs["task"] = task
        f.attrs["samples"] = (
            f"qpos:{len(qpos)} action_joint:{len(aj)} "
            f"action_gripper:{len(ag)} end_pose:{len(ep)} image:{len(img)}")
    print(f"{out}: qpos={len(qpos)} action={len(aj)}/{len(ag)} "
          f"pose={len(ep)} img={len(img)} 时长{dur:.1f}s")
    return True


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    path = sys.argv[1]
    if os.path.exists(os.path.join(path, "metadata.yaml")):
        bag_to_h5(path)                       # 单个episode
    else:                                     # 递归所有episode
        for root, dirs, _ in os.walk(path):
            for d in sorted(dirs):
                bag_to_h5(os.path.join(root, d))


if __name__ == "__main__":
    main()
