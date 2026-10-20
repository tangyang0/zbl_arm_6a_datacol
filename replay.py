#!/usr/bin/env python3
"""数据集episode回放到从臂 —— 数据质检用(眼见为实)

回放内容: observation.joints(6关节+夹爪开度0~1) = 录制时从臂的真实轨迹,
夹爪按 开度x--grip-max(1.70rad) 下发。默认干跑只打印摘要, --engage 才真动臂。

用法(容器内, SDK已launch, 与遥操互斥勿同时跑):
  python3 replay.py --list                                # 列出仓库内全部数据集
  python3 replay.py data/zbl_data/<任务目录>               # 干跑: 打印episode摘要
  python3 replay.py data/zbl_data/<任务目录> --engage      # 回放最新一集
  python3 replay.py <任务目录> --episode 0 --speed 0.5 --engage

安全: 先以当前位姿保持2s -> 限速缓动预对齐到第一帧 -> 按fps*speed回放
(逐帧限幅vmax) -> 结束保持1s后软急停; Ctrl+C随时软急停。
"""
import argparse
import os
import subprocess
import sys
import time

import numpy as np
import pandas as pd

SLAVE_LOWER = [-2.792, 0.0, -3.14, -1.57, -1.4, -1.745]
SLAVE_UPPER = [2.792, 3.44, 0.0, 1.57, 1.4, 1.745]
JOINT_NAMES = [f"arm_joint{i}" for i in range(1, 7)]


def load_episode(ds_dir, ep_idx):
    import json
    info = json.load(open(os.path.join(ds_dir, "meta", "info.json")))
    n_total = int(info.get("total_episodes", 0))
    if ep_idx is None:
        ep_idx = n_total - 1          # 默认最新一集
    if not 0 <= ep_idx < n_total:
        raise SystemExit(f"episode序号{ep_idx}超出范围(共{n_total}集: 0~{n_total-1})")
    pq = os.path.join(ds_dir, "data", "chunk-000", f"file-{ep_idx:03d}.parquet")
    if not os.path.exists(pq):
        raise SystemExit(f"找不到 {pq}")
    df = pd.read_parquet(pq)
    return ep_idx, df, float(info.get("fps", 20))


def cmd_list(root):
    print(f"{'任务数据集':<58} {'集数':>4} {'帧数':>6} {'fps':>4}")
    base = os.path.join(root)
    if not os.path.isdir(base):
        raise SystemExit(f"数据根目录不存在: {base}")
    import json
    for grp in sorted(os.listdir(base)):
        gd = os.path.join(base, grp)
        if not os.path.isdir(gd):
            continue
        for task in sorted(os.listdir(gd)):
            td = os.path.join(gd, task)
            ij = os.path.join(td, "meta", "info.json")
            if not os.path.exists(ij):
                continue
            i = json.load(open(ij))
            print(f"{os.path.join(grp, task):<58} {i['total_episodes']:>4} "
                  f"{i['total_frames']:>6} {i.get('fps', '?'):>4}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('dataset', nargs='?', help="任务数据集目录(或--list)")
    p.add_argument('--list', action='store_true', help='列出数据根目录下全部数据集')
    p.add_argument('--episode', type=int, default=None, help='episode序号(默认最新一集)')
    p.add_argument('--speed', type=float, default=1.0, help='回放倍速(默认1.0, 建议0.5慢放)')
    p.add_argument('--vmax', type=float, default=3.0, help='逐帧限幅rad/s(默认3.0)')
    p.add_argument('--grip-max', type=float, default=1.70, help='夹爪满开rad(默认1.70)')
    p.add_argument('--align-vmax', type=float, default=0.3, help='缓动限速rad/s(回零/预对齐)')
    p.add_argument('--no-home-start', action='store_true', help='跳过开始时回零')
    p.add_argument('--no-home-end', action='store_true', help='跳过结束时回零')
    p.add_argument('--engage', action='store_true', help='真机回放(不加只打印摘要)')
    p.add_argument('--data-root', default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'data'),
        help='--list时的数据根目录(默认仓库data/)')
    args = p.parse_args()

    if args.list:
        cmd_list(args.data_root)
        return
    if not args.dataset:
        p.error("请给数据集目录, 或用 --list")
    ep_idx, df, fps = load_episode(args.dataset, args.episode)
    j = np.stack(df['observation.joints'].values)          # (N,7) 关节+夹爪开度
    n = len(j)
    print(f"episode_{ep_idx:03d}: {n}帧 @ {fps:.0f}fps = {n / fps:.1f}s, "
          f"倍速{args.speed} -> 实际{n / fps / args.speed:.1f}s")
    print("  关节范围(rad):")
    for i in range(6):
        print(f"    J{i + 1}: [{j[:, i].min():+.3f}, {j[:, i].max():+.3f}]")
    print(f"  夹爪开度: [{j[:, 6].min():.2f}, {j[:, 6].max():.2f}]")
    if not args.engage:
        print("干跑模式(加 --engage 才真机回放)")
        return

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Float64MultiArray
    from sensor_msgs.msg import JointState
    from controller_manager_msgs.srv import SwitchController

    rclpy.init()
    node = Node('replay')
    jpub = node.create_publisher(Float64MultiArray,
                                 '/joint_position_controller/commands', 10)
    gpub = node.create_publisher(Float64MultiArray,
                                 '/gripper_controller/commands', 10)
    cur = {'j': None}

    def on_js(msg):
        try:
            idx = [msg.name.index(nm) for nm in JOINT_NAMES]
        except ValueError:
            return
        cur['j'] = np.array([msg.position[i] for i in idx])

    node.create_subscription(JointState, '/joint_states', on_js, 10)
    cli = node.create_client(SwitchController, '/controller_manager/switch_controller')
    if not cli.wait_for_service(timeout_sec=5.0):
        raise SystemExit("switch_controller 服务不可用(SDK起了吗?)")

    def spin(t=0.01):
        rclpy.spin_once(node, timeout_sec=t)

    def switch(activate, deactivate=()):
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.BEST_EFFORT
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
        ok = fut.result() is not None and fut.result().ok
        print(f"切换控制器 activate={activate} deactivate={deactivate} -> {ok}")
        return ok

    def send_joints(q, grip_open):
        msg = Float64MultiArray()
        msg.data = [float(v) for v in q]
        jpub.publish(msg)
        g = Float64MultiArray()
        g.data = [float(np.clip(grip_open, 0.0, 1.0) * args.grip_max)]
        gpub.publish(g)

    def clamp(cur_q, tgt_q, dt):
        out = np.array(tgt_q, dtype=float)
        max_d = args.vmax * dt
        for i in range(6):
            d = out[i] - cur_q[i]
            out[i] = cur_q[i] + np.clip(d, -max_d, max_d)
        return np.clip(out, SLAVE_LOWER, SLAVE_UPPER)

    try:
        # 先全部退出(含夹爪重置: 强杀SDK后gripper伺服可能掉线, 夹爪发软)
        if not switch([], ['cart_pose_controller', 'zero_force_dragging_controller',
                           'gripper_controller']):
            raise SystemExit("控制器切换失败")
        time.sleep(0.5)
        if not switch(['joint_position_controller', 'gripper_controller'], []):
            raise SystemExit("切换到关节位置模式失败")
        t0 = time.time()
        while time.time() - t0 < 2.0 and cur['j'] is None:
            spin(0.05)
        if cur['j'] is None:
            raise SystemExit("未收到从臂/joint_states")
        print("以当前位姿保持2s...")
        t0 = time.time()
        while time.time() - t0 < 2.0:
            spin()
            send_joints(cur['j'], j[0, 6])
            time.sleep(0.01)
        def ramp_to(tq, grip_open, g_from, what):
            """从当前关节位置限速缓动到目标(50Hz, 夹爪同步插值)"""
            c0 = np.array(cur['j'])
            tq = np.asarray(tq, dtype=float)
            dist = float(np.max(np.abs(tq - c0)))
            if dist < 0.02:
                send_joints(tq, grip_open)   # 关节已到位, 夹爪仍需对齐
                return
            dur = dist / args.align_vmax
            print(f"{what}: 偏差{np.degrees(dist):.1f}°, "
                  f"限速{args.align_vmax}rad/s, 约{dur:.1f}s")
            steps = max(int(dur * 50), 2)
            for k in range(1, steps + 1):
                a = k / steps
                send_joints(c0 + (tq - c0) * a,
                            g_from + (grip_open - g_from) * a)
                spin()
                time.sleep(0.02)

        zeros = np.zeros(6)
        if not args.no_home_start:
            ramp_to(zeros, 0.0, j[0, 6], "起始回零")
        ramp_to(j[0, :6], j[0, 6], 0.0, "预对齐到首帧")
        print(f"开始回放 episode_{ep_idx:03d} ({n}帧)...")
        period = 1.0 / (fps * args.speed)
        last = np.array(j[0, :6])
        t_start = time.time()
        for i in range(n):
            tgt = clamp(last, j[i, :6], period)
            send_joints(tgt, j[i, 6])
            last = tgt
            spin()
            el = t_start + (i + 1) * period - time.time()
            if el > 0:
                time.sleep(el)
            if i % max(int(fps), 1) == 0:
                print(f"  {i + 1}/{n}帧 ({100 * (i + 1) // n}%)")
        print("回放完成, 末端保持1s...")
        t0 = time.time()
        while time.time() - t0 < 1.0:
            send_joints(last, j[-1, 6])
            spin()
            time.sleep(0.01)
        if not args.no_home_end:
            ramp_to(zeros, 0.0, j[-1, 6], "结束回零")
    except KeyboardInterrupt:
        print("\n中断, 软急停...")
    finally:
        try:
            switch([], ['joint_position_controller', 'cart_pose_controller',
                        'zero_force_dragging_controller'])
        except Exception:
            subprocess.run(['ros2', 'control', 'switch_controllers', '--deactivate',
                            'joint_position_controller', 'cart_pose_controller',
                            'zero_force_dragging_controller'], check=False)
        node.destroy_node()
        rclpy.shutdown()
        print("已软急停退出")


if __name__ == '__main__':
    main()
