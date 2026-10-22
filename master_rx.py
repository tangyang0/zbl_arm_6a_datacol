#!/usr/bin/env python3
"""主臂关节数据接收进程(独立运行, 不含rclpy)

背景: rclpy执行器的C层调用会周期性持有GIL, 与websocket接收线程同进程时
会把数据流饿出数百毫秒停顿; 独立进程经UDP转发彻底解耦。

功能: 连接主臂PC的rosbridge, 订阅/joint_states提取master_{side}_arm_joint1-6,
以UDP发送 "MJ j1 ... j6" 到 127.0.0.1:<port>;
扳机话题(/joy_trigger/state/*)原样转发 "TRIG {json}"。
由master_bridge.py自动拉起, 也可单独运行调试:
    python3 master_rx.py --side right --port 9870
"""
import argparse
import datetime
import json
import math
import socket
import time

import websockets
from websockets.sync.client import connect as ws_connect
from master_pose import discover_master_pose_topic, parse_master_pose_message


def log_line(msg):
    now = datetime.datetime.now()
    print(f"[{now.strftime('%m-%d %H:%M:%S.%f')[:-3]}] [master_rx] {msg}",
          flush=True)
    print(flush=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--side', choices=['left', 'right'], default='right')
    p.add_argument('--master', default='ws://192.168.31.7:9090')
    p.add_argument('--throttle-ms', type=int, default=5,
                   help='rosbridge服务端节流间隔ms(5=200Hz)')
    p.add_argument('--port', type=int, default=9870)
    p.add_argument('--master-ee-topic', default='auto',
                   help='主臂PoseStamped话题；auto只选择明确标记master/leader及左右侧的唯一话题')
    args = p.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    names = [f"master_{args.side}_arm_joint{i}" for i in range(1, 7)]
    dest = ("127.0.0.1", args.port)

    while True:
        try:
            # max_queue=1: 库内部只保留最新一条消息, 自动丢弃积压旧帧
            with ws_connect(args.master, open_timeout=5, max_queue=1) as ws:
                pose_topic = None
                try:
                    pose_topic = discover_master_pose_topic(ws, args.side, args.master_ee_topic)
                except (ValueError, TimeoutError) as exc:
                    log_line(f"主臂EE订阅未就绪: {exc}; 关节遥操继续，数采将拒绝缺少EE的episode")
                if pose_topic:
                    ws.send(json.dumps({"op": "subscribe", "topic": pose_topic, "queue_length": 1}))
                    log_line(f"主臂EE订阅: {pose_topic} (PoseStamped xyz+quaternion xyzw)")
                last_pose_error = 0.0
                ws.send(json.dumps(
                    {"op": "subscribe", "topic": "/joint_states",
                     "throttle_rate": args.throttle_ms,
                     "queue_length": 1}))
                ws.send(json.dumps(
                    {"op": "subscribe",
                     "topic": f"/joy_trigger/state/{args.side}"}))
                # 摇杆/扳机驱动话题: trigger字段为归一化0~1, 常驻发布
                ws.send(json.dumps(
                    {"op": "subscribe",
                     "topic": f"/{args.side}_joystick/input"}))
                log_line(f"已连接 {args.master}, "
                         f"转发到 udp/{dest[0]}:{dest[1]}")
                while True:
                    raw = ws.recv(timeout=5.0)
                    received_ns = time.monotonic_ns()
                    if isinstance(raw, bytes):
                        raw = raw.decode("utf-8", "ignore")
                    try:
                        msg = json.loads(raw)
                    except ValueError:
                        continue
                    if msg.get("op") != "publish":
                        continue
                    t = msg["topic"]
                    m = msg["msg"]
                    if t == "/joint_states":
                        try:
                            idx = [m["name"].index(n) for n in names]
                            pos = [float(m["position"][i]) for i in idx]
                        except (ValueError, KeyError, IndexError):
                            continue
                        # 主臂初始化瞬间可能发NaN坏帧, 直接丢弃
                        if not all(math.isfinite(v) for v in pos):
                            continue
                        sock.sendto(
                            ("MJ " + " ".join(f"{v:.6f}" for v in pos))
                            .encode(), dest)
                    elif pose_topic and t == pose_topic:
                        try:
                            packet = parse_master_pose_message(m, pose_topic, received_ns, args.side)
                            sock.sendto(("MP " + json.dumps(packet, allow_nan=False)).encode(), dest)
                        except (ValueError, TypeError) as exc:
                            if time.monotonic() - last_pose_error > 2.0:
                                log_line(f"拒绝非法主臂EE消息: {exc}")
                                last_pose_error = time.monotonic()
                    elif "joy_trigger" in t:
                        sock.sendto(("TRIG " + json.dumps(m)).encode(), dest)
                    elif "_joystick/input" in t:
                        sock.sendto(("JOY " + json.dumps(m)).encode(), dest)
        except TimeoutError:
            # websockets 17.x: recv超时抛内建TimeoutError(旧版才在exceptions下)
            log_line("5秒无数据(链路或服务端异常), 重连")
            time.sleep(1.0)
        except KeyboardInterrupt:
            log_line("收到退出信号")
            break
        except Exception as e:
            log_line(f"连接断开({e}), 3秒后重连")
            time.sleep(3.0)


if __name__ == '__main__':
    main()
