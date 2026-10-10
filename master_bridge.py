#!/usr/bin/env python3
"""主臂(rosbridge) -> 从臂(sdk_arm_6a) 关节空间遥操作桥接

在从臂SDK容器内运行(rclpy来自 /opt/xr/core/install/setup.bash):
    python3 master_bridge.py            # 干跑: 只打印主臂->映射后的关节角, 不发指令
    python3 master_bridge.py --engage   # 预对齐后进入实时跟随
启动时默认自动激活主臂重力补偿(--no-gravity-comp 可关闭)

数据源: 独立进程master_rx.py(rosbridge->UDP), 订阅/joint_states(约200Hz)
        与 /joy_trigger/state/<side>(扳机)
输出:   /joint_position_controller/commands (6关节, rad)
        /gripper_controller/commands (扳机->夹爪, 自适应标定:
        松开=闭合, 按压=张开, 行程拉开后自动变比例, 全开开度--gripper-max)

安全设计:
  - 主从同构, 恒等映射, 逐周期限幅(关节限位 + 最大角速度 vmax)
  - 主臂数据流中断 >0.5s 即停止发布(从臂阻抗控制器保持当前位置), 恢复后自动限速追平
  - Ctrl+C 或异常退出时去激活全部控制器(软急停)
"""
import argparse
import datetime
import json
import math
import os
import socket
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController

# 从臂关节限位(来源: examples/python/joint_control.py, rad)
SLAVE_LOWER = [-2.792, 0.0, -3.14, -1.57, -1.4, -1.745]
SLAVE_UPPER = [2.792, 3.44, 0.0, 1.57, 1.4, 1.745]


def log_line(msg, level="INFO"):
    """统一日志: 人类可读时间戳 + 空行分隔"""
    now = datetime.datetime.now()
    print(f"[{now.strftime('%m-%d %H:%M:%S.%f')[:-3]}] [{level}] {msg}",
          flush=True)
    print(flush=True)


class MasterBridge(Node):
    @staticmethod
    def log(msg, level="INFO"):
        log_line(msg, level)

    @staticmethod
    def log_error(msg):
        log_line(msg, "ERROR")

    def log_throttle(self, msg, interval=2.0, level="WARN"):
        now = time.perf_counter()
        if now - self._warn_ts.get(msg, 0.0) >= interval:
            self._warn_ts[msg] = now
            log_line(msg, level)

    def __init__(self, args):
        super().__init__('master_bridge')
        self.args = args
        self._warn_ts = {}          # 节流warn的上次打印时刻
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray, '/joint_position_controller/commands', 10)
        self.slave_joint_sub = self.create_subscription(
            JointState, '/joint_states', self.on_slave_joint_state, 10)

        self.switch_cli = self.create_client(
            SwitchController, '/controller_manager/switch_controller')

        self.master_joints = None       # 最近一帧主臂6关节
        self.master_last_ts = 0.0
        self.master_rx_perf = 0.0       # 最近一帧到达时刻(perf_counter)
        self.slave_joints = None
        self.slave_gripper = None       # 从臂夹爪位置(rad, 0=合, 满开~4.5)
        self.last_cmd = None
        self.trigger_count = 0
        self._last_log = -1
        self.joy_count = 0
        self._last_joy_perf = None        # 最近一帧手柄数据时刻(优先数据源判据)
        # 扳机行程标定: 真实区间[trig_min, trig_max] -> 夹爪[0, gripper_max]
        self.trig_min = args.trig_min if args.trig_min is not None else 0.0
        self.trig_max = args.trig_max if args.trig_max is not None else 1.0
        self._trig_obs_min = None
        self._trig_obs_max = None
        # 扳机 -> 夹爪: 自适应标定(松开=闭合基线, 按压=张开)
        self.trig_value = None          # 最近一次扳机原始数值
        self.trig_baseline = None       # 松开状态基线值
        self.trig_lo = None             # 观测到的最小值
        self.trig_hi = None             # 观测到的最大值
        self.trig_open = 0.0            # 0=合 1=全开
        self.grip_target = None         # 夹爪目标(rad)
        self.grip_last_cmd = None       # 最近一次已发布夹爪指令(rad)
        self.gripper_cmd_pub = self.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)
        # 延迟诊断: 主臂帧到达间隔 / 发布时帧龄 / tick周期
        from collections import deque
        self._stat_rx_gap = deque(maxlen=1000)
        self._stat_age = deque(maxlen=1000)
        self._stat_tick = deque(maxlen=1000)
        self._last_rx_perf = None
        self._last_tick_perf = None
        self._last_stat_log = time.perf_counter()

        # 主臂数据经独立进程(master_rx.py)接收, UDP喂入本进程;
        # 避免websocket线程与rclpy执行器同进程的GIL互相饿死
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", args.port))
        self.udp.setblocking(False)

        self.timer = self.create_timer(1.0 / args.rate, self.tick)
        self.log(
            f"桥接启动: 主臂={args.master} side={args.side} "
            f"rate={args.rate}Hz vmax={args.vmax}rad/s engage={args.engage}")

    # ---------- 主臂重力补偿(经rosbridge调用主臂PC的controller_manager) ----------
    def enable_master_gravity(self):
        """激活主臂侧的零重力补偿控制器({side}_gravity_compensate_controller),
        否则操作者需徒手对抗主臂自重。激活后保持, 退出时不去激活(避免臂突然变沉)。"""
        import websocket as _ws
        try:
            ws = _ws.create_connection(self.args.master, timeout=5)
            ws.send(json.dumps({
                "op": "call_service", "id": "grav",
                "service": "/controller_manager/switch_controller",
                "args": {
                    "activate_controllers":
                        [f"{self.args.side}_gravity_compensate_controller"],
                    "deactivate_controllers": [],
                    "strictness": 1,       # BEST_EFFORT
                    "activate_asap": False,
                    "timeout": {"sec": 3, "nanosec": 0},
                }}))
            t0 = time.time()
            while time.time() - t0 < 6.0:
                m = json.loads(ws.recv())
                if m.get("id") == "grav":
                    ok = (m.get("values") or {}).get("ok", False)
                    msg = (m.get("values") or {}).get("message", "")
                    if ok:
                        self.log(
                            f"主臂重力补偿已激活"
                            f"({self.args.side}_gravity_compensate_controller)")
                    else:
                        self.log_error(f"主臂重力补偿激活失败: {msg}")
                    ws.close()
                    return ok
            self.log_error("主臂重力补偿激活无响应(链路抖动?)")
            ws.close()
            return False
        except Exception as e:
            self.log_error(f"主臂重力补偿激活异常: {e}")
            return False

    # ---------- 主臂侧(UDP, 数据来自独立进程master_rx.py) ----------
    def drain_udp(self):
        """非阻塞取出全部待处理报文, 只保留最新一帧关节角"""
        now_pc = time.perf_counter()
        for _ in range(200):   # 每tick最多取200帧, 防极端积压占用
            try:
                data, _ = self.udp.recvfrom(2048)
            except BlockingIOError:
                break
            if data.startswith(b"MJ "):
                try:
                    pos = [float(v) for v in data[3:].split()]
                except ValueError:
                    continue
                if len(pos) != 6:
                    continue
                if not all(math.isfinite(v) for v in pos):
                    continue
                if self._last_rx_perf is not None:
                    self._stat_rx_gap.append(
                        (now_pc - self._last_rx_perf) * 1000.0)
                self._last_rx_perf = now_pc
                self.master_joints = pos
                self.master_last_ts = time.time()
                self.master_rx_perf = now_pc
            elif data.startswith(b"TRIG "):
                try:
                    m = json.loads(data[5:])
                except ValueError:
                    continue
                self.on_trigger("joy_trigger/udp", m)
            elif data.startswith(b"JOY "):
                try:
                    m = json.loads(data[4:])
                except ValueError:
                    continue
                self.on_joy(m)

    @staticmethod
    def pick_joints(msg, want_names):
        try:
            if isinstance(msg, dict):   # rosbridge JSON(主臂侧)
                names, positions = msg["name"], msg["position"]
            else:                       # rclpy JointState(从臂侧)
                names, positions = msg.name, msg.position
            idx = [names.index(n) for n in want_names]
            return [float(positions[i]) for i in idx]
        except (ValueError, KeyError, IndexError, AttributeError):
            return None

    @staticmethod
    def extract_trigger_value(m):
        """从JoyTriggerState载荷中提取扳机数值(0~1或原始模拟量), 未知的字段名自适应"""
        if not isinstance(m, dict):
            return None
        for k in ("pressed", "is_pressed", "trigger_pressed"):
            v = m.get(k)
            if isinstance(v, bool):
                return 1.0 if v else 0.0
        for k in ("trigger_position", "trigger_value", "position", "value",
                  "trigger", "analog", "raw"):
            v = m.get(k)
            if isinstance(v, (int, float)) and not isinstance(v, bool) \
                    and math.isfinite(v):
                return float(v)
        for v in m.values():   # 兜底: 第一个数值字段
            if isinstance(v, (int, float)) and not isinstance(v, bool) \
                    and math.isfinite(v):
                return float(v)
        return None

    def on_joy(self, m):
        """手柄驱动话题(x2robot_msgs/Joystick): trigger字段名义0~1,
        实际区间因手柄个体差异可能不满量程, 用trig_min/max标定后重映射"""
        if not isinstance(m, dict):
            return
        self.joy_count += 1
        self._last_joy_perf = time.perf_counter()
        # 只取trigger字段; 摇杆/按钮等与单臂遥操无关的字段不解析不打印
        v = m.get("trigger")
        if not isinstance(v, (int, float)) or isinstance(v, bool) \
                or not math.isfinite(v):
            return
        v = float(v)                  # 保留原始值用于标定
        self.trig_value = v
        # 运行期观测范围(供--calib-trigger使用)
        if self._trig_obs_min is None or v < self._trig_obs_min:
            self._trig_obs_min = v
        if self._trig_obs_max is None or v > self._trig_obs_max:
            self._trig_obs_max = v
        span = self.trig_max - self.trig_min
        if span > 1e-6:
            self.trig_open = max(0.0, min(
                (v - self.trig_min) / span, 1.0))
        else:
            self.trig_open = 0.0
        self.grip_target = self.trig_open * self.args.gripper_max

    def on_trigger(self, topic, m):
        # 厂家/joy_trigger话题通常休眠; 手柄数据在手时优先, 仅作手柄断流时的兜底,
        # 避免两个数据源同时在场时互相覆盖夹爪目标
        if self._last_joy_perf is not None and \
                time.perf_counter() - self._last_joy_perf < 1.0:
            return
        self.trigger_count += 1
        if self.trigger_count <= 10 or self.trigger_count % 200 == 0:
            self.log(
                f"扳机数据[{topic}]: {json.dumps(m)}")
        v = self.extract_trigger_value(m)
        if v is None:
            return
        self.trig_value = v
        if self.trig_baseline is None:
            # 首个采样默认为"松开"状态(厂家逻辑: 松开=闭合)
            self.trig_baseline = v
            self.trig_lo = v
            self.trig_hi = v
        self.trig_lo = min(self.trig_lo, v)
        self.trig_hi = max(self.trig_hi, v)
        spread = self.trig_hi - self.trig_lo
        if spread > 1e-6:
            self.trig_open = max(0.0, min(1.0, (v - self.trig_lo) / spread))
        else:
            # 行程还没拉开: 高于基线即视为按压张开
            self.trig_open = 1.0 if v > self.trig_baseline + 1e-9 else 0.0
        self.grip_target = self.trig_open * self.args.gripper_max

    # ---------- 从臂侧 ----------
    def on_slave_joint_state(self, msg):
        pos = self.pick_joints(
            msg, [f"arm_joint{i}" for i in range(1, 7)])
        if pos is not None:
            self.slave_joints = pos
        # 夹爪关节(0~4.5rad = 双边0~78.75mm)
        grip = self.pick_joints(msg, ["arm_gripper"])
        if grip is not None:
            self.slave_gripper = grip[0]

    def gripper_str(self):
        if self.slave_gripper is None:
            return "夹爪:--"
        if not math.isfinite(self.slave_gripper):
            return "夹爪:NaN(从臂无反馈)"
        # 百分比以真实物理满开(gripper_max)为基准, mm按厂家标称比例(4.5rad=78.75mm)
        gmax = self.args.gripper_max if self.args.gripper_max > 0.1 else 4.5
        pct = max(0.0, min(self.slave_gripper / gmax * 100.0, 100.0))
        mm = self.slave_gripper / 4.5 * 78.75
        return f"夹爪:{self.slave_gripper:.2f}rad({pct:.0f}%开/{mm:.0f}mm)"

    def switch_controllers(self, activate, deactivate=()):
        if not self.switch_cli.wait_for_service(timeout_sec=5.0):
            self.log_error("switch_controller 服务不可用")
            return False
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.BEST_EFFORT
        fut = self.switch_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        ok = fut.result() is not None and fut.result().ok
        self.log(
            f"切换控制器 activate={activate} deactivate={deactivate} -> {ok}")
        return ok

    def soft_stop(self):
        """软急停: 优先走service; Ctrl+C后rcl上下文可能已失效, 用ros2 CLI兜底"""
        try:
            if self.switch_controllers(
                    [], ["joint_position_controller", "cart_pose_controller",
                         "zero_force_dragging_controller"]):
                return
        except Exception:
            pass  # context已失效(SIGINT路径), 落到CLI兜底
        try:
            subprocess.run(
                ["ros2", "control", "switch_controllers", "--deactivate",
                 "joint_position_controller", "cart_pose_controller",
                 "zero_force_dragging_controller"],
                timeout=10,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            log_line("软急停(CLI兜底)已执行")
        except Exception as e:
            self.log_error(
                f"软急停失败: {e} —— 请手动执行: ros2 control switch_controllers "
                "--deactivate joint_position_controller cart_pose_controller "
                "zero_force_dragging_controller")

    # ---------- 主循环 ----------
    def clamp_step(self, target):
        """限幅: 关节限位 + 单周期最大角速度"""
        out = list(target)
        if self.last_cmd is not None:
            max_d = self.args.vmax / self.args.rate
            for i in range(6):
                d = out[i] - self.last_cmd[i]
                out[i] = self.last_cmd[i] + math.copysign(
                    min(abs(d), max_d), d)
        for i in range(6):
            out[i] = min(max(out[i], SLAVE_LOWER[i]), SLAVE_UPPER[i])
        return out

    def publish_joints(self, q):
        msg = Float64MultiArray()
        msg.data = q
        self.joint_cmd_pub.publish(msg)
        self.last_cmd = q

    def wait_master(self, timeout=10.0):
        self.log("等待主臂数据流...")
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.drain_udp()
            if self.master_joints is not None:
                return True
            time.sleep(0.1)
        return False

    def prealign(self, target):
        """从臂以低速线性插值平滑对齐到主臂当前构型"""
        cur = self.slave_joints
        if cur is None:
            self.log_error("未收到从臂 /joint_states, 放弃对齐")
            return False
        if not all(math.isfinite(v) for v in cur):
            self.log_error(
                "从臂关节数据为NaN —— 机械臂未上电或CAN断连, "
                "请检查臂的电源/连接并重启SDK栈, 放弃对齐")
            return False
        if not all(math.isfinite(v) for v in target):
            self.log_error("主臂关节角含NaN(坏帧), 放弃对齐")
            return False
        dist = max(abs(t - c) for t, c in zip(target, cur))
        if dist < 0.02:
            return True
        dur = dist / (self.args.vmax / 3.0)
        steps = max(int(dur * self.args.rate), 2)
        self.log(
            f"预对齐: 最大关节偏差 {math.degrees(dist):.1f}°, "
            f"预计 {dur:.1f}s —— 请确认从臂周围无人!")
        time.sleep(2.0)
        for k in range(1, steps + 1):
            a = k / steps
            q = [c + (t - c) * a for c, t in zip(cur, target)]
            self.publish_joints(self.clamp_step(q))
            time.sleep(1.0 / self.args.rate)
        return True

    @staticmethod
    def _pct(seq, p):
        if not seq:
            return 0.0
        s = sorted(seq)
        return s[min(int(len(s) * p / 100), len(s) - 1)]

    def log_stats(self):
        def fmt(seq):
            return (f"p50={self._pct(seq,50):.1f} p95={self._pct(seq,95):.1f} "
                    f"max={max(seq) if seq else 0:.1f}ms")
        self.log(
            f"[延迟诊断] 主臂帧到达间隔: {fmt(self._stat_rx_gap)} | "
            f"发布时帧龄: {fmt(self._stat_age)} | "
            f"tick周期: {fmt(self._stat_tick)} | "
            f"{self.gripper_str()} | {self.trigger_str()} | "
            f"手柄帧:{self.joy_count}"
            + (f" 兜底扳机帧:{self.trigger_count}"
               if self.trigger_count else ""))
        self._stat_rx_gap.clear()
        self._stat_age.clear()
        self._stat_tick.clear()

    def update_gripper(self):
        """扳机目标 -> 夹爪指令(带速度限幅与死区)"""
        if self.grip_target is None:
            return
        if self.grip_last_cmd is None:
            self.grip_last_cmd = (
                self.slave_gripper if self.slave_gripper is not None else 0.0)
        cmd = self.grip_target
        max_d = self.args.gripper_vmax / self.args.rate
        d = cmd - self.grip_last_cmd
        cmd = self.grip_last_cmd + math.copysign(min(abs(d), max_d), d)
        cmd = max(0.0, min(cmd, 4.5))
        if abs(cmd - self.grip_last_cmd) < 0.005:
            return
        msg = Float64MultiArray()
        msg.data = [cmd]
        self.gripper_cmd_pub.publish(msg)
        self.grip_last_cmd = cmd

    def trigger_str(self):
        if self.trig_value is None:
            return "扳机:无数据"
        return (f"扳机:{self.trig_value:.3f}[{self.trig_min:.3f},"
                f"{self.trig_max:.3f}](开度{self.trig_open*100:.0f}%"
                f"->目标{self.grip_target:.2f}rad)")

    def calibrate_trigger(self, seconds=6.0):
        """交互标定: 期间将扳机 松开->按到底->松开, 取观测区间"""
        self.log(
            f"扳机标定开始: {seconds:.0f}秒内将扳机 松开->按到底->松开...")
        self._trig_obs_min = None
        self._trig_obs_max = None
        t0 = time.time()
        while time.time() - t0 < seconds:
            self.drain_udp()
            time.sleep(0.02)
        lo, hi = self._trig_obs_min, self._trig_obs_max
        if lo is None or hi is None or hi - lo < 1e-4:
            self.log_error("标定失败: 未观测到扳机变化(没收到手柄数据?)")
            return False
        self.trig_min, self.trig_max = lo, hi
        self.log(
            f"扳机标定完成: 实测区间 [{lo:.4f}, {hi:.4f}] 线性映射到 "
            f"夹爪 [0, {self.args.gripper_max:.2f}]rad; "
            f"固化参数: --trig-min {lo:.4f} --trig-max {hi:.4f}")
        return True

    def tick(self):
        now_pc = time.perf_counter()
        if self._last_tick_perf is not None:
            self._stat_tick.append((now_pc - self._last_tick_perf) * 1000.0)
        self._last_tick_perf = now_pc

        if now_pc - self._last_stat_log > 5.0:
            self._last_stat_log = now_pc
            self.log_stats()

        self.drain_udp()

        master = self.master_joints
        age = (time.time() - self.master_last_ts) if master else 1e9
        rx_perf = self.master_rx_perf

        if master is None:
            return
        if not self.args.engage:
            self._stat_age.append((now_pc - rx_perf) * 1000.0)
            now = int(time.time() * 2)
            if now != self._last_log:
                self._last_log = now
                self.log(
                    f"[干跑] 主臂关节: {['%.3f' % v for v in master]} | "
                    f"{self.gripper_str()} | {self.trigger_str()}")
            return
        if age > 0.5:
            self.log_throttle(
                "主臂数据流中断, 暂停跟随(从臂保持当前位置)", 2.0, "WARN")
            return
        self._stat_age.append((now_pc - rx_perf) * 1000.0)
        self.publish_joints(self.clamp_step(master))
        self.update_gripper()
        # 2Hz对照日志: 分辨"稳态不对应(映射问题)"还是"动态跟不上(滞后问题)"
        now = int(time.time() * 2)
        if now != self._last_log:
            self._last_log = now
            actual = (self.slave_gripper
                      if self.slave_gripper is not None else 0.0)
            tgt = self.grip_target if self.grip_target is not None else 0.0
            self.log(
                f"[跟随] 夹爪 目标{tgt:.2f}rad 实际{actual:.2f}rad "
                f"偏差{actual - tgt:+.2f}rad | {self.trigger_str()}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--side', choices=['left', 'right'], default='right',
                   help='使用哪个主臂(默认right)')
    p.add_argument('--master', default='ws://192.168.31.7:9090',
                   help='主臂PC rosbridge地址')
    p.add_argument('--rate', type=float, default=100.0,
                   help='从臂指令发布频率Hz(默认100)')
    p.add_argument('--throttle-ms', type=int, default=5,
                   help='rosbridge服务端节流间隔ms(5=200Hz, 10=100Hz)')
    p.add_argument('--vmax', type=float, default=3.0,
                   help='软限最大关节角速度rad/s(默认3.0)')
    p.add_argument('--gripper-max', type=float, default=1.70,
                   help='扳机全开对应的夹爪开度rad(默认1.70=本机实测物理满开)')
    p.add_argument('--gripper-vmax', type=float, default=12.0,
                   help='夹爪最大角速度rad/s(默认12, 满开约0.38s)')
    p.add_argument('--trig-min', type=float, default=None,
                   help='扳机松开时的输出值(标定用, 默认0)')
    p.add_argument('--trig-max', type=float, default=None,
                   help='扳机按到底时的输出值(标定用, 默认1)')
    p.add_argument('--calib-trigger', action='store_true',
                   help='启动时交互标定扳机行程(6秒内松开->按到底->松开)')
    p.add_argument('--no-gravity-comp', action='store_true',
                   help='禁用启动时自动激活主臂重力补偿(默认激活)')
    p.add_argument('--engage', action='store_true',
                   help='实际控制从臂(默认干跑只打印)')
    p.add_argument('--port', type=int, default=9870,
                   help='接收master_rx进程UDP数据的端口')
    args = p.parse_args()

    # 拉起主臂接收子进程(独立进程, 避免与rclpy的GIL互相饿死)
    rx_script = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'master_rx.py')
    rx_proc = subprocess.Popen(
        [sys.executable, rx_script,
         '--side', args.side, '--master', args.master,
         '--throttle-ms', str(args.throttle_ms),
         '--port', str(args.port)])
    log_line(f"已拉起master_rx子进程 pid={rx_proc.pid}")

    rclpy.init()
    node = MasterBridge(args)
    if not args.no_gravity_comp:
        node.enable_master_gravity()
    engaged = False
    activated = False   # 控制器已切换成功(此后任何退出路径都必须软急停)
    try:
        if not node.wait_master():
            raise SystemExit("10秒内未收到主臂数据, 退出")
        if args.calib_trigger:
            node.calibrate_trigger()
        if args.engage:
            if not node.switch_controllers(["joint_position_controller"],
                                           ["cart_pose_controller",
                                            "zero_force_dragging_controller"]):
                raise SystemExit("切换到关节位置模式失败")
            activated = True
            time.sleep(1.0)
            if not node.prealign(node.master_joints):
                raise SystemExit("预对齐失败")
            engaged = True
            node.log("=== 实时跟随已启动(Ctrl+C 安全退出) ===")
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if activated:
            node.soft_stop()
        rx_proc.terminate()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass  # 信号已触发rclpy关闭时二次shutdown会报错, 忽略
        log_line("桥接已退出")


if __name__ == '__main__':
    main()
