#!/usr/bin/env python3
"""主臂(rosbridge) -> 从臂(sdk_arm_6a) 关节空间遥操作桥接

在从臂SDK容器内运行(rclpy来自 /opt/xr/core/install/setup.bash):
    python3 master_bridge.py            # 干跑: 只打印主臂->映射后的关节角, 不发指令
    python3 master_bridge.py --engage   # 预对齐后进入实时跟随
启动时默认: 主从臂同步限速回零(--no-master-zero跳过) -> 激活主臂重力补偿
(--no-gravity-comp 可关闭)

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
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState, CompressedImage
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from controller_manager_msgs.srv import SwitchController

from lerobot_writer import EpisodeBuffer, write_episode, task_dirname

# 从臂关节限位(来源: examples/python/joint_control.py, rad)
SLAVE_LOWER = [-2.792, 0.0, -3.14, -1.57, -1.4, -1.745]
SLAVE_UPPER = [2.792, 3.44, 0.0, 1.57, 1.4, 1.745]


# 日志级别配色(ANSI): 数采各状态一眼可辨
_LEVEL_COLOR = {
    "ERROR": "\033[1;91m",    # 亮红
    "REC▶": "\033[1;92m",     # 亮绿: episode开始
    "REC…": "\033[1;93m",     # 亮黄: 正在写入
    "REC✓": "\033[1;96m",     # 亮青: 写入完成
    "REC✗": "\033[1;41;97m",  # 红底白字: 丢弃/异常
}


def log_line(msg, level="INFO"):
    """统一日志: 人类可读时间戳 + 空行分隔; REC/ERROR级别着色并加前置空行"""
    now = datetime.datetime.now()
    c = _LEVEL_COLOR.get(level, "")
    head = f"[{now.strftime('%m-%d %H:%M:%S.%f')[:-3]}] [{level}] "
    if c:
        print(f"\n{c}{head}{msg}\033[0m", flush=True)
    else:
        print(f"{head}{msg}", flush=True)
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
        # 数采直接写LeRobot所需的额外观测流
        self.endpose_sub = self.create_subscription(
            PoseStamped, '/arm/end_pose', self.on_end_pose, 10)
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/eye_in_hand/eye_in_hand/image_raw/image_compressed',
            self.on_image, 10)

        self.switch_cli = self.create_client(
            SwitchController, '/controller_manager/switch_controller')

        self.master_joints = None       # 最近一帧主臂6关节
        self.master_last_ts = 0.0
        self.master_rx_perf = 0.0       # 最近一帧到达时刻(perf_counter)
        self.slave_joints = None
        self.slave_gripper = None       # 从臂夹爪位置(rad, 0=合, 满开~4.5)
        self.last_cmd = None
        self._follow_enabled = False    # 启动阶段(归零/对齐)先挡住tick的跟随发布
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

        # 数采: 主臂手柄sw1短按 开/停一个episode
        self.rec_buf = None        # episode缓冲(录制中在内存累积原始流)
        self._rec_write_lock = threading.Lock()   # LeRobot写入串行化
        self.rec_dir = None          # 当前episode目录
        self.rec_t0 = None
        self.rec_ep = 0
        self._sw1_prev = False
        self._last_sw1_ts = 0.0
        self._sw2_prev = False
        self._home_busy = False       # 运行中主臂回零进行中(防重入/防开录)
        self._scan_episodes()

        self.timer = self.create_timer(1.0 / args.rate, self.tick)
        self.log(
            f"桥接启动: 主臂={args.master} side={args.side} "
            f"rate={args.rate}Hz vmax={args.vmax}rad/s engage={args.engage}")

    # ---------- 主臂重力补偿(经rosbridge调用主臂PC的controller_manager) ----------
    def _master_list_controllers(self, ws):
        ws.send(json.dumps({
            "op": "call_service", "id": "lst",
            "service": "/controller_manager/list_controllers", "args": {}}))
        t0 = time.time()
        while time.time() - t0 < 5.0:
            m = json.loads(ws.recv())
            if m.get("id") == "lst":
                return (m.get("values") or {}).get("controller", [])
        return []

    def _master_switch(self, ws, activate, deactivate, strictness=1):
        """返回ok(bool)或None(无响应)"""
        ws.send(json.dumps({
            "op": "call_service", "id": "sw",
            "service": "/controller_manager/switch_controller",
            "args": {
                "activate_controllers": activate,
                "deactivate_controllers": deactivate,
                "strictness": strictness,
                "activate_asap": False,
                "timeout": {"sec": 3, "nanosec": 0},
            }}))
        t0 = time.time()
        while time.time() - t0 < 5.0:
            m = json.loads(ws.recv())
            if m.get("id") == "sw":
                return (m.get("values") or {}).get("ok", False)
        return None

    def enable_master_gravity(self):
        """启动时强制主臂进入重力补偿模式:
        显式去激活关节/笛卡尔控制器(即"保持使能锁死"状态), 激活重力补偿,
        回查确认; 失败则FORCE_AUTO(4)自动踢掉占用接口的控制器再试一次。
        激活后保持, 退出时不去激活(避免臂突然变沉)。"""
        import websocket as _ws
        gc = f"{self.args.side}_gravity_compensate_controller"
        others = [f"master_{self.args.side}_arm_joint_controller",
                  f"master_{self.args.side}_arm_cartesian_controller"]
        try:
            ws = _ws.create_connection(self.args.master, timeout=5)
            before = {c.get("name"): c.get("state")
                      for c in self._master_list_controllers(ws)}
            stuck = [n for n in others if before.get(n) == "active"]
            if stuck:
                self.log(
                    f"检测到主臂位置伺服锁定: {stuck}, "
                    f"强制切换至重力补偿模式...")
            self._master_switch(ws, activate=[gc], deactivate=others,
                                strictness=1)
            states = {c.get("name"): c.get("state")
                      for c in self._master_list_controllers(ws)}
            if states.get(gc) != "active":
                # 二次强推: FORCE_AUTO自动解除一切占用关节接口的控制器
                self._master_switch(ws, activate=[gc], deactivate=[],
                                    strictness=4)
                states = {c.get("name"): c.get("state")
                          for c in self._master_list_controllers(ws)}
            ws.close()
            if states.get(gc) == "active":
                self.log(f"主臂已进入重力补偿模式({gc})")
                if stuck:
                    self.log(f"已解除位置伺服锁定: {stuck}")
                return True
            self.log_error(
                f"主臂重力补偿模式切换失败, 当前状态: {states.get(gc)}")
            return False
        except Exception as e:
            self.log_error(f"主臂重力补偿激活异常: {e}")
            return False

    # ---------- 主臂关节归零(位置伺服回零后交回重力补偿) ----------
    def _udp_latest_mj(self, wait=0.5):
        """从UDP流取最新一帧主臂关节角(归零到位反馈), 无则None"""
        pos, t0 = None, time.time()
        while time.time() - t0 < wait:
            try:
                data, _ = self.udp.recvfrom(2048)
            except BlockingIOError:
                time.sleep(0.01)
                continue
            if data.startswith(b"MJ "):
                try:
                    pos = [float(v) for v in data[3:].split()]
                except ValueError:
                    pass
        return pos

    def hold_slave(self, dur=2.0):
        """激活从臂控制器后立即以rate频率发布当前关节角:
        覆盖控制器可能保持的上一会话旧目标, 消除激活瞬间的窜动"""
        self.log(f"从臂控制器已激活, 以当前位姿保持{dur:.0f}s(覆盖旧目标)...")
        t0 = time.time()
        while time.time() - t0 < dur:
            rclpy.spin_once(self, timeout_sec=0.01)
            if self.slave_joints is not None and \
                    all(math.isfinite(v) for v in self.slave_joints):
                self.publish_joints(list(self.slave_joints))
            time.sleep(1.0 / self.args.rate)

    # ---------- 运行中主臂回零(sw2短按) ----------
    def master_home_toggle(self):
        """数采中主臂限速回零: 后台线程执行, 跟随循环不停
        (从臂经正常跟随自动同步到零位); 归零完成后主臂自动恢复重力补偿。"""
        if self._home_busy:
            self.log_throttle("主臂回零进行中, 忽略按键", 1.0, "WARN")
            return
        if self._recording():
            self.log("✗ 录制进行中, 先sw1结束本集再按sw2归零", "REC✗")
            return
        self._home_busy = True
        threading.Thread(target=self._master_home_worker, daemon=True).start()

    def _master_home_worker(self):
        try:
            ok = self.zero_master(notice=1.0)
            if self.enable_master_gravity():
                self.log("主臂回零完成, 已恢复重力补偿" if ok
                         else "主臂回零未完全到位, 已恢复重力补偿")
        except Exception as e:
            self.log_error(f"主臂回零流程异常: {e}")
        finally:
            self._home_busy = False

    def zero_master(self, timeout=20.0, tol=0.05, move_slave=False,
                    notice=3.0):
        """主臂(可选连同从臂)关节位置归零: 主臂切关节位置伺服, 主从各自从
        当前角度限速缓动收敛到0(move_slave时从臂经joint_position_controller
        同步缓动, 一次运动完成)。归零完成后由随后的enable_master_gravity
        交回重力补偿(主臂在零位变为自由)。超时/失败不阻塞启动, 只告警。"""
        import websocket as _ws
        jc = f"master_{self.args.side}_arm_joint_controller"
        gc = f"{self.args.side}_gravity_compensate_controller"
        cart = f"master_{self.args.side}_arm_cartesian_controller"
        topic = f"/{jc}/commands"
        who = "主从臂同步" if move_slave else "主臂"
        if notice > 0:
            self.log(f"★ {notice:.0f}秒后{who}将限速缓动回零, 请勿手持主臂, "
                     f"确保主从臂周围无人!", "INFO")
            time.sleep(notice)
        else:
            self.log(f"★ {who}开始限速缓动回零, 请勿手持主臂, "
                     f"确保主从臂周围无人!", "INFO")
        try:
            ws = _ws.create_connection(self.args.master, timeout=5)
        except Exception as e:
            self.log_error(f"主臂归零: rosbridge连接失败({e}), 跳过")
            return False
        try:
            ok = self._master_switch(ws, activate=[jc], deactivate=[gc, cart])
            if ok is not True:   # 接口被占时FORCE_AUTO再试一次
                ok = self._master_switch(ws, activate=[jc], deactivate=[],
                                         strictness=4)
            if ok is not True:
                self.log_error("主臂归零: 关节位置伺服激活失败, 跳过归零")
                ws.close()
                return False
            ws.send(json.dumps({"op": "advertise", "topic": topic,
                                "type": "std_msgs/msg/Float64MultiArray"}))
            # 限速缓动回零: 从各自当前角度线性收敛到0, 不直接下发0(会全速冲)
            m0 = self._udp_latest_mj(1.0)
            if m0 is None or len(m0) != 6:
                ws.send(json.dumps({"op": "unadvertise", "topic": topic}))
                ws.close()
                self.log_error("主臂归零: 无关节反馈流, 跳过")
                return False
            q0 = None
            if move_slave:
                t0 = time.time()
                while time.time() - t0 < 2.0 and (
                        self.slave_joints is None):
                    rclpy.spin_once(self, timeout_sec=0.05)
                if self.slave_joints is not None and \
                        all(math.isfinite(v) for v in self.slave_joints):
                    q0 = list(self.slave_joints)
                else:
                    self.log_error("从臂关节数据无效, 本次仅主臂回零")
            vmax = self.args.master_zero_vmax
            m_max = max(abs(v) for v in m0)
            s_max = max((abs(v) for v in (q0 or [])), default=0.0)
            dur = max(m_max, s_max) / vmax if vmax > 0 else 0.0
            msg = f"主臂最大{math.degrees(m_max):.0f}°"
            if q0 is not None:
                msg += f", 从臂最大{math.degrees(s_max):.0f}°"
            self.log(f"{who}回零: {msg}, 限速{vmax:.2f}rad/s, "
                     f"预计{dur:.1f}s")
            t0, err_m, err_s = time.time(), None, None
            while time.time() - t0 < dur + timeout:
                el = time.time() - t0
                s = max(0.0, 1.0 - el / dur) if dur > 0 else 0.0
                ws.send(json.dumps({"op": "publish", "topic": topic, "msg": {
                    "layout": {"dim": [], "data_offset": 0},
                    "data": [v * s for v in m0]}}))
                if q0 is not None:
                    rclpy.spin_once(self, timeout_sec=0.0)
                    self.publish_joints(
                        self.clamp_step([v * s for v in q0]))
                    if self.slave_joints is not None:
                        err_s = max(abs(v) for v in self.slave_joints)
                cur = self._udp_latest_mj(0.05)
                if cur is not None and len(cur) == 6:
                    err_m = max(abs(v) for v in cur)
                done = (s <= 0.0 and err_m is not None and err_m < tol and
                        (q0 is None or
                         (err_s is not None and err_s < tol)))
                if done:
                    break
                time.sleep(0.05)
            try:
                ws.send(json.dumps({"op": "unadvertise", "topic": topic}))
            except Exception:
                pass
            ws.close()
            ok_m = err_m is not None and err_m < tol
            ok_s = q0 is None or (err_s is not None and err_s < tol)
            if ok_m and ok_s:
                res = f"主臂偏差{err_m:.3f}rad"
                if q0 is not None:
                    res += f", 从臂偏差{err_s:.3f}rad"
                self.log(f"{who}已归零到位({res}, 耗时{time.time() - t0:.1f}s)")
                return True
            res = (f"主臂{err_m:.3f}" if err_m is not None else "主臂无反馈")
            if q0 is not None:
                res += (f", 从臂{err_s:.3f}" if err_s is not None
                        else ", 从臂无反馈")
            self.log_error(f"{who}归零超时(缓动{dur:.0f}s+收敛{timeout:.0f}s, "
                           f"当前偏差{res}rad), 继续启动")
            return False
        except Exception as e:
            self.log_error(f"主臂归零异常: {e}")
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
        # sw1上升沿(短按)切换数采录制开/停
        if not self.args.no_rec:
            sw1 = bool(m.get("sw1", False))
            now_pc = time.perf_counter()
            if sw1 and not self._sw1_prev and \
                    now_pc - self._last_sw1_ts > 0.5:
                self._last_sw1_ts = now_pc
                self.rec_toggle()
            self._sw1_prev = sw1
        # sw2上升沿(短按): 运行中主臂限速回零(后台线程, 从臂由跟随自动带零)
        sw2 = bool(m.get("sw2", False))
        if sw2 and not self._sw2_prev:
            self.master_home_toggle()
        self._sw2_prev = sw2
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
        # 数采: 观测流(6关节+夹爪)
        if self.rec_buf is not None and pos is not None \
                and self.slave_gripper is not None:
            self.rec_buf.state.append(list(pos) + [self.slave_gripper])
            self.rec_buf.state_ts.append(time.perf_counter())

    def on_end_pose(self, msg):
        if self.rec_buf is None:
            return
        p, q = msg.pose.position, msg.pose.orientation
        self.rec_buf.pose.append(
            [p.x, p.y, p.z, q.x, q.y, q.z, q.w])
        self.rec_buf.pose_ts.append(time.perf_counter())

    def on_image(self, msg):
        if self.rec_buf is None:
            return
        self.rec_buf.imgs.append(bytes(msg.data))
        self.rec_buf.img_ts.append(time.perf_counter())

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
        if self.rec_buf is not None:
            self.rec_buf.aj.append(list(q))
            self.rec_buf.aj_ts.append(time.perf_counter())

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
        dur = dist / self.args.align_vmax
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
            f"tick周期: {fmt(self._stat_tick)} | {self.rec_str()} | "
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
        if self.rec_buf is not None:
            self.rec_buf.ag.append([cmd])
            self.rec_buf.ag_ts.append(time.perf_counter())

    # ---------- 数采录制(sw1短按开/停episode) ----------
    def rec_ds_root(self):
        """本集的目标数据集目录: <data_root>/<数据集名>/<任务名>/"""
        return os.path.join(self.args.data_root, self.args.dataset,
                            task_dirname(self.args.task))

    def _scan_episodes(self):
        # episode计数来自本任务的已有数据集
        info = os.path.join(self.rec_ds_root(), "meta", "info.json")
        try:
            with open(info) as f:
                self.rec_ep = json.load(f)["total_episodes"]
        except Exception:
            self.rec_ep = 0

    def _recording(self):
        return self.rec_buf is not None

    def _check_dataset_schema(self, info_path):
        """已有数据集schema与当前录制格式不符时返回差异列表(用于开录前拒绝)"""
        try:
            with open(info_path) as f:
                meta = json.load(f)
        except Exception:
            return []                      # 元数据读不了就不拦, 写入时还有兜底报错
        feats = meta.get("features", {})
        want = {
            "observation.state": 10,
            "observation.joints": 7,
            "action": 10 if self.args.action_mode == "ee" else 7,
        }
        bad = []
        for k, d in want.items():
            got = feats.get(k, {}).get("shape", [None])[0]
            if got != d:
                bad.append(f"{k}: 数据集{got if got is not None else '无此键'}维"
                           f" vs 录制{d}维")
        if meta.get("fps") and int(meta["fps"]) != int(self.args.fps):
            bad.append(f"fps: 数据集{meta['fps']} vs 录制{self.args.fps}")
        return bad

    def rec_toggle(self):
        if self._recording():
            self.rec_stop()
            return
        if self._home_busy:
            self.log("✗ 主臂回零进行中, 稍后再开录", "REC✗")
            return
        # 开录前格式预检: 不符立即拒绝, 避免录完整集才在写入时报错白录
        info = os.path.join(self.rec_ds_root(), "meta", "info.json")
        if os.path.exists(info):
            bad = self._check_dataset_schema(info)
            if bad:
                self.log(f"✗✗✗ 拒绝录制: 数据集{self.args.dataset}与当前录制格式"
                         f"不符({'; '.join(bad)}), 请换 --dataset 名称", "REC✗")
                return
        self.rec_ep += 1
        self.rec_buf = EpisodeBuffer()   # 内存累积原始流, 停止时写数据集
        self.rec_dir = os.path.join(self.args.data_root, self.args.dataset)
        self.rec_t0 = time.time()
        self.log(f"▶▶▶ episode_{self.rec_ep:04d} 开始录制 -> LeRobot数据集 "
                 f"{self.rec_ds_root()} 任务: {self.args.task}", "REC▶")

    def rec_stop(self, sync=False):
        if self.rec_buf is None:
            return
        buf, ep = self.rec_buf, self.rec_ep
        dur = time.time() - self.rec_t0
        self.rec_buf = None
        # 过短的录制(测试性按压)掐头去尾后必然不足10帧, 直接丢弃不写
        if dur < self.args.trim * 2 + 0.6:
            self.log(f"✗ episode_{ep:04d} 时长{dur:.1f}s过短, 已丢弃"
                     f"(需>{self.args.trim * 2 + 0.6:.1f}s才成集)", "REC✗")
            return
        self.log(f"■ episode_{ep:04d} 录制结束({dur:.1f}s), "
                 f"正在写入LeRobot...", "REC…")

        def _write():
            # 写入串行化: 上一集还在编码时, 本集排队等待, 避免并发写坏数据集
            with self._rec_write_lock:
                try:
                    n = write_episode(
                        buf, self.rec_dir, self.args.task.strip(),
                        action_mode=self.args.action_mode,
                        fps=self.args.fps, trim=self.args.trim,
                        grip_max=self.args.gripper_max)
                    self.log(f"✓✓✓ episode_{ep:04d} 已写入: {n}帧 -> "
                             f"{self.rec_dir}", "REC✓")
                except Exception as e:
                    self.log(
                        f"✗✗✗ episode_{ep:04d} LeRobot写入失败: {e}",
                        "REC✗")

        if sync:
            _write()
        else:   # 后台写入, 不阻塞遥操控制循环(视频编码需数秒)
            threading.Thread(target=_write, daemon=True).start()

    def wait_rec_writes(self, timeout=180.0):
        """等待在途的后台LeRobot写入完成(退出前必须调用, 否则编码被杀数据丢失)"""
        if self._rec_write_lock.acquire(timeout=timeout):
            self._rec_write_lock.release()
            return True
        self.log_error(f"等待写入超时({timeout:.0f}s), 在途episode可能不完整")
        return False

    def rec_str(self):
        if self._recording():
            return f"REC:ep{self.rec_ep:04d}"
        return "REC:-"

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
        if not self._follow_enabled:
            return   # 启动编排中(回零/对齐), 跟随发布由编排流程接管
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
                f"[跟随] {self.rec_str()} | 夹爪 目标{tgt:.2f}rad "
                f"实际{actual:.2f}rad "
                f"偏差{actual - tgt:+.2f}rad | {self.trigger_str()}")


def main():
    os.umask(0o000)   # 数据文件对宿主机用户全开放(容器uid与本机不同, 避免带锁)
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
                   help='跳过启动时激活主臂重力补偿')
    p.add_argument('--no-master-zero', action='store_true',
                   help='跳过启动时主臂关节位置归零(默认: 归零后交回重力补偿;'
                        ' 仅在启用重力补偿时生效)')
    p.add_argument('--master-zero-vmax', type=float, default=0.3,
                   help='主从回零缓动限速rad/s(默认0.3, 想更慢再调小)')
    p.add_argument('--align-vmax', type=float, default=0.3,
                   help='从臂预对齐缓动限速rad/s(默认0.3, 与主臂回零同节奏)')
    p.add_argument('--task', default='demo',
                   help='数采任务(自然语言指令, 如 '
                        '"pick up the cube and put it in the bin")')
    p.add_argument('--dataset', default='zbl_data',
                   help='LeRobot数据集名(默认zbl_data, 增量追加)')
    p.add_argument('--data-root', default='/home/xr/workspace/zbl_arm_6a_datacol/data',
                   help='数据集根目录(容器内路径, 建议保持挂载到仓库内)')
    p.add_argument('--action-mode', choices=['ee', 'joints'], default='ee',
                   help='动作空间: ee=末端xyz+6D姿态+夹爪(10维), '
                        'joints=6关节+夹爪(7维)')
    p.add_argument('--fps', type=float, default=20.0,
                   help='数据集目标帧率(默认20, 与模型预训练一致, '
                        '各流最近邻对齐重采样)')
    p.add_argument('--trim', type=float, default=0.5,
                   help='每集掐头去尾秒数(默认0.5, 0关闭)')
    p.add_argument('--no-rec', action='store_true',
                   help='禁用sw1手柄按键的数采开/停功能')
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
            node.hold_slave(2.0)   # 激活后立即保持当前位姿, 消除旧目标窜动
        if not args.no_gravity_comp:
            if not args.no_master_zero:
                node.zero_master(move_slave=args.engage, notice=0.0)
            node.enable_master_gravity()
        if args.engage:
            if not node.prealign(node.master_joints):
                raise SystemExit("预对齐失败")
            node._follow_enabled = True
            engaged = True
            node.log("=== 实时跟随已启动(Ctrl+C 安全退出) ===")
        ex = SingleThreadedExecutor()
        ex.add_node(node)
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        log_line("退出中: 等待在途数据写入完成(视频编码可能需数十秒)...")
        node.wait_rec_writes()
        if activated:
            node.soft_stop()
        node.rec_stop(sync=True)   # 若在录制中, 同步写完最后一集再退出
        rx_proc.terminate()
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass  # 信号已触发rclpy关闭时二次shutdown会报错, 忽略
        log_line("桥接已退出")


if __name__ == '__main__':
    main()
