#!/usr/bin/env python3
"""主从臂限速缓动回零(独立工具, 不需要master_bridge在跑)

主臂: 经rosbridge切关节位置伺服 -> 订阅/joint_states取反馈 -> 50Hz限速缓动回零
      -> 恢复重力补偿(臂在零位释放为自由)
从臂: 切joint_position_controller(含夹爪控制器重置) -> 当前位姿保持 -> 与主臂
      同一条缓动曲线同步回零 -> 结束软急停
两侧同时运动, 时长 = 两侧最大偏差 / 限速。

用法(容器内, SDK已launch):
  python3 home.py                  # 主从臂同步回零
  python3 home.py --vmax 0.15      # 更慢
  python3 home.py --no-master      # 只回零从臂
  python3 home.py --no-slave       # 只回零主臂
"""
import argparse
import json
import socket
import time

import numpy as np

SLAVE_LOWER = [-2.792, 0.0, -3.14, -1.57, -1.4, -1.745]
SLAVE_UPPER = [2.792, 3.44, 0.0, 1.57, 1.4, 1.745]


def log(msg):
    print(msg, flush=True)


class MasterArm:
    """主臂: rosbridge上的控制器切换+关节反馈+指令下发"""

    def __init__(self, url, side, vmax, tol):
        import websocket as _ws
        self.side, self.vmax, self.tol = side, vmax, tol
        self.names = [f"master_{side}_arm_joint{i}" for i in range(1, 7)]
        self.pos = None
        log(f"连接主臂 rosbridge {url} ...")
        try:
            self.ws = _ws.create_connection(
                url, timeout=5,
                sockopt=((socket.IPPROTO_TCP, socket.TCP_NODELAY, 1),))
        except Exception as e:
            raise SystemExit(f"主臂rosbridge连接失败({e}) —— 检查主臂PC电源/网络;"
                             f" 只回零从臂请加 --no-master")
        self.ws.send(json.dumps({"op": "subscribe", "topic": "/joint_states",
                                 "throttle_rate": 20, "queue_length": 1}))

    def _switch(self, activate, deactivate, strictness=1):
        self.ws.send(json.dumps({
            "op": "call_service", "id": "sw",
            "service": "/controller_manager/switch_controller",
            "args": {"activate_controllers": activate,
                     "deactivate_controllers": deactivate,
                     "strictness": strictness,
                     "activate_asap": False,
                     "timeout": {"sec": 3, "nanosec": 0}}}))
        t0 = time.time()
        while time.time() - t0 < 5.0:
            try:
                m = json.loads(self.ws.recv())
            except (TimeoutError, Exception):
                continue
            if m.get("id") == "sw":
                return (m.get("values") or {}).get("ok", False)
        return None

    def pump(self, wait=0.0):
        """非阻塞收一帧/joint_states, 更新self.pos(主臂6关节)"""
        try:
            m = json.loads(self.ws.recv())
        except Exception:
            return
        if m.get("op") != "publish" or m.get("topic") != "/joint_states":
            return
        msg = m.get("msg", {})
        try:
            idx = [msg["name"].index(n) for n in self.names]
            pos = [float(msg["position"][i]) for i in idx]
        except (ValueError, KeyError, IndexError):
            return
        import math
        if all(math.isfinite(v) for v in pos):
            self.pos = np.array(pos)

    def servo_on(self):
        jc = f"master_{self.side}_arm_joint_controller"
        gc = f"{self.side}_gravity_compensate_controller"
        cart = f"master_{self.side}_arm_cartesian_controller"
        ok = self._switch([jc], [gc, cart])
        if ok is not True:
            ok = self._switch([jc], [], strictness=4)
        if ok is not True:
            raise SystemExit("主臂关节位置伺服激活失败")
        self.topic = f"/{jc}/commands"
        self.ws.send(json.dumps({"op": "advertise", "topic": self.topic,
                                 "type": "std_msgs/msg/Float64MultiArray"}))

    def send(self, q):
        self.ws.send(json.dumps({"op": "publish", "topic": self.topic, "msg": {
            "layout": {"dim": [], "data_offset": 0},
            "data": [float(v) for v in q]}}))

    def gravity_comp(self):
        gc = f"{self.side}_gravity_compensate_controller"
        jc = f"master_{self.side}_arm_joint_controller"
        cart = f"master_{self.side}_arm_cartesian_controller"
        ok = self._switch([gc], [jc, cart])
        if ok is not True:
            ok = self._switch([gc], [], strictness=4)
        log("主臂已恢复重力补偿" if ok
            else "主臂重力补偿恢复失败, 请手动处理")
        return ok

    def close(self):
        try:
            if getattr(self, "topic", None):
                self.ws.send(json.dumps({"op": "unadvertise",
                                         "topic": self.topic}))
            self.ws.close()
        except Exception:
            pass


class SlaveArm:
    """从臂: rclpy节点, 控制器切换(含夹爪重置)/反馈/指令"""

    def __init__(self, tol):
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Float64MultiArray
        from controller_manager_msgs.srv import SwitchController
        rclpy.init()
        self.rclpy = rclpy
        self.tol = tol
        self.node = Node('home_zero')
        self.jnames = [f"arm_joint{i}" for i in range(1, 7)]
        self.pos = None
        self.grip = None
        self.jpub = self.node.create_publisher(
            Float64MultiArray, '/joint_position_controller/commands', 10)
        self.gpub = self.node.create_publisher(
            Float64MultiArray, '/gripper_controller/commands', 10)
        self.cli = self.node.create_client(
            SwitchController, '/controller_manager/switch_controller')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            raise SystemExit("switch_controller 服务不可用(SDK起了吗?)")

        def on_js(msg):
            try:
                idx = [msg.name.index(n) for n in self.jnames]
                self.pos = np.array([msg.position[i] for i in idx])
                self.grip = msg.position[msg.name.index("arm_gripper")]
            except ValueError:
                pass

        self.node.create_subscription(JointState, '/joint_states', on_js, 10)

    def spin(self, t=0.0):
        self.rclpy.spin_once(self.node, timeout_sec=t)

    def switch(self, activate, deactivate):
        from controller_manager_msgs.srv import SwitchController
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.BEST_EFFORT
        fut = self.cli.call_async(req)
        self.rclpy.spin_until_future_complete(self.node, fut, timeout_sec=5.0)
        ok = fut.result() is not None and fut.result().ok
        log(f"切换控制器 activate={list(activate)} "
            f"deactivate={list(deactivate)} -> {ok}")
        return ok

    def wait_state(self, timeout=3.0):
        t0 = time.time()
        while time.time() - t0 < timeout and self.pos is None:
            self.spin(0.05)
        if self.pos is None:
            raise SystemExit("未收到从臂/joint_states")
        return self.pos

    def send(self, q, grip_rad=0.0):
        from std_msgs.msg import Float64MultiArray
        jm = Float64MultiArray()
        jm.data = [float(np.clip(v, lo, hi)) for v, lo, hi
                   in zip(q, SLAVE_LOWER, SLAVE_UPPER)]
        self.jpub.publish(jm)
        gm = Float64MultiArray()
        gm.data = [float(grip_rad)]
        self.gpub.publish(gm)

    def soft_stop(self):
        self.switch([], ['joint_position_controller', 'cart_pose_controller',
                         'zero_force_dragging_controller'])

    def close(self):
        self.node.destroy_node()
        self.rclpy.shutdown()


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--side', choices=['left', 'right'], default='right')
    p.add_argument('--master', default='ws://192.168.31.7:9090',
                   help='主臂PC rosbridge地址')
    p.add_argument('--vmax', type=float, default=0.3, help='缓动限速rad/s(默认0.3)')
    p.add_argument('--tol', type=float, default=0.05, help='到位判据rad(默认0.05)')
    p.add_argument('--no-master', action='store_true', help='不回零主臂')
    p.add_argument('--no-slave', action='store_true', help='不回零从臂')
    p.add_argument('--no-gc', action='store_true',
                   help='主臂回零后不恢复重力补偿(保持位置伺服)')
    args = p.parse_args()

    log("★ 1秒后主从臂将限速缓动回零, 请勿手持主臂, 确保周围无人!")
    time.sleep(1.0)

    master = slave = None
    m0 = q0 = None
    try:
        if not args.no_master:
            master = MasterArm(args.master, args.side, args.vmax, args.tol)
            master.servo_on()
            t0 = time.time()
            while time.time() - t0 < 2.0 and master.pos is None:
                master.pump()
                time.sleep(0.01)
            if master.pos is None:
                raise SystemExit("主臂关节数据流无反馈(检查rosbridge/主臂PC)")
            m0 = master.pos
        if not args.no_slave:
            slave = SlaveArm(args.tol)
            # 先全退(含夹爪重置)再进, 防夹爪伺服"假活"(强杀SDK后常见)
            slave.switch([], ['cart_pose_controller',
                              'zero_force_dragging_controller',
                              'gripper_controller'])
            time.sleep(0.5)
            if not slave.switch(['joint_position_controller', 'gripper_controller'],
                                []):
                raise SystemExit("从臂关节位置模式激活失败")
            q0 = slave.wait_state()
            log("从臂以当前位姿保持1s...")
            t0 = time.time()
            while time.time() - t0 < 1.0:
                slave.send(q0, 0.0)
                slave.spin()
                time.sleep(0.02)

        m_max = float(np.max(np.abs(m0))) if m0 is not None else 0.0
        s_max = float(np.max(np.abs(q0))) if q0 is not None else 0.0
        dur = max(m_max, s_max) / args.vmax
        msg = f"开始同步回零: 限速{args.vmax}rad/s, 预计{dur:.1f}s ("
        msg += f"主臂最大{np.degrees(m_max):.0f}°" if m0 is not None else "无主臂"
        msg += ", " if m0 is not None and q0 is not None else ""
        msg += f"从臂最大{np.degrees(s_max):.0f}°" if q0 is not None else "无从臂"
        log(msg + ")")

        t0 = time.time()
        period = 1.0 / 50.0
        next_t = time.perf_counter()
        err_m = err_s = None
        while time.time() - t0 < dur + 20.0:
            s = max(0.0, 1.0 - (time.time() - t0) / dur) if dur > 0 else 0.0
            if master is not None:
                master.send(m0 * s)
                master.pump()
                if master.pos is not None:
                    err_m = float(np.max(np.abs(master.pos)))
            if slave is not None:
                slave.send(q0 * s, 0.0)
                slave.spin()
                if slave.pos is not None:
                    err_s = float(np.max(np.abs(slave.pos)))
            done = (s <= 0.0
                    and (master is None or (err_m is not None and err_m < args.tol))
                    and (slave is None or (err_s is not None and err_s < args.tol)))
            if done:
                break
            next_t += period
            dt = next_t - time.perf_counter()
            if dt > 0:
                time.sleep(dt)
            else:
                next_t = time.perf_counter()

        res = []
        if master is not None:
            res.append(f"主臂偏差{err_m:.3f}rad" if err_m is not None else "主臂无反馈")
        if slave is not None:
            res.append(f"从臂偏差{err_s:.3f}rad" if err_s is not None else "从臂无反馈")
        log(f"回零完成({', '.join(res)}, 耗时{time.time() - t0:.1f}s)")
    except KeyboardInterrupt:
        log("\n中断...")
    finally:
        if master is not None:
            master.close()
            if not args.no_gc:
                master.gravity_comp()
        if slave is not None:
            log("从臂保持1s后软急停...")
            t0 = time.time()
            while time.time() - t0 < 1.0:
                slave.send(np.zeros(6), 0.0)
                slave.spin()
                time.sleep(0.02)
            slave.soft_stop()
            slave.close()
        log("退出")


if __name__ == '__main__':
    main()
