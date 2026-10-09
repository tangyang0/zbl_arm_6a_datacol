#!/usr/bin/env python3
"""手动夹爪控制(过渡用): 主臂扳机映射完成前, 用它开合夹爪

在从臂SDK容器内运行:
    python3 gripper_pub.py 0.0     # 闭合
    python3 gripper_pub.py 1.2     # 张开(范围0~4.5 rad, 双边行程0~78.75mm)
"""
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


def main():
    if len(sys.argv) != 2:
        raise SystemExit(__doc__)
    width = float(sys.argv[1])

    rclpy.init()
    node = Node('gripper_pub')
    pub = node.create_publisher(
        Float64MultiArray, '/gripper_controller/commands', 10)
    time.sleep(0.5)
    msg = Float64MultiArray()
    msg.data = [width]
    # 连发几帧确保指令到达
    for _ in range(10):
        pub.publish(msg)
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()
    print(f"夹爪指令已发送: {width} rad")


if __name__ == '__main__':
    main()
