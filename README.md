# 自变量六轴臂(zbl_arm_6a) 单臂遥操作 Pipeline(主臂 → 从臂)

链路: 主臂PC(192.168.31.7, rosbridge:9090, /joint_states 约197Hz)
   →→ 本机SDK容器内 master_bridge.py(100Hz, 限幅保护)
   →→ 从臂 /joint_position_controller/commands + /gripper_controller/commands

| 位置 | 路径 |
|---|---|
| 脚本源(宿主机) | /home/tangyang/workspace/zbl_arm_6a_teleop/ |
| 运行位置(容器内) | /home/xr/workspace/zbl_arm_6a_teleop/ |

两侧是独立拷贝(**没有挂载**), 宿主机改完脚本后执行同步(见第6节)。

## 1. 上电启动(每次)

宿主机:
```bash
docker restart zbl_arm_6a_sdk
docker exec -it zbl_arm_6a_sdk bash
ros2 launch zbl_arm_6a_description test_single_arm.launch.py can_interface:=can4 &
```
主臂PC开机即可,无需在其WebUI上做任何操作(它常驻发布关节流)。

### 1.1 停止/重启 SDK 进程(机械臂断电重上电后, 不用重启容器)

在当初运行 `&` 的那个终端里:
```bash
jobs                 # 看到 [1]+ Running ros2 launch ...
kill -INT %1         # 等价于前台Ctrl+C, 优雅关闭全部节点(controller_manager/相机等)
```

那个终端已关闭(后台任务变孤儿进程)时:
```bash
pgrep -af "ros2 launch"                    # 找到PID
kill -INT <PID>
# 或一条命令:
pkill -INT -f "ros2 launch.*zbl_arm_6a"
```

确认清理干净后重新 launch:
```bash
pgrep -af "x2robot|usb_cam|controller_manager"   # 应无残留进程
ros2 launch zbl_arm_6a_description test_single_arm.launch.py can_interface:=can4 &
```

注意:
- **不要用 `kill -9`**: launch 进程没机会清理子节点, 会留下孤儿进程,
  此时再 launch 就是SDK README警告的"同一臂多套控制服务", 臂会异常抖动。
  不慎发生时补刀: `pkill -9 -f x2robot_control_node; pkill -9 -f usb_cam`
- 机械臂断电重上电不影响容器和宿主机 can4 接口(PCAN由USB供电),
  清干净进程后直接重新 launch 即可, 无需动容器。

## 2. 干跑验证(不动从臂, 安全)

容器内:
```bash
cd /home/xr/workspace/zbl_arm_6a_teleop
python3 master_bridge.py              # 右主臂(默认); 左臂加 --side left
```

> 环境说明: 容器 `~/.bashrc` 末尾已自动执行
> `source /opt/ros/jazzy/setup.bash`、`source /opt/xr/core/install/setup.bash`、
> `export LD_LIBRARY_PATH=...:/usr/local/lib`、`export ROS_DOMAIN_ID=134`,
> 所以**交互式终端**(docker exec -it ... bash)不需要手动 source。
> 只有绕过 bash 非交互执行时才需要, 例如:
> `docker exec zbl_arm_6a_sdk bash -c 'source /opt/xr/core/install/setup.bash && python3 ...'`
手持主臂缓慢运动, 确认打印的关节角跟随、数值在从臂限位内:
J1 ±2.792, J2 0~3.44, J3 -3.14~0, J4 ±1.57, J5 ±1.4, J6 ±1.745 (rad)
Ctrl+C 退出。

## 3. 实时遥操作

**确认从臂工作空间无人无物后**:
```bash
python3 master_bridge.py --engage
```
自动流程: 切 joint_position_controller → 2s缓冲 → 低速预对齐到主臂当前构型
→ 100Hz 实时跟随。主臂急动被 vmax=1.5rad/s 软限; 更温和用 --vmax 1.0。

夹爪(扳机映射接入前的手动方式, 另开一个容器终端):
```bash
python3 gripper_pub.py 1.2   # 张开(0~4.5)
python3 gripper_pub.py 0.0   # 闭合
```

急停(容器内随时执行, 从臂停止接受指令):
```bash
ros2 control switch_controllers --deactivate joint_position_controller cart_pose_controller zero_force_dragging_controller
```

## 4. 参数速查(master_bridge.py)

| 参数 | 默认 | 说明 |
|---|---|---|
| --side | right | 用右主臂还是左主臂 |
| --master | ws://192.168.31.7:9090 | 主臂PC rosbridge地址 |
| --rate | 100 | 从臂指令频率Hz |
| --vmax | 1.5 | 软限最大关节角速度rad/s |
| --engage | 关 | 不加只干跑打印, 加了才真控制 |

## 5. 注意事项

- 主臂数据流中断>0.5s自动悬停, 恢复后限速追平; Ctrl+C 自动软急停。
- 同一从臂只允许一个控制源(勿同时跑示例代码/其他控制节点)。
- WiFi 抖动(本机↔主臂PC ping 3~100ms)导致跟手卡顿的话, 两台机器接有线。
- 第一次真机遥操作时, 桥接会打印扳机话题(/joy_trigger/state/*)载荷格式,
  拿到格式后补 0~4.5rad 映射即可去掉手动夹爪。
- 主臂若与从臂镜像摆放, 需在 clamp_step 前加符号映射。

## 6. 宿主机 → 容器 同步脚本

宿主机改了脚本后, 执行一次即可让容器内生效:

```bash
docker cp /home/tangyang/workspace/zbl_arm_6a_teleop/. zbl_arm_6a_sdk:/home/xr/workspace/zbl_arm_6a_teleop/
```

容器重建/删除后, 同样执行上面这条同步, 再补装一次依赖:

```bash
docker exec zbl_arm_6a_sdk bash -c 'mkdir -p /home/xr/workspace/zbl_arm_6a_teleop && pip3 install websockets'
```

(数采 record_data.sh 暂缓使用, 遥操跑顺后再启用。)
