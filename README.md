# 自变量六轴臂(zbl_arm_6a) 单臂数采 Pipeline(主臂→从臂遥操 + LeRobot录制)

链路: 主臂PC(192.168.31.7, rosbridge:9090, /joint_states 约197Hz)
   →→ 本机SDK容器内 master_bridge.py(100Hz, 限幅保护)
   →→ 从臂 /joint_position_controller/commands + /gripper_controller/commands

| 位置 | 路径 |
|---|---|
| 脚本源(宿主机) | ~/workspace/zbl_arm_6a_datacol/ |
| 运行位置(容器内) | /home/xr/workspace/zbl_arm_6a_datacol/ |

整个仓库目录**直接挂载**进容器(两侧即时互通, 宿主机是唯一真源)。

## 0. 首次部署(新机器只做一次)

**0.1 获取镜像**(二选一):
```bash
# 方式A(推荐): 从GitHub Container Registry拉取
docker pull ghcr.io/tangyang0/zbl_arm_6a_datacol:v0.1.0
docker tag ghcr.io/tangyang0/zbl_arm_6a_datacol:v0.1.0 zbl_arm_6a_datacol_image:v0.1.0

# 方式B(兜底): 本机已有厂家镜像 zbl_arm_6a_sdk_image:v0.1.1 时一键重建等价镜像
bash build_image.sh
```

**0.2 克隆仓库并创建容器**(宿主机):
```bash
git clone https://github.com/tangyang0/zbl_arm_6a_datacol.git ~/workspace/zbl_arm_6a_datacol
chmod -R a+rwX ~/workspace/zbl_arm_6a_datacol   # 开放权限(容器uid与本机用户不同)
docker run -d --name zbl_arm_6a_sdk --network host \
  --privileged --shm-size 2g \
  -v /dev:/dev:rw -v /etc/localtime:/etc/localtime:ro \
  -v /etc/timezone:/etc/timezone:ro -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /home/$USER/workspace/zbl_arm_6a_datacol:/home/xr/workspace/zbl_arm_6a_datacol \
  -e ROS2_DISTRO=jazzy -e DISPLAY=:1 -e QT_X11_NO_MITSHM=1 \
  -e DEBIAN_FRONTEND=noninteractive -w /home/xr \
  zbl_arm_6a_datacol_image:v0.1.0 \
  bash -i -c "echo 'Development environment ready!' && while sleep 1000; do :; done"
```

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

那个终端已关闭(后台任务变孤儿进程)时, 按阶梯升级杀:
```bash
# 1.直接给控制节点发INT(最有效; 只杀ros2 launch可能卡在等子进程退出,
#   实测运行数天的launch树INT送达后仍僵着, controller_manager不退出)
pkill -INT -f x2robot_control_node
pkill -INT -f "ros2 launch.*zbl_arm_6a"
sleep 3
# 2.确认无残留, 有输出就没杀干净
pgrep -af "ros2 launch|x2robot|usb_cam"
# 3.仍有残留才补刀KILL(launch没机会优雅清理, 但此时子进程也一并杀, 无孤儿化风险)
pkill -9 -f "ros2 launch.*zbl_arm_6a|x2robot_control_node|usb_cam"
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
cd /home/xr/workspace/zbl_arm_6a_datacol
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
> 干跑启动同样会执行: 主臂缓动回零 → 主臂重力补偿(干跑不控制从臂,
> 跳过用 --no-master-zero / --no-gravity-comp)。

## 3. 实时遥操作

**确认从臂工作空间无人无物后**:
```bash
python3 master_bridge.py --engage
```
自动流程: 从臂切 joint_position_controller 并以当前位姿保持2s(覆盖旧目标,
防激活窜动) → 保持结束**立即**主从臂同步限速缓动回零(默认0.3rad/s,
--master-zero-vmax可调, --no-master-zero跳过) → 主臂重力补偿
→ 残余预对齐(回零后通常瞬间完成) → 100Hz 实时跟随。
主臂急动被 vmax=3.0rad/s 软限; 更温和用 --vmax 1.5。

**主臂手柄按键布局**(实机测定; sw编号是固件逻辑名,与物理排列不一致):

| 物理位置(左→右) | 数据字段 | 功能 |
|---|---|---|
| 第1个 | sw2 | **主臂限速回零**(短按, 非录制中; 归零时松手, 完成自动恢复重力补偿) |
| 第2个 | sw1 | **开录**(空闲时; 若有未保存的上一集=丢弃重录) |
| 第3个 | sw3 | **结束本集**(数据挂起待保存) |
| 第4个 | sw4 | **确认保存**(把挂起的本集写入数据集) |
| 扳机 | trigger | 夹爪比例开合(0~1) |
| 摇杆 | joystick_x/y | 空闲(单臂不用; 厂家整机用于底盘/头部) |

急停(容器内随时执行, 从臂停止接受指令):
```bash
ros2 control switch_controllers --deactivate joint_position_controller cart_pose_controller zero_force_dragging_controller
```

## 4. 参数速查(master_bridge.py)

| 参数 | 默认 | 说明 |
|---|---|---|
| --side | right | 用右主臂还是左主臂 |
| --master | ws://192.168.31.7:9090 | 主臂PC rosbridge地址 |
| --engage | 关 | 不加只干跑打印, 加了才真控制 |
| --rate | 100 | 从臂指令频率Hz |
| --vmax | 3.0 | 跟随软限最大关节角速度rad/s |
| --task | demo | 数采任务指令(自然语言, 见§7) |
| --dataset | zbl_data | 数据集分组名(其下按任务自动分目录) |
| --action-mode | ee | ee=末端10维; joints=关节7维 |
| --fps | 20 | 数据集帧率(与模型预训练一致) |
| --trim | 0.5 | 每集掐头去尾秒数 |
| --master-zero-vmax | 0.3 | 主从回零缓动限速rad/s |
| --align-vmax | 0.3 | 残余预对齐缓动限速rad/s |
| --no-master-zero / --no-gravity-comp / --no-rec | 关 | 跳过启动回零/重力补偿/关闭sw1录制 |
| --calib-trigger | 关 | 启动时交互标定扳机行程 |

## 5. 注意事项

- 主臂数据流中断>0.5s自动悬停, 恢复后限速追平; Ctrl+C 自动软急停。
- 同一从臂只允许一个控制源(勿同时跑示例代码/其他控制节点)。
- WiFi 抖动(本机↔主臂PC ping 3~100ms)导致跟手卡顿的话, 两台机器接有线。
- 主臂若与从臂镜像摆放, 需在 clamp_step 前加符号映射。

## 6. 镜像与维护

镜像 `zbl_arm_6a_datacol_image:v0.1.0`(初版数采镜像, 6.73GiB):
厂家v0.1.1 + torch-cpu/lerobot==0.4.4/websockets/websocket-client
(数采录制依赖) + git跨属主safe.directory例外, 单层压平。

`build_image.sh` = 用厂家镜像一键重建等价数采镜像的脚本
(装依赖 → safe.directory → 清缓存 → export/import压平),
厂家更新SDK或依赖变化时改脚本内版本号重跑, 递增tag(如v0.2.0)。
新版本镜像推到ghcr.io供学生pull(见第0节):
```bash
docker tag zbl_arm_6a_datacol_image:v0.2.0 ghcr.io/tangyang0/zbl_arm_6a_datacol:v0.2.0
docker push ghcr.io/tangyang0/zbl_arm_6a_datacol:v0.2.0
```

权限说明: 宿主机仓库已整体 `chmod -R a+rwX`(容器uid与本机用户不同, 不开放
则容器内写不了); **宿主机侧 git pull/checkout 后新文件权限是默认umask,
需补一句 `chmod -R a+rwX ~/workspace/zbl_arm_6a_datacol`** 再让容器内编辑。

## 7. 数据采集(数采) → wall-x 训练数据

**直接录成 LeRobot v3 格式**(wall-x训练格式, 无需事后转换):

```bash
# 容器内启动遥操, --task 用自然语言任务指令(VLA微调任务标签):
python3 master_bridge.py --engage --task "pick up test tube No. 1 from the left rack and place it in the right rack"
# 遥操中每集三键流程: sw1=开录 -> sw3=结束本集(挂起待存) -> sw4=确认写入
#   - 不按sw4直接再按sw1 = 丢弃上一集, 直接重录
#   - Ctrl+C退出时: 录制中的集同步写完, 挂起未确认的集自动保存
#   - sw4写入在后台进行(视频编码需数秒), 不影响继续遥操
# 数据直接落在仓库 data/ 目录(仓库整体挂载, 宿主机即时可见, 不需拷贝):
#   ~/workspace/zbl_arm_6a_datacol/data/zbl_data/<任务名>/
#   每个任务名目录是一个独立完整的LeRobot数据集(同任务自动增量追加)
#   训练时 repo_id 指到任务目录 -> wall-x compute_norm_stats.py -> 训练
```

录制相关参数:
- `--task "..."` 自然语言任务标签(进LeRobot tasks元数据)
- `--dataset zbl_data` 数据集分组名(其下按 `--task` 自动建任务子目录,
  每个任务目录=独立数据集, 同任务增量追加)
- `--action-mode ee|joints`  ee(默认)=末端xyz+6D姿态+夹爪10维;
  joints=6关节+夹爪7维
- `--fps 20` 目标帧率(与模型预训练一致; 相机/关节/位姿各流最近邻对齐重采样)
- `--trim 0.5` 每集掐头去尾秒数(0关闭)
- 注意: 同一数据集内图像流需一致(中途开/关过相机就换个 `--dataset` 分组名)

数据集内容: observation.state(10=末端xyz+6D姿态+夹爪, 与预训练agent_pos同构),
observation.joints(7=6关节+夹爪归一化, 备用), action(10=从臂末端绝对位姿,
与state同一条从臂流错开一帧, action[i]≡state[i+1], 官方x2robot2lerobot同语义;
训练侧loader自动转delta; joints模式为7), observation.images.wrist(相机视频),
任务标签来自--task自然语言指令。
训练侧: 把 `data/zbl_data/<任务名>/` 整个目录拷到训练机, 用wall-x的
scripts/compute_norm_stats.py算归一化统计后即可训练。
