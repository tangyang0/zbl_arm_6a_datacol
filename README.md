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

> **与厂家《自变量六轴臂SDK开发指南.pdf》的分工**(先看这个再动手):
> - **先按指南做**: 1.1 环境检查、1.2 连接手臂、1.3 连接相机、1.4.3 安装Docker;
> - **本文档已替代指南 1.4 全节**: 下载代码(1.4.1)、CAN脚本(1.4.2, 本机为
>   PCAN自动枚举, 接口名用 `ip link | grep can` 确认, 本机是 can4)、
>   镜像加速(1.4.4)、compose构建镜像(1.4.5)、Dev Container(1.4.6)
>   都不需要; 相机权限chmod与launch命令已含在本文档流程;
> - 指南 2.x 接口说明仅作参考手册按需查阅。

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

### 1.0 启动自检: 控制器状态(SDK冷启动约30s后执行)

```bash
ros2 control list_controllers
```

**正常待机状态**(三项active, 三项inactive):
```
gripper_controller             x2robot_controllers/JointImpedanceController     active
joint_state_broadcaster        joint_state_broadcaster/JointStateBroadcaster    active
endpose_broadcaster            x2robot_controllers/EndposeBroadcaster           active
cart_pose_controller           x2robot_controllers/QPIKController               inactive
joint_position_controller      x2robot_controllers/JointImpedanceController     inactive
zero_force_dragging_controller x2robot_controllers/GravityCompensateController  inactive
```

异常处理: gripper_controller 非 active **或夹爪发软**(显示active但伺服未使能,
SDK被强杀后重启常见) → 按第5节方法重置; 遥操启动后 joint_position_controller
会变 active 属正常, 退出软急停后回到上表状态。

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
| --master-ee-topic | auto | 主臂EE的PoseStamped话题；自动识别失败时填已确认的实际话题 |
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
- **夹爪发软/不响应指令** = gripper伺服掉线(SDK进程被强杀后重启时常见,
  控制器显示active但未真正使能)。处理: `ros2 control switch_controllers
  --deactivate gripper_controller && ros2 control switch_controllers
  --activate gripper_controller`, 无效则重启SDK栈; master_bridge与replay
  启动时已自动重置夹爪控制器。
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
python3 master_bridge.py --engage --dataset zbl_data_v4 --gripper-max 1.70 \
  --task "pick up test tube No. 1 from the left rack and place it in the right rack"
# 遥操中每集三键流程: sw1=开录 -> sw3=结束本集(挂起待存) -> sw4=确认写入
#   - 不按sw4直接再按sw1 = 丢弃上一集, 直接重录
#   - Ctrl+C退出时: 录制中的集同步写完, 挂起未确认的集自动保存
#   - sw4写入在后台进行(视频编码需数秒), 不影响继续遥操
# 数据直接落在仓库 data/ 目录(仓库整体挂载, 宿主机即时可见, 不需拷贝):
#   ~/workspace/zbl_arm_6a_datacol/data/zbl_data_v4/<任务名>/
#   每个任务名目录是一个独立完整的LeRobot数据集(同任务自动增量追加)
#   训练时 repo_id 指到任务目录 -> wall-x compute_norm_stats.py -> 训练
```

**回放验证(数据质检, 与遥操互斥勿同时跑)**:
```bash
python3 replay.py --list                       # 列出仓库内全部数据集
python3 replay.py data/zbl_data/<任务目录>      # 干跑: 打印episode摘要
python3 replay.py data/zbl_data_v4/<任务目录> --engage         # 新数据从meta读取夹爪比例
python3 replay.py data/zbl_data/<任务目录> --episode 0 --speed 0.5 --grip-max <当时实际采集上限rad> --engage
# 回放的是observation.joints=录制时从臂真实轨迹; 开始/结束自动限速缓动回零
# (--no-home-start/--no-home-end跳过), 自带预对齐/限幅/软急停
```

录制相关参数:
- `--task "..."` 自然语言任务标签(进LeRobot tasks元数据)
- `--dataset zbl_data` 数据集分组名(其下按 `--task` 自动建任务子目录,
  每个任务目录=独立数据集, 同任务增量追加)
- `--action-mode ee|joints`  ee(默认)=末端xyz+6D姿态+夹爪10维;
  joints=6关节+夹爪7维
- `--fps 20` 目标整数帧率；相机/关节/位姿各流最近邻重采样，执行端应保持对应时间尺度
- `--trim 0.5` 每集掐头去尾秒数(0关闭)
- 注意: 同一数据集内图像流需一致(中途开/关过相机就换个 `--dataset` 分组名)

### 7.1 新采集记录与旧数据兼容

此次采集 schema 升至 3：包含 `command.*`、原始夹爪反馈、主臂EE和对应诊断列，模型使用的
`observation.state/action` 仍为原来的 10 维，`observation.joints` 仍为 7 维。
**已有旧 schema（包括只含原 `acq.*` 的上一版）会在开录前拒绝追加**，请使用
全新分组名（例如 `--dataset zbl_data_v4`）；程序不会回填或改写旧
50 条轨迹、旧 `norm_stats.json` 和 checkpoint。

夹爪数据是实测 `arm_gripper / --gripper-max` 后的 `[0,1]` 比例。命令中的
1.70 rad 是本次采集选择的行程上限示例，不代表硬件统一物理满开值；应核验后设置。
同一数据集的上限、动作模式、帧率、时钟模式和标定编号必须一致。新数据会保存实际
参数，`replay.py` 自动读取；旧数据真机回放必须显式提供已核实的 `--grip-max`。
回放按 LeRobot v3 的 episode metadata 查找文件，支持同一个 Parquet 包含多个
episode，额外的 `acq.*` 列不会改变回放轨迹。
回放在任何 ROS 导入及控制器操作前检查 `speed/vmax/align-vmax/fps` 为有限正数，
并检查关节轨迹非空、形状为 `[N,7]`、全部有限，六关节在脚本既定 SDK 限位内、
夹爪比例在 `[0,1]` 内；不合格时直接退出。

每个任务目录保持 LeRobot 结构：

```text
<task>/
├── data/                         # state/action、反馈、对齐命令及acq.*数值列
├── videos/                       # 编码后的相机视频
├── meta/
│   ├── info.json、stats.json     # 由LeRobot管理
│   ├── episodes/、tasks.parquet  # 由LeRobot管理
│   └── acquisition/              # 本项目扩展，不是LeRobot标准必填字段
│       ├── dataset_contract.json
│       ├── episode_000000.json
│       └── episode_000000.streams.jsonl  # 全频率命令/关节反馈/主臂EE
└── images/                       # 仅编码临时PNG，成功后删除空父目录
```

仅当 `save_episode()` 和 `finalize()` 都成功，才用 `rmdir` 清理空的相机父目录及
最外层 `images/`。非空缓存、符号链接和编码失败的 PNG 不会递归删除。写入期间保留
`.pending.json`，完整保存后转为正式 episode JSON；若存在未完成记录，会拒绝追加，
应先检查部分写入内容，不能直接删除缓存后当作成功。
数据集目录自身是符号链接时也不清理。后台 writer 在启动或排队前登记未完成计数，
退出等待以一次总超时覆盖全部已登记 writer，避免“线程已启动、尚未抢到写锁”时提前退出。

`acq.*` 逐帧记录原始 header 纳秒、主机接收 monotonic 纳秒、源样本序号、匹配误差、
匹配帧龄、相邻源帧间隔及有效标记。`acq.action_pose.*` 和 `acq.action_joints.*`
分别追溯下一采样时刻的动作目标，包含最后一条 action 对应的未保存观测。整数时间
以 `int64` 保存，不混入 10 维模型输入，也不生成训练归一化统计。

episode JSON 保存实际 CLI、源码 SHA/dirty 状态及源码文件 hash、topic/frame_id、
单位、夹爪上限、时钟域、质量摘要和 `--calibration-id`。标定 ID 仅引用外部核验记录；
采集程序不自动验证物理坐标轴/工具零姿态，也不应用额外坐标变换。

### 7.1.1 同时保存下发命令和从臂反馈

确认保存（sw4或退出时自动保存）后，每个训练帧的 `data/` Parquet 包含：

| 字段 | 形状、单位 | 含义 |
|---|---|---|
| `observation.joints` | 7；前6维rad，末维[0,1] | 从臂实测关节角及归一化夹爪反馈 |
| `observation.gripper_position` | 1；rad，float64 | 未归一化、未裁剪的夹爪反馈，便于直接比较跟踪误差 |
| `observation.state` | 10；xyz米+rotation6D+夹爪比例 | 当前实测末端状态，模型输入定义不变 |
| `observation.master_ee_pose` | 7；xyz米+四元数xyzw，float64 | 主臂发布的EE位姿，保留它的原坐标系；不拼进模型state |
| `command.joints` | 6；rad，float64 | 最近一次成功调用publish的从臂关节目标，已经过控制循环的限位/限速处理 |
| `command.gripper` | 1；rad，float64 | 最近一次成功调用publish的夹爪目标，已经过限速/裁剪处理 |
| `action` | ee模式10；joints模式7 | 下一采样时刻的实测状态，仍不是原始下发命令 |

关节顺序始终为 `arm_joint1` 到 `arm_joint6`；夹爪为 `arm_gripper`。
命令使用**从臂关节反馈接收时刻之前最后一次发布值**进行因果保持，不取未来最近邻。
即使 `--sync-clock header`，该关联仍用本机 monotonic 时间；它不能证明曝光同步、
控制器收到指令或指令已经执行。每帧各有以下两个诊断字段组：

```text
acq.command_joints.{publish_mono_ns,reference_mono_ns,source_index,age_ns,valid}
acq.command_gripper.{publish_mono_ns,reference_mono_ns,source_index,age_ns,valid}
```

`reference_mono_ns` 是所选关节反馈的接收时间；`age_ns` 是该时刻减去发送时间。
夹爪不变时不会每帧重发，旧目标继续作为最后发布值保留，原发送时间不会改写。
开录时携带已知的最后发布目标，标记为 `carried_from_before_recording=true`；
不会把仅缓存的反馈伪装成命令。尚无对应发布值时，`valid=0`、`source_index=-1`、
`publish_mono_ns=0`、`age_ns=-1`，数值列的零仅为占位，不能用于跟踪误差分析。
`acq.valid` 仍表示观测流对齐质量，命令是否存在须查看各自的 `valid`。

为保留20Hz重采样丢弃的数值消息，另写
`meta/acquisition/episode_000000.streams.jsonl`。这是本项目的原始采集证据扩展，
不在 `data/` 下放入另一套不同频率的 Parquet，避免训练reader误扫为训练帧。
该JSONL保存掐头去尾前的以下四种数值流；各流内按时间和 `sample_index` 排序：

| `stream` | `values` | 时间及额外字段 |
|---|---|---|
| `command.joints` | 原始发布的6个关节目标，rad | `publish_mono_ns`、`sample_index`、`carried_from_before_recording` |
| `command.gripper` | 原始发布的1个夹爪目标，rad | 同上 |
| `feedback.joints` | 6个关节反馈+1个原始夹爪反馈，全部rad | `receive_mono_ns`、原始`header_time_ns`、`frame_id`、`sample_index` |
| `feedback.master_ee_pose` | 主臂xyz米+四元数xyzw，7维 | `receive_mono_ns`、原始`header_time_ns`、`frame_id`、`sample_index`、`carried_from_before_recording` |

这份文件在episode确认保存时写入，不是实时掉电保护日志。记录进程崩溃前尚未保存的
内存数据仍可能丢失。episode JSON的 `raw_streams` 保存相对路径、各流样本数和SHA256；
编码失败时保留原始流和 `.pending.json`，不会标成完整episode。
`command.*` 和 `acq.*` 不加入训练输入，也不生成归一化统计；后续若要用实际命令
训练关节策略，应显式选择这些字段及时间关系。

### 7.1.2 主臂EE位姿的接入与字段

`master_rx.py` 从主臂PC的rosbridge接收EE消息，经本机UDP转发到录制进程。
当前明确支持 `geometry_msgs/PoseStamped`（ROS1/ROS2时间戳字段均支持），保存原始
`[x,y,z,qx,qy,qz,qw]`。不会猜测无类型数组中的Euler顺序/单位，也不会用从臂FK冒充主臂EE。

默认 `--master-ee-topic auto` 通过只读 `/rosapi/topics` 查询，只在唯一候选同时满足
PoseStamped类型、名称包含master/leader及所选left/right，并排除target/goal/command等
明显的指令话题时自动订阅。话题的实际物理含义仍需核验。不能唯一判断，
或主臂没有rosapi服务时，请用 `--master-ee-topic` 填入实际确认的话题名；明确指定话题
不依赖rosapi，但收到的消息结构仍需通过PoseStamped校验。
当前开发环境尚未连通主臂PC，实际话题名和实机发布需现场核验。

未收到新鲜EE时拒绝开录，已有待保存数据不会因此丢弃；这不阻止原来的关节遥操。
若录制中部分EE样本陈旧，会保留最后一次实测值并标 `valid=0`；没有任何新鲜匹配
时拒绝整条episode保存。缺少先前样本时数值零只是占位，不代表有效姿态。
字段如下：

```text
observation.master_ee_pose                           # float64[7]
acq.master_pose.header_time_ns                        # 原主臂ROS stamp，未知为0
acq.master_pose.receive_mono_ns                       # 主臂接收进程收到该消息的本机时刻
acq.master_pose.reference_mono_ns                     # 对齐所用从臂关节反馈接收时刻
acq.master_pose.source_index                         # 原始主臂EE流索引
acq.master_pose.age_ns                               # reference - receive
acq.master_pose.valid                                # 是否有此前样本且年龄在阈值内
```

主臂EE使用同一采集机的monotonic接收时钟进行因果保持，不用主臂ROS stamp与从臂
stamp直接相减；阈值复用 `--sync-max-age-ms`。`frame_id`、topic、消息类型和左右侧
写入episode JSON的 `master_pose_source`，原始流每条也保留frame_id。主臂与从臂
即使出现同名frame_id，也不代表物理坐标系相同；本程序不做主从外参变换。
更换主臂来源或frame定义需要新数据集，episode内来源变化会拒绝保存。
该字段为辅助记录，排除归一化统计，不改变现有从臂 `observation.state/action`。

### 7.2 同步检查

默认 `--sync-clock receive` 使用同一主机的接收 monotonic 时钟，原始消息 header
另行保留。**接收时间接近不等于曝光时刻和机械臂采样时刻一致**；不同 clock domain
的原始时间不能直接相减。程序取所有必需流的时间覆盖交集后，再掐头去尾并构造固定
网格。

以下阈值是可配置的初值，尚不是这套实物系统测量得到的最优参数：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `--sync-max-match-ms` | 50 | 选中源时间与网格时间的最大绝对差 |
| `--sync-max-age-ms` | 100 | 网格相对较旧源样本的最大年龄，不是网络传输延迟 |
| `--sync-max-gap-ms` | 250 | 网格处相邻源样本的最大间隔 |
| `--sync-max-clock-skew-ms` | 100 | header模式各流header→receive中位偏移的最大差 |

任何网格不合格、必需流断流、正的 header 重复/倒退、episode 内 frame_id 改变、
图像尺寸改变或非法数值都会拒绝整条 episode 写入训练集；**不会删掉坏帧再压缩成
看似规则的20Hz序列**。失败会显示诊断并保留当前进程内的 buffer，可按 sw4 重试
或 sw1 明确丢弃；内存 buffer 不会跨进程退出保留。已发生部分文件写入时，还需先
检查 `.pending.json`，不能自动重试覆盖。

只有确认各源 header 使用同一时钟域后，才使用：

```bash
--sync-clock header --header-clock-domain <已核验的共同源时钟域>
```

该模式要求所有 header 有效且递增，并检查各流相对接收时钟偏移的合理一致性；
它仍不代替曝光、驱动延迟和物理同步实验。缺少相机默认拒绝写入；仅非当前 VLA
用途可显式使用 `--allow-no-camera`。

### 7.3 统计兼容与验证

本项目的 `lerobot_stats.py` 适配器固定针对 **LeRobot 0.4.4**：只在受锁保护的
单次保存期间替换统计回调，在 `finally` 恢复，不修改 site-packages 文件。
图像先转 `float64` 再计算平方和，避免 `uint8` 溢出造成 std=0；跳过 `acq.*` 和 `command.*`
诊断统计，避免绝对纳秒整数溢出和误入归一化。其他版本先验证 API 再使用，开录
前会检查版本。不要在同一进程中并发运行其他未经过该锁的 LeRobot writer。

不连接 ROS/机械臂的 CPU 回归：

```bash
python -m unittest discover -s tests -v
# 使用已安装的lerobot==0.4.4在临时目录写入/追加/回读toy数据，含模拟编码失败：
CUDA_VISIBLE_DEVICES="" RUN_LEROBOT_INTEGRATION=1 \
  python -m unittest discover -s tests -v
```

数据集内容: observation.state(10=末端xyz+6D姿态+夹爪),
observation.joints(7=6关节+夹爪归一化, 备用), action(10=从臂末端绝对位姿,
与state同一条从臂流错开一帧, action[i]≡state[i+1], 官方x2robot2lerobot同语义;
训练侧loader自动转delta; joints模式为7), observation.images.wrist(相机视频),
任务标签来自--task自然语言指令。
训练侧: 把 `data/zbl_data_v4/<任务名>/` 整个目录拷到训练机，先固定完整episode
划分，再只用训练episodes及episode内合法动作窗口计算归一化统计。
