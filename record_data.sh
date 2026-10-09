#!/bin/bash
# 遥操作数据采集: 在从臂SDK容器内运行
#   bash record_data.sh [任务名]
# 录制包落盘到容器内 /home/xr/workspace/recordings/<任务名>_<时间>/
# (该路径未挂载到宿主机, 采完用 docker cp 拷出, 见脚本尾部提示)
set -e
source /opt/xr/core/install/setup.bash
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/usr/local/lib

TASK=${1:-demo}
OUT=/home/xr/workspace/recordings/${TASK}_$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"

ros2 bag record -o "$OUT" \
  /joint_states \
  /arm/end_pose \
  /joint_position_controller/commands \
  /gripper_controller/commands \
  /eye_in_hand/eye_in_hand/image_raw/image_compressed

echo "录制完成: $OUT"
echo "宿主机取出: docker cp zbl_arm_6a_sdk:$OUT ~/teleop_data/"
