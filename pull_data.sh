#!/bin/bash
# 数采数据: 容器 -> 宿主机 拉取
# 用法(宿主机执行): bash pull_data.sh [目标目录]
# 拉取后可在容器内先转换(python3 bag2hdf5.py)或拉回后另行处理
set -e
DEST=${1:-$HOME/teleop_data/$(date +%Y%m%d)}
mkdir -p "$DEST"
docker cp zbl_arm_6a_sdk:/home/xr/workspace/recordings/. "$DEST/"
echo "已拉取到: $DEST"
echo "episode数: $(find "$DEST" -name '*.db3' | wc -l)"
du -sh "$DEST"
echo
echo "可选: 在容器内批量转换HDF5:"
echo "  docker exec -it zbl_arm_6a_sdk bash -c 'cd /home/xr/workspace/zbl_arm_6a_teleop && python3 bag2hdf5.py /home/xr/workspace/recordings'"
