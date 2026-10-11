#!/bin/bash
# 数采一键转换: 容器内bag->h5 -> 拉取宿主机 -> LeRobot数据集(增量追加)
# 录完episode后, 在宿主机执行:  bash make_dataset.sh [数据集名]
#   默认数据集名 zbl_data, 输出 ~/lerobot_data/<数据集名>/
#   已转换过的episode自动跳过, 可反复执行
set -e
DIR="$(cd "$(dirname "$0")" && pwd)"
REPO=${1:-zbl_data}
REC=/home/xr/workspace/recordings
DEST=$HOME/teleop_data

echo "== [1/3] 容器内 bag -> episode.h5 (增量) =="
docker exec zbl_arm_6a_sdk bash -c \
  "source /opt/xr/core/install/setup.bash && cd /home/xr/workspace/zbl_arm_6a_teleop && python3 bag2hdf5.py $REC"

echo "== [2/3] 拉取到宿主机 $DEST =="
mkdir -p "$DEST"
docker cp "zbl_arm_6a_sdk:$REC/." "$DEST/"

echo "== [3/3] 转 LeRobot v3 (zbl_lerobot环境, 增量追加到 $REPO) =="
source ~/miniconda3/etc/profile.d/conda.sh
conda activate zbl_lerobot
cd "$DIR"
python h5_to_lerobot.py "$DEST" --repo-id "$REPO"
echo "== 完成: ~/lerobot_data/$REPO (直接拷给wall-x训练) =="
