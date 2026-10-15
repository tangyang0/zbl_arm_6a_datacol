#!/bin/bash
# 从厂家原版镜像重建数采镜像(与v0.1.0等价): 装依赖 -> 清缓存 -> export/import压平
# 前提: 本机已有厂家镜像 zbl_arm_6a_sdk_image:v0.1.1
# 用法: bash build_image.sh [输出tag, 默认zbl_arm_6a_datacol_image:v0.1.0]
set -e
TAG="${1:-zbl_arm_6a_datacol_image:v0.1.0}"
BASE="zbl_arm_6a_sdk_image:v0.1.1"
TMP="datacol_img_build"

[ -n "$(docker images -q $BASE)" ] || { echo "缺厂家基础镜像 $BASE"; exit 1; }

echo "== 1/4 起临时容器装数采依赖 =="
docker rm -f $TMP 2>/dev/null || true
docker run -d --name $TMP $BASE sleep 1800
docker exec -u root $TMP bash -c '
pip3 install websocket-client websockets \
  torch==2.10.0+cpu torchvision==0.25.0+cpu \
  -i https://download.pytorch.org/whl/cpu &&
pip3 install lerobot==0.4.4'

echo "== 2/4 配置git跨属主访问例外 =="
docker exec -u xr $TMP bash -c \
  'git config --global --add safe.directory /home/xr/workspace/zbl_arm_6a_datacol'

echo "== 3/4 清缓存并压平为单层(export/import) =="
docker exec -u root $TMP bash -c \
  'rm -rf /root/.cache /home/xr/.cache /var/lib/apt/lists/* /tmp/*'
docker export $TMP | docker import \
  --change 'USER xr' \
  --change 'WORKDIR /home/xr' \
  --change 'ENV ROS2_DISTRO=jazzy' \
  --change 'ENV DISPLAY=:1' \
  --change 'ENV QT_X11_NO_MITSHM=1' \
  --change 'ENV DEBIAN_FRONTEND=noninteractive' \
  --change 'CMD ["bash","-i","-c","echo Development environment ready! && while sleep 1000; do :; done"]' \
  - "$TAG"
docker rm -f $TMP

echo "== 4/4 完成: $TAG =="
docker images "$TAG"
