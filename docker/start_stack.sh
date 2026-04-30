#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"

build_images=0
if [[ "${1:-}" == "--build" ]]; then
  build_images=1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "docker 명령을 찾을 수 없습니다."
  exit 1
fi

if [[ "${build_images}" == "1" ]]; then
  echo "[1/5] 이미지 빌드 중..."
  docker build -f "${repo_root}/docker/Dockerfile.bridge" -t ros1-bridge:local "${repo_root}"
  docker build -f "${repo_root}/docker/Dockerfile" -t li9i/cbgl:latest "${repo_root}"
  docker build -f "${repo_root}/docker/Dockerfile.relocalization" -t relocalization-toolbox:local "${repo_root}"
else
  echo "[1/5] 이미지 빌드 생략 (--build를 주면 재빌드합니다)"
fi

echo "[2/5] 기존 stack 컨테이너 정리 중..."
docker rm -f relocalization_toolbox cbgl ros1_bridge >/dev/null 2>&1 || true

echo "[3/5] ros1_bridge 시작 중 (ROS 1 master owner)..."
docker run -d \
  --name ros1_bridge \
  --network host \
  --ipc host \
  -e ROS_MASTER_URI=http://127.0.0.1:11311 \
  -e ROS_HOSTNAME=127.0.0.1 \
  -e ROS_IP=127.0.0.1 \
  -e ROS_DOMAIN_ID=42 \
  -e ROS_LOCALHOST_ONLY=0 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  ros1-bridge:local >/dev/null

for _ in $(seq 1 30); do
  if docker exec ros1_bridge bash -lc 'source /opt/ros/noetic/setup.bash; rostopic list >/dev/null 2>&1'; then
    break
  fi
  sleep 1
done

echo "[4/5] CBGL 시작 중..."
docker run -d \
  --name cbgl \
  --network host \
  --ipc host \
  --user cbgl \
  -e ROS_MASTER_URI=http://127.0.0.1:11311 \
  -e ROS_HOSTNAME=127.0.0.1 \
  -e ROS_IP=127.0.0.1 \
  -e CBGL_USE_SIM_TIME=true \
  -v "${repo_root}/cbgl:/home/cbgl/catkin_ws/src/cbgl:rw" \
  li9i/cbgl:latest >/dev/null

echo "[5/5] relocalization_toolbox 시작 중..."
docker run -d \
  --name relocalization_toolbox \
  --network host \
  --ipc host \
  --user relocalization \
  -e ROS_MASTER_URI=http://127.0.0.1:11311 \
  -e ROS_HOSTNAME=127.0.0.1 \
  -e ROS_IP=127.0.0.1 \
  -v "${repo_root}/relocalization_toolbox/relocalization_toolbox/relocalization_toolbox:/home/relocalization/catkin_ws/src/relocalization_toolbox:rw" \
  -v "${repo_root}/relocalization_toolbox/relocalization_toolbox/relocalization_toolbox_msgs:/home/relocalization/catkin_ws/src/relocalization_toolbox_msgs:rw" \
  relocalization-toolbox:local >/dev/null

echo "stack 시작 완료"
docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}' \
  --filter name='^(ros1_bridge|cbgl|relocalization_toolbox)$'
