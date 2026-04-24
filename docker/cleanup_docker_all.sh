#!/usr/bin/env bash

set -euo pipefail

if ! command -v docker >/dev/null 2>&1; then
  echo "docker 명령을 찾을 수 없습니다. Docker가 설치되어 있는지 확인하세요."
  exit 1
fi

echo "[1/2] 모든 컨테이너 삭제 중..."
containers="$(docker ps -aq)"
if [ -n "$containers" ]; then
  docker rm -f $containers
  echo "컨테이너 삭제 완료"
else
  echo "삭제할 컨테이너가 없습니다."
fi

echo "[2/2] 모든 이미지 삭제 중..."
images="$(docker images -aq)"
if [ -n "$images" ]; then
  docker rmi -f $images
  echo "이미지 삭제 완료"
else
  echo "삭제할 이미지가 없습니다."
fi

echo "작업 완료"
