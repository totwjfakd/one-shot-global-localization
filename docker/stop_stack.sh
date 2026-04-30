#!/usr/bin/env bash

set -euo pipefail

if ! command -v docker >/dev/null 2>&1; then
  echo "docker 명령을 찾을 수 없습니다."
  exit 1
fi

docker rm -f relocalization_toolbox cbgl ros1_bridge >/dev/null 2>&1 || true
echo "stack 종료 완료"
