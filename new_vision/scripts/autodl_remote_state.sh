#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-connect.westd.seetacloud.com}"
PORT="${2:-26365}"
USER_NAME="${3:-root}"
PASS="${4:-${AUTODL_SSH_PASS:-}}"

if [[ -z "$PASS" ]]; then
  echo "缺少密码。用法: $0 [host] [port] [user] [password]"
  exit 2
fi

if ! command -v sshpass >/dev/null 2>&1; then
  echo "sshpass 未安装。可执行: sudo apt-get update && sudo apt-get install -y sshpass"
  exit 2
fi

sshpass -p "$PASS" ssh \
  -o StrictHostKeyChecking=no \
  -o UserKnownHostsFile=/dev/null \
  -o ConnectTimeout=8 \
  -o PubkeyAuthentication=no \
  -o PreferredAuthentications=password \
  -p "$PORT" "$USER_NAME@$HOST" '
    set -e
    echo "HOST:$(hostname)"
    nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader | head -n 1

    if docker version --format "Server={{.Server.Version}}" >/dev/null 2>&1; then
      echo "DOCKER_OK"
      docker version --format "Server={{.Server.Version}}"
    else
      echo "DOCKER_NOT_READY"
      exit 0
    fi

    if [ -f /root/.docker/config.json ] && grep -q nvcr.io /root/.docker/config.json; then
      echo "NVCR_LOGIN_FOUND"
    else
      echo "NVCR_LOGIN_MISSING"
    fi

    imgs=$(docker images --format "{{.Repository}}:{{.Tag}}" | grep -E "^nvcr.io/nvidia/isaac-sim:" | head -n 3 || true)
    if [ -n "$imgs" ]; then
      echo "ISAAC_IMAGE_FOUND"
      echo "$imgs"
    else
      echo "ISAAC_IMAGE_NOT_FOUND"
    fi
  '
