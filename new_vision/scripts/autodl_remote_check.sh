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
  -p "$PORT" "$USER_NAME"@"$HOST" '
    set -e
    echo "HOST:$(hostname)"
    nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -n 1

    if docker version --format "Server={{.Server.Version}}" >/tmp/docker_v.txt 2>/dev/null; then
      cat /tmp/docker_v.txt
      docker images --format "{{.Repository}}:{{.Tag}}" | grep -E "isaac-sim|nvcr.io/nvidia/isaac" | head -n 5 || echo ISAAC_IMAGE_NOT_FOUND
    else
      echo DOCKER_NOT_READY
      echo ISAAC_IMAGE_UNKNOWN
    fi
  '
