#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-connect.westd.seetacloud.com}"
PORT="${2:-26365}"
USER_NAME="${3:-root}"
PASS="${4:-${AUTODL_SSH_PASS:-}}"
TAG="${5:-4.5.0}"

if [[ -z "$PASS" ]]; then
  echo "缺少密码。用法: $0 [host] [port] [user] [password] [tag]"
  exit 2
fi

sshpass -p "$PASS" ssh \
  -o StrictHostKeyChecking=no \
  -o UserKnownHostsFile=/dev/null \
  -o ConnectTimeout=10 \
  -o PubkeyAuthentication=no \
  -o PreferredAuthentications=password \
  -p "$PORT" "$USER_NAME@$HOST" '
    set -e
    docker pull "nvcr.io/nvidia/isaac-sim:'"$TAG"'"
    docker images --format "{{.Repository}}:{{.Tag}}" | grep -E "^nvcr.io/nvidia/isaac-sim:" | head -n 3
  '
