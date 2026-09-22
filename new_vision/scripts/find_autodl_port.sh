#!/usr/bin/env bash
set -euo pipefail

host="${1:-${AUTODL_HOST:-connect.westd.seetacloud.com}}"
user="${2:-${AUTODL_USER:-root}}"
pass="${3:-${AUTODL_SSH_PASS:-}}"
ports_csv="${4:-${AUTODL_PORTS:-26365,22,26030,26041,26070,26076,26088,26114,26157,26165,26167,26168,26173,26190,26217,26220,26239,26250,26252,26291,26309,26325,26345,26351,26353,26358,26368,26370,26399,26425,26445}}"
timeout_s="${AUTODL_TIMEOUT:-12}"

if ! command -v sshpass >/dev/null 2>&1; then
  echo "sshpass 未安装。可执行: sudo apt-get update && sudo apt-get install -y sshpass"
  exit 2
fi

if [[ -z "$pass" ]]; then
  echo "缺少密码。"
  echo "用法: $0 [host] [user] [password] [ports_csv]"
  echo "或先导出环境变量: AUTODL_SSH_PASS"
  exit 2
fi

IFS=',' read -r -a ports <<< "$ports_csv"

denied_count=0
no_count=0

for p in "${ports[@]}"; do
  echo "TRY:$p"
  out="$(timeout "$timeout_s" sshpass -p "$pass" ssh \
    -o StrictHostKeyChecking=no \
    -o UserKnownHostsFile=/dev/null \
    -o ConnectTimeout=6 \
    -o PubkeyAuthentication=no \
    -o PreferredAuthentications=password \
    -p "$p" "$user"@"$host" \
    "echo AUTH_OK; hostname; nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -n 1" 2>&1 || true)"

  if grep -q "AUTH_OK" <<< "$out"; then
    echo "FOUND_PORT:$p"
    echo "$out"
    exit 0
  fi

  if grep -q "Permission denied" <<< "$out"; then
    echo "DENIED:$p"
    denied_count=$((denied_count + 1))
  else
    echo "NO:$p"
    no_count=$((no_count + 1))
  fi

done

echo "NO_VALID_PORT"
echo "SUMMARY: denied=$denied_count no=$no_count"
echo "提示: DENIED 表示端口可达但凭据不匹配；NO 常见于非 SSH 服务或连接超时。"
exit 1
