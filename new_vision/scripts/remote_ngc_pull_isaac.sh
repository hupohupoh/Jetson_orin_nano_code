#!/usr/bin/env bash
set -euo pipefail

HOST="${1:-connect.westd.seetacloud.com}"
PORT="${2:-26365}"
USER_NAME="${3:-root}"
PASS="${4:-}"
KEY_PART1="${5:-}"
KEY_PART2="${6:-}"
TAG="${7:-4.5.0}"

if [[ -z "$PASS" || -z "$KEY_PART1" || -z "$KEY_PART2" ]]; then
  echo "用法: $0 <host> <port> <user> <pass> <key_part1> <key_part2> [tag]"
  exit 2
fi

if ! command -v sshpass >/dev/null 2>&1; then
  echo "sshpass 未安装。可执行: sudo apt-get update && sudo apt-get install -y sshpass"
  exit 2
fi

tmp_script="$(mktemp)"
cat >"$tmp_script" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail
key_part1="$1"
key_part2="$2"
tag="$3"
export NGC_API_KEY="${key_part1}${key_part2}"

echo "$NGC_API_KEY" | docker login nvcr.io -u '$oauthtoken' --password-stdin
docker pull "nvcr.io/nvidia/isaac-sim:${tag}"
docker images --format '{{.Repository}}:{{.Tag}}' | grep -E '^nvcr.io/nvidia/isaac-sim:' | head -n 3
EOF
chmod +x "$tmp_script"

ssh_opts=(
  -o StrictHostKeyChecking=no
  -o UserKnownHostsFile=/dev/null
  -o ConnectTimeout=10
  -o PubkeyAuthentication=no
  -o PreferredAuthentications=password
)

sshpass -p "$PASS" scp -P "$PORT" "${ssh_opts[@]}" "$tmp_script" "$USER_NAME@$HOST:/root/do_ngc_pull.sh"
sshpass -p "$PASS" ssh -p "$PORT" "${ssh_opts[@]}" "$USER_NAME@$HOST" \
  "bash /root/do_ngc_pull.sh '$KEY_PART1' '$KEY_PART2' '$TAG'"

rm -f "$tmp_script"
