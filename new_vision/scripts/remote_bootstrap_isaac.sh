#!/usr/bin/env bash
set -euo pipefail

# 在云端实例上执行本脚本，用于自动安装 Docker + NVIDIA 容器工具并做 Isaac 冒烟测试。
# 用法:
#   chmod +x remote_bootstrap_isaac.sh
#   export NGC_API_KEY="你的key"
#   ./remote_bootstrap_isaac.sh
# 可选环境变量:
#   ISAAC_TAG=4.5.0
#   SMOKE_SECONDS=120
#   SKIP_PULL=0
#   SKIP_LOGIN=0
#   SKIP_SMOKE=0

ISAAC_TAG="${ISAAC_TAG:-4.5.0}"
SMOKE_SECONDS="${SMOKE_SECONDS:-120}"
SKIP_PULL="${SKIP_PULL:-0}"
SKIP_LOGIN="${SKIP_LOGIN:-0}"
SKIP_SMOKE="${SKIP_SMOKE:-0}"

log() {
  echo "[$(date +'%F %T')] $*"
}

log "Step 1/8: 基础信息"
uname -a || true
cat /etc/os-release || true
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader

log "Step 2/8: 安装 Docker 基础包"
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y docker.io curl gnupg2 ca-certificates

log "Step 3/8: 安装 nvidia-container-toolkit"
distribution=$(. /etc/os-release; echo ${ID}${VERSION_ID})
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor --batch --yes -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/${distribution}/libnvidia-container.list |
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' |
  tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null
apt-get update
apt-get install -y nvidia-container-toolkit

log "Step 4/8: 配置并启动 dockerd"
nvidia-ctk runtime configure --runtime=docker || true
pkill -9 dockerd || true
rm -f /var/run/docker.pid /var/run/docker.sock
nohup dockerd --iptables=false --bridge=none --storage-driver=vfs >/tmp/dockerd.log 2>&1 &
sleep 8
docker version --format 'Server={{.Server.Version}}'

log "Step 5/8: 验证 GPU 容器"
if docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi | sed -n '1,10p'; then
  log "DockerHub GPU 测试通过"
else
  log "DockerHub 不可达或测试失败，继续执行 NGC 登录与 Isaac 拉取"
fi

if [[ "$SKIP_LOGIN" != "1" ]]; then
  if [[ -z "${NGC_API_KEY:-}" ]]; then
    log "未设置 NGC_API_KEY，无法登录 nvcr.io"
    exit 2
  fi
  log "Step 6/8: 登录 nvcr.io"
  echo "$NGC_API_KEY" | docker login nvcr.io -u '$oauthtoken' --password-stdin
fi

if [[ "$SKIP_PULL" != "1" ]]; then
  log "Step 7/8: 拉取 Isaac 镜像"
  docker pull "nvcr.io/nvidia/isaac-sim:${ISAAC_TAG}"
fi

if [[ "$SKIP_SMOKE" == "1" ]]; then
  log "Step 8/8: 跳过冒烟测试（SKIP_SMOKE=1）"
else
  log "Step 8/8: 运行 ${SMOKE_SECONDS}s headless 冒烟测试"
  mkdir -p /root/isaac/cache/kit /root/isaac/cache/ov /root/isaac/cache/pip /root/isaac/data /root/isaac/logs
  docker rm -f isaac-test >/dev/null 2>&1 || true
  nohup docker run --name isaac-test --gpus all --network host \
    -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ACCEPT_EULA=YES \
    -v /root/isaac/cache/kit:/isaac-sim/kit/cache:rw \
    -v /root/isaac/cache/ov:/root/.cache/ov:rw \
    -v /root/isaac/cache/pip:/root/.cache/pip:rw \
    -v /root/isaac/data:/isaac-sim/data:rw \
    "nvcr.io/nvidia/isaac-sim:${ISAAC_TAG}" \
    /bin/bash -lc "./runheadless.sh --/app/quitAfter=${SMOKE_SECONDS}" \
    >/root/isaac/logs/isaac_test.log 2>&1 &

  sleep 20
  docker ps --format 'table {{.Names}}\t{{.Status}}\t{{.Image}}' | sed -n '1,6p'
  sleep "$SMOKE_SECONDS"
  docker ps -a --format '{{.Names}} {{.Status}}' | grep isaac-test || true
  echo '--- ISAAC LOG TAIL ---'
  tail -n 120 /root/isaac/logs/isaac_test.log || true
fi

log "Done"
