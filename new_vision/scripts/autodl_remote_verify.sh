#!/usr/bin/env bash
set -euo pipefail

echo "---GPU---"
nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -n 1

echo "---NCTK---"
nvidia-ctk --version
dpkg -l | grep nvidia-container-toolkit | sed -n '1,3p'

echo "---DOCKER---"
docker version --format 'Server={{.Server.Version}}'

echo "---ISAAC_IMAGES---"
docker images --format '{{.Repository}}:{{.Tag}}' | grep -E 'isaac-sim|nvcr.io/nvidia/isaac' | head -n 10 || echo "ISAAC_IMAGE_NOT_FOUND"

echo "---RUNTIMES---"
docker info 2>/dev/null | grep -E 'Runtimes|Default Runtime|nvidia' -n | sed -n '1,20p' || true

echo "---NVCR---"
curl -I -m 10 https://nvcr.io/v2/ | sed -n '1,6p' || true

echo "---DOCKERHUB---"
curl -I -m 10 https://registry-1.docker.io/v2/ | sed -n '1,6p' || true
