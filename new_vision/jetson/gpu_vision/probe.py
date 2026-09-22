"""板子环境探测 — 第一步在 Jetson 上跑。

输出：CUDA 设备、cv2.cuda 可用性、OpenCV 编译 CUDA 段、每算子计时。
"""
import os
import sys
import time

_GPU_DIR = os.path.dirname(os.path.abspath(__file__))
_JETSON_DIR = os.path.dirname(_GPU_DIR)
for _p in (_GPU_DIR, _JETSON_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import cv2

import backend


def _print_cuda_build_info():
    info = cv2.getBuildInformation()
    for key in ("CUDA", "cuDNN", "NVIDIA GPU arch"):
        for line in info.splitlines():
            if line.startswith("  " + key) or line.startswith(key):
                print("   ", line.strip())


def main():
    print("== CUDA 设备 ==")
    print("  cv2.cuda 设备数:", cv2.cuda.getCudaEnabledDeviceCount())
    print("== OpenCV 编译选项 ==")
    _print_cuda_build_info()
    print("== backend ==")
    print("  HAS_CUDA:", backend.HAS_CUDA)

    print("== 算子计时（CPU/GPU 当前生效路径）==")
    gray = (np.random.rand(540, 960) * 255).astype(np.uint8)
    _t("shape_binary_chain 960×540", lambda: backend.shape_binary_chain(gray))
    big = (np.random.rand(720, 1280, 3) * 255).astype(np.uint8)
    _t("birdseye_warp 1280×720→320×400",
       lambda: backend.birdseye_warp(big, np.eye(3, dtype=np.float64), (320, 400)))
    small = (np.random.rand(400, 320) * 255).astype(np.uint8)
    _t("line_binary_chain 320×400",
       lambda: backend.line_binary_chain(small, th_offset=-2))


def _t(name, fn, n=20):
    fn()  # warmup
    t0 = time.time()
    for _ in range(n):
        fn()
    ms = (time.time() - t0) / n * 1000
    print(f"  {name}: {ms:.2f} ms/次")


if __name__ == "__main__":
    main()
