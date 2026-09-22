"""CPU 版 vs gpu_vision 同帧一致性（板子上跑 GPU 路径，桌面跑 CPU 回退）。

- 合成 6 类图卡场景：_binary_selective 逐像素一致、update action 一致
- 巡线合成帧：conf/dev 输出一致
- GPU 路径下允许 adaptiveThreshold ≤0.1% 像素差（CUDA float 化）
"""
import os
import sys

_GPU_DIR = os.path.dirname(os.path.abspath(__file__))
_JETSON_DIR = os.path.dirname(_GPU_DIR)
for _p in (_GPU_DIR, _JETSON_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
import cv2

import backend
from shape_detector import ShapeDetector as CPU_SD
from shape_detector_gpu import ShapeDetectorGPU
from line_detector_v1_warp import LineDetector as CPU_LD
from line_detector_gpu import LineDetector as GPU_LD

_DATA = os.path.join(_JETSON_DIR, "..", "6_pictures")
if _DATA not in sys.path:
    sys.path.insert(0, _DATA)

print(f"HAS_CUDA: {backend.HAS_CUDA}  "
      f"({'GPU 路径' if backend.HAS_CUDA else 'CPU 回退路径（桌面）'})")

sd1 = CPU_SD(stable_frames=1, cooldown_ms=0)
sd2 = ShapeDetectorGPU(stable_frames=1, cooldown_ms=0)

from generate_synthetic_cards import render_card, compose_scene  # noqa: E402

ok_bin = ok_action = n = 0
diff_report = []
for s_idx in range(6):
    for trial in range(3):
        card, quad = render_card(s_idx, size=256,
                                 offset_frac=0.04 if trial == 1 else None)
        scene, _, _ = compose_scene(card, quad, 960, 540, "homography",
                                    conservative=(trial == 1))
        g1 = cv2.cvtColor(scene, cv2.COLOR_BGR2GRAY)
        b1 = sd1._binary_selective(g1)
        b2 = sd2._binary_selective(g1)
        same = np.array_equal(b1, b2)
        ok_bin += same
        if not same:
            diff = np.count_nonzero(b1 != b2) / b1.size * 100
            diff_report.append(f"  binary diff {s_idx}/{trial}: {diff:.3f}%")
        a1, _ = sd1.update(scene)
        a2, _ = sd2.update(scene)
        ok_action += a1 == a2
        if a1 != a2:
            diff_report.append(f"  action diff {s_idx}/{trial}: cpu={a1} gpu={a2}")
        n += 1

print(f"找框: binary 一致 {ok_bin}/{n} | action 一致 {ok_action}/{n}")
for line in diff_report:
    print(line)

# 巡线
big = np.full((720, 1280, 3), 255, np.uint8)
cv2.line(big, (400, 100), (880, 620), (0, 0, 0), 30)
cv2.line(big, (900, 120), (1240, 560), (0, 0, 0), 30)
r1 = CPU_LD().process(big)
r2 = GPU_LD().process(big)
print(f"巡线: conf cpu={r1[2]:.3f} gpu={r2[2]:.3f} | dev cpu={r1[0]:.2f} gpu={r2[0]:.2f}")
print("PASS" if ok_bin == n and ok_action == n else "CHECK DIFFS")
