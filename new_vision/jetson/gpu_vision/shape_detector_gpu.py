"""ShapeDetector GPU 版 — 继承 CPU 版，预处理链与候选 warp 走 backend。

GPU 化范围（按收益）：
- _binary_selective 前半链（blackhat+adaptive+close+aniso，960×540 最大收益段）
- _warp_card 的 warpPerspective（几何构建保留原样）
笔画宽/细长度过滤、候选生成、几何验证、形状分类
留 CPU：几何算法无 CUDA 对应或收益小。

cv2.cuda 不可用时 backend 自动 CPU 回退（逐算子与原 CPU 版等价）。
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

from backend import HAS_CUDA, shape_binary_chain, warp_perspective
from shape_detector import ShapeDetector as _CPU


class ShapeDetectorGPU(_CPU):
    """同参数同接口：ShapeDetector(stable_frames=3, cooldown_ms=3200)。"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if not HAS_CUDA:
            print("[shape_gpu] cv2.cuda 不可用 → CPU 回退 "
                  "（输出与原 CPU 版一致）")

    def _binary_selective(self, gray):
        b = shape_binary_chain(gray, self.cfg["bh_kernel"],
                               self.cfg["adaptive_block"],
                               self.cfg["adaptive_c"])
        b = self._stroke_width_filter(b)
        return self._elongation_filter(b)

    def _warp_card(self, binary, quad):
        """同 CPU 版：角排序 + 外扩/内缩，warpPerspective 换 backend。"""
        ws = self.cfg["warp_size"]
        q = quad.astype(np.float32)
        c = q.mean(axis=0)
        v = q - c
        norms = np.linalg.norm(v, axis=1, keepdims=True) + 1e-9
        q = c + v / norms * (norms + 4.0)
        q = c + (q - c) * (1.0 - self.cfg["warp_inset"])
        cx, cy = q.mean(axis=0)
        ang = np.arctan2(q[:, 1] - cy, q[:, 0] - cx)
        order = np.argsort(ang)
        qs = q[order]
        start = int(np.argmin(qs.sum(axis=1)))
        qs = np.roll(qs, -start, axis=0)
        dst = np.float32([[0, 0], [ws - 1, 0], [ws - 1, ws - 1], [0, ws - 1]])
        M = cv2.getPerspectiveTransform(qs, dst)
        return warp_perspective(binary, M, (ws, ws),
                                flags=cv2.INTER_NEAREST)
