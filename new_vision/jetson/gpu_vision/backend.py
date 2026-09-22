"""GPU 算子后端 — cv2.cuda 可用时整链走 GPU，否则纯 CPU 回退。

回退实现与 jetson/ CPU 版同源（逐算子等价），保证本地无 CUDA 环境
跑 gpu_vision = 验证正确逻辑，板子上仅算子在换。

原则：链级函数内部一次上传、多次算子、一次下载，调用方只见 numpy。
"""
import cv2
import numpy as np

HAS_CUDA = False
try:
    HAS_CUDA = cv2.cuda.getCudaEnabledDeviceCount() > 0
except Exception:
    HAS_CUDA = False

_kernel_cache = {}


def kernel(shape, size):
    """结构化核缓存（CPU 版每帧重建的浪费在此消除）。"""
    key = (shape, size)
    if key not in _kernel_cache:
        _kernel_cache[key] = cv2.getStructuringElement(shape, size)
    return _kernel_cache[key]


def _morph(gpu, op, shape, size, iters=1):
    """GpuMat 形态学滤波（核缓存复用；filter 对象每次重建）。"""
    k_gpu = cv2.cuda_GpuMat()
    k_gpu.upload(kernel(shape, size))
    flt = cv2.cuda.createMorphologyFilter(op, cv2.CV_8UC1, k_gpu)
    for _ in range(iters):
        dst = cv2.cuda_GpuMat()
        flt.apply(gpu, dst)
        gpu = dst
    return gpu


def _upload(img):
    g = cv2.cuda_GpuMat()
    g.upload(img)
    return g


# ═══════════════════════════════════════════════════════════════════════
# 图卡找框链（等价 shape_detector._binary_selective 前半段）
#   960×540 灰度 → 线白二值（blackhat → adaptive → close3 → aniso close）
# ═══════════════════════════════════════════════════════════════════════

def shape_binary_chain(gray, bh_kernel=9, block=31, c=-12):
    """返回 numpy 二值图（喂笔画宽/细长度 CPU 过滤）。"""
    if not HAS_CUDA:
        kbh = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                        (bh_kernel, bh_kernel))
        b = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kbh)
        b = cv2.adaptiveThreshold(b, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv2.THRESH_BINARY, block, c)
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        b = cv2.morphologyEx(b, cv2.MORPH_CLOSE, k3)
        kv = cv2.getStructuringElement(cv2.MORPH_RECT, (1, 5))
        kh = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 1))
        b = cv2.morphologyEx(b, cv2.MORPH_CLOSE, kv)
        b = cv2.morphologyEx(b, cv2.MORPH_CLOSE, kh)
        return b

    gpu = _morph(_upload(gray), cv2.MORPH_BLACKHAT,
                 cv2.MORPH_ELLIPSE, (bh_kernel, bh_kernel))
    out = cv2.cuda_GpuMat()
    cv2.cuda.adaptiveThreshold(gpu, out, 255,
                               cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY, block, c)
    gpu = _morph(out, cv2.MORPH_CLOSE, cv2.MORPH_ELLIPSE, (3, 3))
    gpu = _morph(gpu, cv2.MORPH_CLOSE, cv2.MORPH_RECT, (1, 5))
    gpu = _morph(gpu, cv2.MORPH_CLOSE, cv2.MORPH_RECT, (5, 1))
    return gpu.download()


# ═══════════════════════════════════════════════════════════════════════
# 透视矫正（找框候选 warp / 巡线 IPM）
# ═══════════════════════════════════════════════════════════════════════

def warp_perspective(src, mtx, dsize, flags=cv2.INTER_LINEAR):
    """单应变换。候选 warp 输出小，每调用往返开销可忽略。"""
    if not HAS_CUDA:
        return cv2.warpPerspective(src, mtx, dsize, flags=flags)
    out = cv2.cuda_GpuMat()
    cv2.cuda.warpPerspective(_upload(src), out, mtx, dsize, flags=flags)
    return out.download()


def birdseye_warp(bgr, mtx, dsize):
    """原图 → IPM 鸟瞰（大图 GPU 收益最大的一步）。"""
    if not HAS_CUDA:
        return cv2.warpPerspective(bgr, mtx, dsize)
    out = cv2.cuda_GpuMat()
    cv2.cuda.warpPerspective(_upload(bgr), out, mtx, dsize)
    return out.download()


# ═══════════════════════════════════════════════════════════════════════
# 巡线链（等价 line_detector.process L926-959）
#   鸟瞰灰度 → blackhat → adaptive → threshold → close5/open5/close5/open3
#   black_th 自适应修正（adaptive mask 中位数）与 Otsu 回退保留在链内
# ═══════════════════════════════════════════════════════════════════════

def line_binary_chain(gray, th_offset=-2, bh_size=31, block=31, c=-8,
                      th_min=25, th_max=80, otsu_fn=None):
    """返回 (gray_detect, binary_clean, black_th)。

    otsu_fn: 当 adaptive mask 像素过少时替代 black_th 的回退函数
    （CPU Python 直方图版，传 LineDetector._otsu_threshold）。
    """
    if not HAS_CUDA:
        kbh = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                        (bh_size, bh_size))
        gd = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kbh)
        adaptive = cv2.adaptiveThreshold(
            gd, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY, block, c)
        black_th = _resolve_black_th(gd, adaptive, th_offset, th_min,
                                     th_max, otsu_fn)
        _, binary = cv2.threshold(gd, black_th, 255, cv2.THRESH_BINARY)
        k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k5)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, k5)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, k5)
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, k3)
        return gd, binary, black_th

    # GPU: blackhat → adaptive（adaptive mask 中位数需 CPU → 暂停一次）
    gpu = _morph(_upload(gray), cv2.MORPH_BLACKHAT,
                 cv2.MORPH_ELLIPSE, (bh_size, bh_size))
    gd = gpu.download()
    out = cv2.cuda_GpuMat()
    cv2.cuda.adaptiveThreshold(gpu, out, 255,
                               cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY, block, c)
    black_th = _resolve_black_th(gd, out.download(), th_offset, th_min,
                                 th_max, otsu_fn)

    # GPU: 二次阈值 + 形态学链
    out2 = cv2.cuda_GpuMat()
    cv2.cuda.threshold(_upload(gd), out2, black_th, 255, cv2.THRESH_BINARY)
    gpu2 = _morph(out2, cv2.MORPH_CLOSE, cv2.MORPH_ELLIPSE, (5, 5))
    gpu2 = _morph(gpu2, cv2.MORPH_OPEN, cv2.MORPH_ELLIPSE, (5, 5))
    gpu2 = _morph(gpu2, cv2.MORPH_CLOSE, cv2.MORPH_ELLIPSE, (5, 5))
    gpu2 = _morph(gpu2, cv2.MORPH_OPEN, cv2.MORPH_ELLIPSE, (3, 3))
    return gd, gpu2.download(), black_th


def _resolve_black_th(gray_detect, adaptive_binary, th_offset, th_min,
                      th_max, otsu_fn):
    """与 line_detector L943-950 等价：mask 中位数 + offset，Otsu 回退。"""
    mask = adaptive_binary == 255
    if np.count_nonzero(mask) > 100:
        bt = np.median(gray_detect[mask]) + th_offset
    elif otsu_fn is not None:
        bt = otsu_fn(gray_detect) + th_offset
    else:
        bt = 64.0
    return float(np.clip(bt, th_min, th_max))
