"""相机参数加载 — config/cameras.json（多摄像头配置）。

用法:
    from camera_config import load as load_camera
    cam = load_camera()                 # 取 active 配置
    cam = load_camera("laptop_test")    # 指定配置
    # 或用环境变量 CAMERA_PROFILE 切换

配置缺失时回退默认值（USB 实车相机参数）。
"""
import json
import os

_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     "config", "cameras.json")

_DEFAULTS = {
    "index": 0,
    "width": 1280,
    "height": 720,
    "vfov_deg": 56.2,
    "mount_height_cm": 40.0,
    "pitch_deg": 45.0,
}


def load(profile=None):
    """返回当前摄像头参数字典（含 profile 名）。"""
    cam = dict(_DEFAULTS)
    cam["profile"] = "default"
    try:
        with open(_PATH, encoding="utf-8") as f:
            cfg = json.load(f)
        key = profile or os.environ.get("CAMERA_PROFILE") or cfg.get("active")
        entry = dict(cfg["cameras"][key])
        cam.update({k: v for k, v in entry.items() if v is not None})
        cam["profile"] = key
    except Exception:
        pass
    return cam
