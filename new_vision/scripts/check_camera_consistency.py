#!/usr/bin/env python3
"""Check camera resolution/FOV consistency between params and Webots world."""

from __future__ import annotations

import json
import math
import os
import re
import sys
from typing import Optional, Tuple


def load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def parse_world_camera(world_text: str, camera_name: str = "camera_ext") -> Optional[Tuple[int, int, float]]:
    block_re = re.compile(r"Camera\s*\{[^{}]*name\s+\"%s\"[^{}]*\}" % re.escape(camera_name), re.S)
    m = block_re.search(world_text)
    if not m:
        return None

    block = m.group(0)

    w_m = re.search(r"\bwidth\s+(\d+)", block)
    h_m = re.search(r"\bheight\s+(\d+)", block)
    fov_m = re.search(r"\bfieldOfView\s+([0-9eE+\-.]+)", block)
    if not (w_m and h_m and fov_m):
        return None

    return int(w_m.group(1)), int(h_m.group(1)), float(fov_m.group(1))


def ratio(w: int, h: int) -> float:
    return float(w) / float(max(h, 1))


def infer_vfov_deg(hfov_deg: float, w: int, h: int) -> float:
    hfov = math.radians(hfov_deg)
    vfov = 2.0 * math.atan(math.tan(hfov * 0.5) * (float(h) / float(w)))
    return math.degrees(vfov)


def main() -> int:
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    params_path = os.path.join(repo, "line_follow_params.json")
    world_path = os.path.join(repo, "Webots", "worlds", "Robocup.wbt")

    if not os.path.isfile(params_path):
        print("[ERROR] Missing params:", params_path)
        return 2
    if not os.path.isfile(world_path):
        print("[ERROR] Missing world:", world_path)
        return 3

    cfg = load_json(params_path)
    with open(world_path, "r", encoding="utf-8") as f:
        world_text = f.read()

    cal_img = cfg.get("camera", {}).get("calibration", {}).get("image_size", {})
    base_frame = cfg.get("base_frame", {})
    web_cam = cfg.get("webots", {}).get("camera", {})
    hfov_deg = float(cfg.get("camera", {}).get("hfov_deg", 0.0))
    vfov_deg = float(cfg.get("camera", {}).get("vfov_deg", 0.0))

    cal_w = int(cal_img.get("width", 0))
    cal_h = int(cal_img.get("height", 0))
    base_w = int(base_frame.get("width", 0))
    base_h = int(base_frame.get("height", 0))
    web_w = int(web_cam.get("width", 0))
    web_h = int(web_cam.get("height", 0))

    world_cam = parse_world_camera(world_text, str(web_cam.get("device_name", "camera_ext")))

    print("[INFO] calibration image:", cal_w, "x", cal_h)
    print("[INFO] base_frame (algorithm reference):", base_w, "x", base_h)
    print("[INFO] params webots camera:", web_w, "x", web_h)
    if world_cam is None:
        print("[WARN] world camera block not found")
    else:
        ww, wh, wfov = world_cam
        print("[INFO] world camera:", ww, "x", wh, "fov(rad)=", round(wfov, 6))

    ok = True

    if cal_w > 0 and cal_h > 0 and web_w > 0 and web_h > 0:
        dr = abs(ratio(cal_w, cal_h) - ratio(web_w, web_h))
        if dr > 1e-3:
            ok = False
            print("[FAIL] aspect ratio mismatch between calibration and params webots camera")
        else:
            print("[OK] calibration and params webots camera share the same aspect ratio")

    if base_w > 0 and base_h > 0 and web_w > 0 and web_h > 0:
        dr_base = abs(ratio(base_w, base_h) - ratio(web_w, web_h))
        if dr_base > 1e-3:
            ok = False
            print("[FAIL] base_frame aspect ratio differs from webots camera")
        else:
            print("[OK] base_frame and webots camera share the same aspect ratio")
        if base_w != web_w or base_h != web_h:
            print("[INFO] base_frame size differs from camera size: this is allowed if it is only a processing reference frame")

    if world_cam is not None:
        ww, wh, wfov = world_cam
        if ww != web_w or wh != web_h:
            ok = False
            print("[FAIL] world camera size != params webots camera size")
        else:
            print("[OK] world camera size matches params")

        params_fov = float(web_cam.get("field_of_view", 0.0))
        if abs(params_fov - wfov) > 1e-6:
            ok = False
            print("[FAIL] world camera FOV != params webots.camera.field_of_view")
        else:
            print("[OK] world camera FOV matches params")

    if hfov_deg > 0 and web_w > 0 and web_h > 0:
        vfov_from_h = infer_vfov_deg(hfov_deg, web_w, web_h)
        print("[INFO] vfov from hfov+ratio:", round(vfov_from_h, 6), "deg")
        if vfov_deg > 0:
            dv = abs(vfov_from_h - vfov_deg)
            if dv > 0.5:
                ok = False
                print("[FAIL] camera.vfov_deg seems inconsistent with camera.hfov_deg and image ratio")
            else:
                print("[OK] camera.vfov_deg is consistent with camera.hfov_deg")

    if ok:
        print("\nRESULT: PASS")
        return 0

    print("\nRESULT: FAIL")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
