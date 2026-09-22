#!/usr/bin/env python3
"""Calibrate camera intrinsics and distortion from chessboard images using OpenCV.

Example:
  python scripts/calibrate_camera_opencv.py \
    --images "calib_photos/*.jpg" \
    --cols 9 --rows 6 --square-size-mm 20 \
    --output generated/camera_calibration_result.json \
    --debug-dir generated/calib_debug
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
from dataclasses import dataclass
from datetime import datetime
from typing import List, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class DetectionResult:
    image_path: str
    corners: np.ndarray
    image_size: Tuple[int, int]  # (width, height)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="OpenCV chessboard camera calibration")
    parser.add_argument(
        "--images",
        required=True,
        help="Image glob pattern(s), comma separated. Example: calib_photos/*.jpg,calib_photos/*.png",
    )
    parser.add_argument("--cols", type=int, required=True, help="Chessboard inner corners per row (width)")
    parser.add_argument("--rows", type=int, required=True, help="Chessboard inner corners per column (height)")
    parser.add_argument("--square-size-mm", type=float, default=20.0, help="Chessboard square size in mm")
    parser.add_argument(
        "--output",
        default="generated/camera_calibration_result.json",
        help="Output JSON path",
    )
    parser.add_argument(
        "--debug-dir",
        default="",
        help="Optional directory to save corner detection visualization",
    )
    parser.add_argument(
        "--min-images",
        type=int,
        default=12,
        help="Minimum valid images required to run calibration",
    )
    return parser.parse_args()


def ensure_parent(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def list_images(patterns: str) -> List[str]:
    paths: List[str] = []
    for pat in patterns.split(","):
        pat = pat.strip()
        if not pat:
            continue
        paths.extend(glob.glob(pat))
    unique_sorted = sorted(set(paths))
    return [p for p in unique_sorted if os.path.isfile(p)]


def build_object_points(cols: int, rows: int, square_size_mm: float) -> np.ndarray:
    grid = np.zeros((rows * cols, 3), np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    grid *= square_size_mm
    return grid


def find_corners(image_bgr: np.ndarray, pattern_size: Tuple[int, int]) -> Tuple[bool, np.ndarray]:
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)

    # Prefer the newer detector when available for better robustness.
    if hasattr(cv2, "findChessboardCornersSB"):
        ok, corners = cv2.findChessboardCornersSB(gray, pattern_size)
        if ok:
            return True, corners.astype(np.float32)

    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not ok:
        return False, corners

    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), term)
    return True, corners


def detect_all(
    image_paths: Sequence[str],
    cols: int,
    rows: int,
    debug_dir: str,
) -> Tuple[List[DetectionResult], List[str]]:
    pattern = (cols, rows)
    success: List[DetectionResult] = []
    failed: List[str] = []

    if debug_dir:
        os.makedirs(debug_dir, exist_ok=True)

    for path in image_paths:
        img = cv2.imread(path)
        if img is None:
            failed.append(path)
            continue

        h, w = img.shape[:2]
        ok, corners = find_corners(img, pattern)

        if ok:
            success.append(DetectionResult(image_path=path, corners=corners, image_size=(w, h)))

        if debug_dir:
            vis = img.copy()
            cv2.drawChessboardCorners(vis, pattern, corners, ok)
            stem = os.path.splitext(os.path.basename(path))[0]
            mark = "ok" if ok else "fail"
            out = os.path.join(debug_dir, f"{stem}_{mark}.jpg")
            cv2.imwrite(out, vis)

        if not ok:
            failed.append(path)

    return success, failed


def compute_reprojection_errors(
    object_points: Sequence[np.ndarray],
    image_points: Sequence[np.ndarray],
    rvecs: Sequence[np.ndarray],
    tvecs: Sequence[np.ndarray],
    k: np.ndarray,
    d: np.ndarray,
) -> Tuple[float, List[float]]:
    errors: List[float] = []
    for obj, img, rv, tv in zip(object_points, image_points, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rv, tv, k, d)
        err = cv2.norm(img, proj, cv2.NORM_L2) / len(proj)
        errors.append(float(err))
    mean_err = float(np.mean(errors)) if errors else float("nan")
    return mean_err, errors


def focal_to_fov_deg(fx: float, fy: float, width: int, height: int) -> Tuple[float, float]:
    hfov = 2.0 * math.degrees(math.atan(width / (2.0 * fx)))
    vfov = 2.0 * math.degrees(math.atan(height / (2.0 * fy)))
    return hfov, vfov


def main() -> int:
    args = parse_args()
    image_paths = list_images(args.images)

    if not image_paths:
        print("[ERROR] No images matched pattern(s):", args.images)
        return 2

    detections, failed = detect_all(image_paths, args.cols, args.rows, args.debug_dir)
    if len(detections) < args.min_images:
        print(f"[ERROR] Valid chessboard detections: {len(detections)} (< min_images={args.min_images})")
        print("[HINT] Check cols/rows, image quality, and board coverage.")
        if failed:
            print("[INFO] Failed images:")
            for p in failed:
                print("  -", p)
        return 3

    image_size = detections[0].image_size
    if any(d.image_size != image_size for d in detections):
        print("[ERROR] Mixed image sizes detected. Use one fixed resolution for calibration.")
        return 4

    obj_template = build_object_points(args.cols, args.rows, args.square_size_mm)
    object_points = [obj_template.copy() for _ in detections]
    image_points = [d.corners for d in detections]

    rms, k, d, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )

    mean_err, per_image_err = compute_reprojection_errors(object_points, image_points, rvecs, tvecs, k, d)

    fx = float(k[0, 0])
    fy = float(k[1, 1])
    cx = float(k[0, 2])
    cy = float(k[1, 2])
    hfov_deg, vfov_deg = focal_to_fov_deg(fx, fy, image_size[0], image_size[1])

    result = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "input": {
            "images_pattern": args.images,
            "images_total": len(image_paths),
            "images_used": len(detections),
            "images_failed": len(failed),
            "board_inner_corners": {"cols": args.cols, "rows": args.rows},
            "square_size_mm": args.square_size_mm,
        },
        "image_size": {"width": image_size[0], "height": image_size[1]},
        "calibration": {
            "model": "opencv_pinhole",
            "rms_reprojection_error": float(rms),
            "mean_reprojection_error": mean_err,
            "camera_matrix": k.tolist(),
            "dist_coeffs": d.reshape(-1).tolist(),
            "intrinsics": {"fx": fx, "fy": fy, "cx": cx, "cy": cy},
            "fov_deg": {"hfov": hfov_deg, "vfov": vfov_deg},
            "per_image_reprojection_error": per_image_err,
        },
        "images": {
            "used": [d.image_path for d in detections],
            "failed": failed,
        },
    }

    ensure_parent(args.output)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(result, f, indent=2, ensure_ascii=False)

    print("[OK] Calibration finished")
    print(f"  - images used: {len(detections)} / {len(image_paths)}")
    print(f"  - RMS reprojection error: {float(rms):.6f}")
    print(f"  - mean reprojection error: {mean_err:.6f}")
    print(f"  - fx, fy: {fx:.3f}, {fy:.3f}")
    print(f"  - cx, cy: {cx:.3f}, {cy:.3f}")
    print(f"  - hfov, vfov (deg): {hfov_deg:.3f}, {vfov_deg:.3f}")
    print(f"  - output: {args.output}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
