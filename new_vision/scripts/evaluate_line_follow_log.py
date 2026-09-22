#!/usr/bin/env python3
"""Evaluate line-follow quality from telemetry log and print a simple verdict."""

from __future__ import annotations

import argparse
import json
import os
import re
from typing import Dict, List

TELEMETRY_RE = re.compile(
    r"th=(?P<th>\d+)\s+"
    r"steer=(?P<steer>-?\d+(?:\.\d+)?)\s+"
    r"ex=(?P<ex>-?\d+(?:\.\d+)?)cm\s+"
    r"ang=(?P<ang>-?\d+(?:\.\d+)?)deg\s+"
    r"z=(?P<z>-?\d+(?:\.\d+)?)cm\s+"
    r"tg=(?P<tg>-?\d+(?:\.\d+)?)\s+"
    r"mode=(?P<mode>\d+)\s+"
    r"conf=(?P<conf>-?\d+(?:\.\d+)?)\s+"
    r"lost=(?P<lost>\d+)\s+"
    r"L=(?P<L>-?\d+(?:\.\d+)?)\s+"
    r"R=(?P<R>-?\d+(?:\.\d+)?)"
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Evaluate line-follow telemetry log")
    p.add_argument("--log", default="generated/manual_webots_logs/trial_000.log")
    p.add_argument("--output", default="generated/manual_webots_quality.json")
    p.add_argument("--params-json", default="line_follow_params.json")
    p.add_argument("--in-lane-abs-ex-cm", type=float, default=0.0)
    p.add_argument("--in-lane-min-conf", type=float, default=0.0)
    return p.parse_args()


def mean(values: List[float]) -> float:
    return sum(values) / max(len(values), 1)


def percentile(values: List[float], p: float) -> float:
    if not values:
        return 0.0
    s = sorted(values)
    idx = int(round((len(s) - 1) * p))
    if idx < 0:
        idx = 0
    if idx >= len(s):
        idx = len(s) - 1
    return s[idx]


def grade(v: float, th_good: float, th_ok: float, lower_is_better: bool = True) -> str:
    if lower_is_better:
        if v <= th_good:
            return "A"
        if v <= th_ok:
            return "B"
        return "C"
    if v >= th_good:
        return "A"
    if v >= th_ok:
        return "B"
    return "C"


def load_thresholds(args: argparse.Namespace) -> Dict[str, float]:
    thresholds = {
        "in_lane_abs_ex_cm": 8.0,
        "in_lane_min_conf": 0.20,
    }

    params_path = args.params_json
    if not os.path.isabs(params_path):
        params_path = os.path.join(os.path.dirname(__file__), "..", params_path)
    params_path = os.path.abspath(params_path)

    if os.path.isfile(params_path):
        try:
            with open(params_path, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            eval_cfg = cfg.get("evaluation", {})
            if "in_lane_abs_ex_cm" in eval_cfg:
                thresholds["in_lane_abs_ex_cm"] = float(eval_cfg["in_lane_abs_ex_cm"])
            if "in_lane_min_conf" in eval_cfg:
                thresholds["in_lane_min_conf"] = float(eval_cfg["in_lane_min_conf"])
        except Exception:
            pass

    if args.in_lane_abs_ex_cm > 0:
        thresholds["in_lane_abs_ex_cm"] = float(args.in_lane_abs_ex_cm)
    if args.in_lane_min_conf > 0:
        thresholds["in_lane_min_conf"] = float(args.in_lane_min_conf)

    return thresholds


def main() -> int:
    args = parse_args()
    thresholds = load_thresholds(args)

    if not os.path.isfile(args.log):
        print("[ERROR] log not found:", args.log)
        return 2

    rows: List[Dict[str, float]] = []
    with open(args.log, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = TELEMETRY_RE.search(line)
            if not m:
                continue
            L = float(m.group("L"))
            R = float(m.group("R"))
            rows.append(
                {
                    "ex": float(m.group("ex")),
                    "ang": float(m.group("ang")),
                    "conf": float(m.group("conf")),
                    "lost": float(m.group("lost")),
                    "v": (abs(L) + abs(R)) * 0.5,
                    "steer": float(m.group("steer")),
                }
            )

    if not rows:
        print("[ERROR] no telemetry rows parsed")
        return 3

    valid = [r for r in rows if r["conf"] > 0.01 and r["lost"] <= 0.5]
    abs_ex = [abs(r["ex"]) for r in valid]
    abs_ang = [abs(r["ang"]) for r in valid]

    def is_in_lane(r: Dict[str, float]) -> bool:
        return (
            r["lost"] <= 0.5
            and r["conf"] >= thresholds["in_lane_min_conf"]
            and abs(r["ex"]) <= thresholds["in_lane_abs_ex_cm"]
        )

    in_lane_flags = [is_in_lane(r) for r in rows]

    max_out_streak = 0
    out_streak = 0
    for ok in in_lane_flags:
        if ok:
            out_streak = 0
        else:
            out_streak += 1
            if out_streak > max_out_streak:
                max_out_streak = out_streak

    metrics = {
        "samples": len(rows),
        "valid_ratio": len(valid) / float(len(rows)),
        "lost_ratio": mean([1.0 if r["lost"] > 0.5 else 0.0 for r in rows]),
        "mean_abs_ex_cm": mean(abs_ex) if abs_ex else 999.0,
        "p95_abs_ex_cm": percentile(abs_ex, 0.95) if abs_ex else 999.0,
        "mean_abs_ang_deg": mean(abs_ang) if abs_ang else 999.0,
        "mean_conf": mean([r["conf"] for r in rows]),
        "mean_speed": mean([r["v"] for r in rows]),
        "low_speed_ratio_v_lt_1_2": mean([1.0 if r["v"] < 1.2 else 0.0 for r in rows]),
        "in_lane_ratio": mean([1.0 if x else 0.0 for x in in_lane_flags]),
        "out_lane_ratio": mean([0.0 if x else 1.0 for x in in_lane_flags]),
        "max_out_lane_streak_frames": int(max_out_streak),
        "max_out_lane_streak_sec": float(max_out_streak) * 0.25,
    }

    grades = {
        "lost": grade(metrics["lost_ratio"], 0.08, 0.20, lower_is_better=True),
        "offset": grade(metrics["mean_abs_ex_cm"], 3.0, 7.0, lower_is_better=True),
        "confidence": grade(metrics["mean_conf"], 0.70, 0.45, lower_is_better=False),
        "speed": grade(metrics["mean_speed"], 2.0, 1.5, lower_is_better=False),
        "in_lane": grade(metrics["in_lane_ratio"], 0.90, 0.75, lower_is_better=False),
    }

    score = 0
    for g in grades.values():
        score += {"A": 2, "B": 1, "C": 0}[g]

    if score >= 7:
        verdict = "GOOD"
    elif score >= 4:
        verdict = "OK"
    else:
        verdict = "POOR"

    report = {
        "log": args.log,
        "thresholds": thresholds,
        "metrics": metrics,
        "grades": grades,
        "verdict": verdict,
    }

    out_dir = os.path.dirname(args.output)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print("[INFO] verdict:", verdict)
    print("[INFO] samples:", metrics["samples"])
    print("[INFO] lost_ratio:", round(metrics["lost_ratio"], 3), "grade", grades["lost"])
    print("[INFO] mean_abs_ex_cm:", round(metrics["mean_abs_ex_cm"], 3), "grade", grades["offset"])
    print("[INFO] mean_conf:", round(metrics["mean_conf"], 3), "grade", grades["confidence"])
    print("[INFO] mean_speed:", round(metrics["mean_speed"], 3), "grade", grades["speed"])
    print("[INFO] in_lane_ratio:", round(metrics["in_lane_ratio"], 3), "grade", grades["in_lane"])
    print("[INFO] max_out_lane_streak_sec:", round(metrics["max_out_lane_streak_sec"], 3))
    print("[INFO] output:", args.output)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
