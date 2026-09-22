#!/usr/bin/env python3
"""Iterate line-follow params from existing telemetry logs (no simulator launch)."""

from __future__ import annotations

import argparse
import glob
import json
import math
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
    p = argparse.ArgumentParser(description="Iterate line-follow params from logs")
    p.add_argument("--logs-glob", default="generated/auto_tune_logs*/trial_*.log")
    p.add_argument("--params-json", default="line_follow_params.json")
    p.add_argument("--output", default="generated/iterate_line_follow_report.json")
    p.add_argument("--apply", action="store_true", help="Apply recommended changes to params JSON")
    return p.parse_args()


def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def load_records(paths: List[str]) -> List[Dict[str, float]]:
    recs: List[Dict[str, float]] = []
    for path in paths:
        try:
            with open(path, "r", encoding="utf-8", errors="ignore") as f:
                for line in f:
                    m = TELEMETRY_RE.search(line)
                    if not m:
                        continue
                    recs.append(
                        {
                            "th": float(m.group("th")),
                            "steer": float(m.group("steer")),
                            "ex": float(m.group("ex")),
                            "ang": float(m.group("ang")),
                            "z": float(m.group("z")),
                            "tg": float(m.group("tg")),
                            "mode": float(m.group("mode")),
                            "conf": float(m.group("conf")),
                            "lost": float(m.group("lost")),
                            "L": float(m.group("L")),
                            "R": float(m.group("R")),
                            "source": path,
                        }
                    )
        except Exception:
            continue
    return recs


def mean(vals: List[float]) -> float:
    return sum(vals) / max(len(vals), 1)


def compute_metrics(recs: List[Dict[str, float]]) -> Dict[str, float]:
    if not recs:
        return {
            "samples": 0.0,
            "lost_ratio": 1.0,
            "valid_ratio": 0.0,
            "mean_abs_ex": 0.0,
            "mean_abs_ang": 0.0,
            "mean_conf": 0.0,
            "mean_tg": 0.0,
        }

    valid = [r for r in recs if r["conf"] > 0.01 and r["lost"] <= 0.5]
    return {
        "samples": float(len(recs)),
        "lost_ratio": mean([1.0 if r["lost"] > 0.5 else 0.0 for r in recs]),
        "valid_ratio": float(len(valid)) / float(len(recs)),
        "mean_abs_ex": mean([abs(r["ex"]) for r in valid]) if valid else 0.0,
        "mean_abs_ang": mean([abs(r["ang"]) for r in valid]) if valid else 0.0,
        "mean_conf": mean([r["conf"] for r in recs]),
        "mean_tg": mean([r["tg"] for r in recs]),
    }


def recommend(cfg: Dict, metrics: Dict[str, float]) -> Dict[str, float]:
    rec = {}

    roi = cfg.setdefault("roi", {})
    lost = cfg.setdefault("lost", {})
    webots = cfg.setdefault("webots", {})
    fusion = cfg.setdefault("fusion", {})

    min_valid_lines = int(roi.get("min_valid_lines", 2))
    conf_min = float(roi.get("conf_min", 0.12))
    width_std_max = float(roi.get("width_std_max", 20.0))
    min_line_w = int(roi.get("min_line_width", 2))
    max_line_w = int(roi.get("max_line_width", 80))

    base_speed = float(webots.get("base_speed", 2.3))
    lost_search_turn = float(lost.get("search_turn", 12.0))
    lookahead = float(fusion.get("lookahead_gain", 0.12))

    if metrics["lost_ratio"] > 0.45:
        min_valid_lines = max(2, min_valid_lines - 1)
        conf_min = clamp(conf_min * 0.85, 0.05, 0.25)
        width_std_max = clamp(width_std_max + 3.0, 10.0, 40.0)
        min_line_w = max(1, min_line_w - 1)
        max_line_w = min(140, max_line_w + 15)
        base_speed = clamp(base_speed * 0.9, 1.2, 3.2)
        lost_search_turn = clamp(lost_search_turn * 0.92, 7.0, 24.0)

    if metrics["valid_ratio"] > 0.55 and metrics["mean_abs_ex"] > 4.0:
        lookahead = clamp(lookahead * 0.92, 0.05, 0.30)
        base_speed = clamp(base_speed * 0.93, 1.2, 3.2)

    if metrics["valid_ratio"] > 0.7 and metrics["mean_abs_ex"] < 2.0 and metrics["lost_ratio"] < 0.1:
        base_speed = clamp(base_speed * 1.05, 1.2, 3.2)

    rec["roi.min_valid_lines"] = min_valid_lines
    rec["roi.conf_min"] = round(conf_min, 4)
    rec["roi.width_std_max"] = round(width_std_max, 3)
    rec["roi.min_line_width"] = min_line_w
    rec["roi.max_line_width"] = max_line_w
    rec["webots.base_speed"] = round(base_speed, 3)
    rec["lost.search_turn"] = round(lost_search_turn, 3)
    rec["fusion.lookahead_gain"] = round(lookahead, 4)

    return rec


def apply_recommendation(cfg: Dict, rec: Dict[str, float]) -> None:
    for path, value in rec.items():
        parts = path.split(".")
        cur = cfg
        for k in parts[:-1]:
            if k not in cur or not isinstance(cur[k], dict):
                cur[k] = {}
            cur = cur[k]
        cur[parts[-1]] = value


def main() -> int:
    args = parse_args()
    repo = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

    params_path = os.path.join(repo, args.params_json)
    output_path = os.path.join(repo, args.output)

    if not os.path.isfile(params_path):
        print("[ERROR] params file not found:", params_path)
        return 2

    log_paths = sorted(glob.glob(os.path.join(repo, args.logs_glob)))
    records = load_records(log_paths)

    with open(params_path, "r", encoding="utf-8") as f:
        cfg = json.load(f)

    metrics = compute_metrics(records)
    rec = recommend(cfg, metrics)

    report = {
        "logs_glob": args.logs_glob,
        "log_files": len(log_paths),
        "samples": int(metrics["samples"]),
        "metrics": metrics,
        "recommended": rec,
        "applied": bool(args.apply),
    }

    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(report, f, indent=2, ensure_ascii=False)

    print("[INFO] logs:", len(log_paths), "files")
    print("[INFO] samples:", int(metrics["samples"]))
    print("[INFO] lost_ratio:", round(metrics["lost_ratio"], 4), "valid_ratio:", round(metrics["valid_ratio"], 4))
    print("[INFO] report:", args.output)

    if args.apply:
        apply_recommendation(cfg, rec)
        with open(params_path, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
        print("[OK] applied recommendation to", args.params_json)
    else:
        print("[OK] dry-run only (no params updated)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
