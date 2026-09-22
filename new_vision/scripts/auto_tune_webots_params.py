#!/usr/bin/env python3
"""Auto tune line following parameters by running Webots in black-box mode.

The tuner:
1) Samples candidate parameters.
2) Writes them into line_follow_params.json.
3) Runs Webots for a fixed duration.
4) Parses controller telemetry printed by line_follow_transfer.
5) Minimizes a composite score (lower is better).

Typical usage:
  python scripts/auto_tune_webots_params.py \
    --trials 24 \
    --run-seconds 16 \
    --webots-cmd "webots --batch --mode=fast --stdout --stderr \"{world}\"" \
    --apply-best
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import os
import random
import re
import shutil
import subprocess
import time
from datetime import datetime
from typing import Dict, List, Tuple

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
    parser = argparse.ArgumentParser(description="Auto tune Webots line-following params")
    parser.add_argument("--params-json", default="line_follow_params.json")
    parser.add_argument("--world", default="Webots/worlds/Robocup.wbt")
    parser.add_argument(
        "--webots-exe",
        default="",
        help="Absolute path to webots executable (e.g. D:/Webots/.../webots.exe).",
    )
    parser.add_argument(
        "--webots-cmd",
        default="",
        help="Command template. Must include {world} placeholder.",
    )
    parser.add_argument("--trials", type=int, default=24)
    parser.add_argument("--run-seconds", type=int, default=16)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--apply-best", action="store_true")
    parser.add_argument("--output", default="generated/auto_tune_result.json")
    parser.add_argument("--save-logs", action="store_true")
    parser.add_argument("--logs-dir", default="generated/auto_tune_logs")
    return parser.parse_args()


def ensure_parent(path: str) -> None:
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)


def deep_get(data: Dict, path: str):
    cur = data
    for key in path.split("."):
        cur = cur[key]
    return cur


def deep_set(data: Dict, path: str, value) -> None:
    keys = path.split(".")
    cur = data
    for k in keys[:-1]:
        if k not in cur or not isinstance(cur[k], dict):
            cur[k] = {}
        cur = cur[k]
    cur[keys[-1]] = value


def mean(values: List[float]) -> float:
    return sum(values) / max(len(values), 1)


def percentile(values: List[float], p: float) -> float:
    if not values:
        return float("nan")
    s = sorted(values)
    idx = int(round((len(s) - 1) * p))
    return s[max(0, min(len(s) - 1, idx))]


def make_search_space(base: Dict) -> Dict[str, Tuple[float, float, str]]:
    # Conservative search ranges around current setup.
    return {
        "turn.start_dist_cm": (26.0, 40.0, "float3"),
        "turn.err_cm": (4.0, 10.0, "float3"),
        "fusion.lookahead_gain": (0.05, 0.28, "float3"),
        "fusion.curve_gain": (0.06, 0.22, "float3"),
        "fusion.angle_gain": (0.08, 0.20, "float3"),
        "pid.curve_switch_cm": (5.0, 9.0, "float3"),
        "pid.curve.kp": (0.55, 0.95, "float3"),
        "pid.curve.ki": (0.001, 0.012, "float4"),
        "pid.curve.kd": (0.08, 0.20, "float3"),
        "steer.sat": (36.0, 60.0, "float3"),
        "steer.scale": (24.0, 40.0, "float3"),
        "webots.base_speed": (2.2, 3.2, "float3"),
        "webots.steer_to_wheel": (0.018, 0.032, "float4"),
        "lost.search_turn": (12.0, 24.0, "float3"),
    }


def sample_candidate(base: Dict, rng: random.Random, search_space: Dict[str, Tuple[float, float, str]]) -> Tuple[Dict, Dict[str, float]]:
    candidate = copy.deepcopy(base)
    flat: Dict[str, float] = {}
    for path, (lo, hi, kind) in search_space.items():
        v = lo + (hi - lo) * rng.random()
        if kind == "float4":
            v = round(v, 4)
        else:
            v = round(v, 3)
        deep_set(candidate, path, v)
        flat[path] = v
    return candidate, flat


def run_webots(command: str, cwd: str, timeout_s: int) -> Tuple[str, bool, int]:
    proc = subprocess.Popen(
        command,
        cwd=cwd,
        shell=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="ignore",
    )

    timed_out = False
    try:
        output, _ = proc.communicate(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        timed_out = True
        proc.terminate()
        try:
            output, _ = proc.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            proc.kill()
            output, _ = proc.communicate()

    return output or "", timed_out, int(proc.returncode or 0)


def parse_telemetry(log_text: str) -> List[Dict[str, float]]:
    records: List[Dict[str, float]] = []
    for line in log_text.splitlines():
        m = TELEMETRY_RE.search(line)
        if not m:
            continue
        rec = {
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
        }
        records.append(rec)
    return records


def compute_metrics(records: List[Dict[str, float]], steer_sat: float) -> Dict[str, float]:
    abs_ex = [abs(r["ex"]) for r in records]
    abs_ang = [abs(r["ang"]) for r in records]
    abs_steer = [abs(r["steer"]) for r in records]
    conf = [r["conf"] for r in records]

    lost_ratio = mean([1.0 if r["lost"] > 0 else 0.0 for r in records])
    sat_ratio = mean([1.0 if s > (0.9 * steer_sat) else 0.0 for s in abs_steer])

    metrics = {
        "samples": float(len(records)),
        "mean_abs_ex_cm": mean(abs_ex),
        "p95_abs_ex_cm": percentile(abs_ex, 0.95),
        "mean_abs_ang_deg": mean(abs_ang),
        "steer_rms": math.sqrt(mean([s * s for s in abs_steer])),
        "lost_ratio": lost_ratio,
        "saturation_ratio": sat_ratio,
        "conf_mean": mean(conf),
    }
    return metrics


def score_metrics(metrics: Dict[str, float], min_samples: int) -> float:
    if int(metrics["samples"]) < min_samples:
        # Hard penalty for unstable/no telemetry runs.
        return 1e9 + (min_samples - metrics["samples"]) * 1e6

    score = 0.0
    score += 1.7 * metrics["mean_abs_ex_cm"]
    score += 1.2 * metrics["p95_abs_ex_cm"]
    score += 0.25 * metrics["mean_abs_ang_deg"]
    score += 0.30 * metrics["steer_rms"]
    score += 50.0 * metrics["lost_ratio"]
    score += 22.0 * metrics["saturation_ratio"]
    score -= 2.0 * metrics["conf_mean"]
    return float(score)


def write_json(path: str, data: Dict) -> None:
    ensure_parent(path)
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)


def resolve_webots_cmd(webots_cmd: str, webots_exe: str) -> str:
    if webots_cmd:
        return webots_cmd

    if webots_exe:
        exe = os.path.abspath(webots_exe)
        if os.path.isfile(exe):
            return f'"{exe}" --batch --mode=fast --stdout --stderr "{{world}}"'
        print("[WARN] --webots-exe not found:", exe)

    candidates: List[str] = []
    which_webots = shutil.which("webots")
    if which_webots:
        candidates.append(which_webots)

    # Common Windows install locations for Webots.
    candidates.extend(
        [
            r"D:\Webots\msys64\mingw64\bin\webots.exe",
            r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
            os.path.expandvars(r"%LOCALAPPDATA%\Programs\Webots\msys64\mingw64\bin\webots.exe"),
            os.path.expandvars(r"%LOCALAPPDATA%\Programs\Cyberbotics\Webots\msys64\mingw64\bin\webots.exe"),
        ]
    )

    for exe in candidates:
        if exe and os.path.isfile(exe):
            return f'"{exe}" --batch --mode=fast --stdout --stderr "{{world}}"'

    return 'webots --batch --mode=fast --stdout --stderr "{world}"'


def main() -> int:
    args = parse_args()
    rng = random.Random(args.seed)

    repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    params_path = os.path.join(repo_root, args.params_json)
    world_path = os.path.join(repo_root, args.world)

    if not os.path.isfile(params_path):
        print("[ERROR] params file not found:", params_path)
        return 2
    if not os.path.isfile(world_path):
        print("[ERROR] world file not found:", world_path)
        return 3
    resolved_webots_cmd = resolve_webots_cmd(args.webots_cmd, args.webots_exe)
    if "{world}" not in resolved_webots_cmd:
        print("[ERROR] --webots-cmd must contain {world} placeholder")
        return 4
    print("[INFO] webots command:", resolved_webots_cmd)

    with open(params_path, "r", encoding="utf-8") as f:
        base_params = json.load(f)

    with open(params_path, "r", encoding="utf-8") as f:
        backup_text = f.read()

    search_space = make_search_space(base_params)

    trial_results: List[Dict] = []
    best_idx = -1
    best_score = float("inf")
    best_params = None

    if args.save_logs:
        os.makedirs(os.path.join(repo_root, args.logs_dir), exist_ok=True)

    try:
        for trial in range(args.trials):
            if trial == 0:
                # First run = baseline.
                candidate = copy.deepcopy(base_params)
                flat = {k: deep_get(candidate, k) for k in search_space.keys()}
                label = "baseline"
            else:
                candidate, flat = sample_candidate(base_params, rng, search_space)
                label = "sampled"

            with open(params_path, "w", encoding="utf-8") as f:
                json.dump(candidate, f, indent=2, ensure_ascii=False)

            command = resolved_webots_cmd.format(world=world_path)
            log_text, timed_out, return_code = run_webots(command, repo_root, args.run_seconds)
            records = parse_telemetry(log_text)

            steer_sat = float(flat["steer.sat"])
            metrics = compute_metrics(records, steer_sat)
            score = score_metrics(metrics, args.min_samples)

            result = {
                "trial": trial,
                "label": label,
                "timed_out": timed_out,
                "return_code": return_code,
                "score": score,
                "metrics": metrics,
                "params": flat,
            }
            trial_results.append(result)

            if args.save_logs:
                log_path = os.path.join(repo_root, args.logs_dir, f"trial_{trial:03d}.log")
                with open(log_path, "w", encoding="utf-8") as f:
                    f.write(log_text)

            # Webots batch may still return non-zero while producing valid telemetry.
            is_valid = int(metrics["samples"]) >= args.min_samples
            if is_valid and score < best_score:
                best_score = score
                best_idx = trial
                best_params = copy.deepcopy(candidate)

            print(
                f"[TRIAL {trial:02d}] score={score:.4f}, rc={return_code}, samples={int(metrics['samples'])}, "
                f"mean_abs_ex={metrics['mean_abs_ex_cm']:.3f}, lost_ratio={metrics['lost_ratio']:.3f}"
            )

    finally:
        if args.apply_best and best_params is not None:
            with open(params_path, "w", encoding="utf-8") as f:
                json.dump(best_params, f, indent=2, ensure_ascii=False)
        else:
            with open(params_path, "w", encoding="utf-8") as f:
                f.write(backup_text)

    if best_idx < 0:
        ranked = sorted(trial_results, key=lambda x: x["score"])[:5]
        summary = {
            "timestamp": datetime.now().isoformat(timespec="seconds"),
            "trials": args.trials,
            "run_seconds": args.run_seconds,
            "params_json": args.params_json,
            "world": args.world,
            "webots_cmd": resolved_webots_cmd,
            "apply_best": args.apply_best,
            "best_trial": None,
            "top5": ranked,
            "all_trials": trial_results,
            "error": "No valid trial result (insufficient telemetry samples)",
        }
        output_path = os.path.join(repo_root, args.output)
        write_json(output_path, summary)
        print("[ERROR] No valid trial result")
        print("  output:", args.output)
        return 5

    ranked = sorted(trial_results, key=lambda x: x["score"])[:5]

    summary = {
        "timestamp": datetime.now().isoformat(timespec="seconds"),
        "trials": args.trials,
        "run_seconds": args.run_seconds,
        "params_json": args.params_json,
        "world": args.world,
        "webots_cmd": resolved_webots_cmd,
        "apply_best": args.apply_best,
        "best_trial": trial_results[best_idx],
        "top5": ranked,
        "all_trials": trial_results,
    }

    output_path = os.path.join(repo_root, args.output)
    write_json(output_path, summary)

    print("\n[OK] Auto tuning finished")
    print("  best trial:", best_idx)
    print("  best score:", f"{best_score:.4f}")
    print("  output:", args.output)
    if args.apply_best:
        print("  applied: best params written to", args.params_json)
    else:
        print("  applied: no (original params restored)")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
