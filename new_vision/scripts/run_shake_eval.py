#!/usr/bin/env python3
"""Run three Webots trials to quantify how camera shake hurts the controller
and how the shake_robust layer mitigates it.

Trials:
  baseline           : shake.enable=false, shake_robust.enable=true
  shake_on_no_robust : shake.enable=true,  shake_robust.enable=false
  shake_on_robust    : shake.enable=true,  shake_robust.enable=true

The script edits line_follow_params.json in place between trials and always
restores the original content on exit (success or failure).

Quick iteration: default --run-seconds is 18. For quicker smoke use 12; for steadier stats use 25.
"""

from __future__ import annotations

import argparse
import copy
import json
import os
import re
import shutil
import subprocess
import sys
import time
from typing import Dict, List, Tuple

REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

WEBOTS_CANDIDATES = [
	r"D:\Webots\msys64\mingw64\bin\webots.exe",
	r"C:\Program Files\Webots\msys64\mingw64\bin\webots.exe",
	os.path.expandvars(r"%LOCALAPPDATA%\Programs\Webots\msys64\mingw64\bin\webots.exe"),
	"webots",
]

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
DRMS_RE = re.compile(r"drms=(-?\d+(?:\.\d+)?)\s+sk=(\d+)")


def find_webots() -> str:
	for c in WEBOTS_CANDIDATES:
		if os.path.isfile(c):
			return c
	return "webots"


def run_one(label: str, world: str, log_path: str, run_seconds: int, env_extra: Dict[str, str]) -> Dict:
	exe = find_webots()
	# Avoid shell=True + terminate (it creates orphan webots processes on Windows).
	# Instead the controller reads LINE_FOLLOW_MAX_SECONDS and exits cleanly.
	cmd = [exe, "--batch", "--mode=fast", "--stdout", "--stderr",
		   "--no-rendering", "--minimize", world]
	env = os.environ.copy()
	env.update(env_extra)
	env["LINE_FOLLOW_PROTO_TRANSPORT"] = "none"
	env["LINE_FOLLOW_MAX_SECONDS"] = str(int(run_seconds))

	print(f"[INFO] {label}: launching {exe} (max {run_seconds}s)")
	t0 = time.time()
	proc = subprocess.Popen(
		cmd,
		cwd=REPO_ROOT,
		stdout=subprocess.PIPE,
		stderr=subprocess.STDOUT,
		text=True,
		encoding="utf-8",
		errors="ignore",
		env=env,
	)
	timed_out = False
	# Webots --batch may keep running after the controller returns unless Supervisor calls simulationQuit.
	# Keep margin above run_seconds for startup + teardown (OpenCV remap init, etc.).
	hard_limit = max(run_seconds + 35, int(run_seconds * 2.5 + 30))
	try:
		output, _ = proc.communicate(timeout=hard_limit)
	except subprocess.TimeoutExpired:
		timed_out = True
		try:
			proc.terminate()
			output, _ = proc.communicate(timeout=8)
		except subprocess.TimeoutExpired:
			proc.kill()
			try:
				output, _ = proc.communicate(timeout=5)
			except subprocess.TimeoutExpired:
				output = ""
	elapsed = time.time() - t0
	with open(log_path, "w", encoding="utf-8") as f:
		f.write(output or "")
	print(f"[INFO] {label}: done in {elapsed:.1f}s, log_bytes={len(output or '')}, timed_out={timed_out}")

	# parse
	rows: List[Dict[str, float]] = []
	for line in (output or "").splitlines():
		m = TELEMETRY_RE.search(line)
		if not m:
			continue
		row = {
			"ex": float(m.group("ex")),
			"ang": float(m.group("ang")),
			"conf": float(m.group("conf")),
			"lost": int(m.group("lost")),
			"steer": float(m.group("steer")),
			"L": float(m.group("L")),
			"R": float(m.group("R")),
		}
		d = DRMS_RE.search(line)
		if d:
			row["drms"] = float(d.group(1))
			row["sk"] = int(d.group(2))
		rows.append(row)

	if not rows:
		return {"label": label, "samples": 0, "elapsed_s": elapsed}

	abs_ex = sorted(abs(r["ex"]) for r in rows)
	mean = sum(abs_ex) / len(abs_ex)
	p95 = abs_ex[int(round(0.95 * (len(abs_ex) - 1)))]
	maxv = abs_ex[-1]
	lost_ratio = sum(1 for r in rows if r["lost"] > 0) / len(rows)
	mean_conf = sum(r["conf"] for r in rows) / len(rows)

	abs_steer = [abs(r["steer"]) for r in rows]
	import math as _m
	steer_rms = _m.sqrt(sum(s * s for s in abs_steer) / max(1, len(abs_steer)))
	d_steer = [abs(rows[i]["steer"] - rows[i - 1]["steer"]) for i in range(1, len(rows))]
	mean_d_steer = (sum(d_steer) / len(d_steer)) if d_steer else 0.0
	max_d_steer = max(d_steer) if d_steer else 0.0

	# Wheel asymmetry as a proxy for control activity
	wheel_diff = [abs(r["L"] - r["R"]) for r in rows]
	mean_wheel_diff = sum(wheel_diff) / max(1, len(wheel_diff))

	drms_vals = [r["drms"] for r in rows if "drms" in r]
	sk_active_ratio = (sum(1 for r in rows if r.get("sk", 0) > 0) / len(rows)) if drms_vals else 0.0
	mean_drms = (sum(drms_vals) / len(drms_vals)) if drms_vals else 0.0

	return {
		"label": label,
		"samples": len(rows),
		"elapsed_s": round(elapsed, 2),
		"mean_abs_ex_cm": round(mean, 3),
		"p95_abs_ex_cm": round(p95, 3),
		"max_abs_ex_cm": round(maxv, 3),
		"lost_ratio": round(lost_ratio, 4),
		"mean_conf": round(mean_conf, 3),
		"steer_rms": round(steer_rms, 3),
		"mean_d_steer": round(mean_d_steer, 3),
		"max_d_steer": round(max_d_steer, 3),
		"mean_wheel_diff": round(mean_wheel_diff, 3),
		"shake_active_ratio": round(sk_active_ratio, 4),
		"mean_drms_px": round(mean_drms, 3),
	}


def patch_cfg(cfg: Dict, shake_on: bool, robust_on: bool) -> Dict:
	new_cfg = copy.deepcopy(cfg)
	new_cfg.setdefault("shake", {})["enable"] = bool(shake_on)
	new_cfg.setdefault("shake_robust", {})["enable"] = bool(robust_on)
	# Keep protocol transport off in eval to avoid file-locking concerns.
	new_cfg.setdefault("output", {}).setdefault("protocol", {})["transport"] = "none"
	return new_cfg


def main() -> int:
	ap = argparse.ArgumentParser()
	ap.add_argument("--world", default="Webots/worlds/Robocup.wbt")
	ap.add_argument("--params", default="line_follow_params.json")
	ap.add_argument("--out-dir", default="generated/shake_eval")
	ap.add_argument("--run-seconds", type=int, default=18,
	                help="Controller run per trial (LINE_FOLLOW_MAX_SECONDS). Default 18; 12 quick smoke; 25+ steadier.")
	ap.add_argument("--label-prefix", default="")
	args = ap.parse_args()

	params_path = os.path.join(REPO_ROOT, args.params)
	world_path = os.path.join(REPO_ROOT, args.world)
	out_dir = os.path.join(REPO_ROOT, args.out_dir)
	os.makedirs(out_dir, exist_ok=True)

	with open(params_path, "r", encoding="utf-8") as f:
		original = f.read()
	base_cfg = json.loads(original)

	trials: List[Tuple[str, bool, bool]] = [
		("baseline", False, True),
		("shake_on_no_robust", True, False),
		("shake_on_robust", True, True),
	]
	stamp = time.strftime("%Y%m%d_%H%M%S")
	results = []

	try:
		for label, shake_on, robust_on in trials:
			ctrl_log = os.path.join(out_dir, f"{args.label_prefix}{label}_{stamp}.log")
			# Apply patched cfg
			patched = patch_cfg(base_cfg, shake_on, robust_on)
			with open(params_path, "w", encoding="utf-8") as f:
				json.dump(patched, f, indent=2, ensure_ascii=False)
			res = run_one(label, world_path, ctrl_log, args.run_seconds, env_extra={})
			res["log"] = os.path.relpath(ctrl_log, REPO_ROOT)
			res["shake_enable"] = shake_on
			res["robust_enable"] = robust_on
			results.append(res)
	finally:
		with open(params_path, "w", encoding="utf-8") as f:
			f.write(original)
		print("[INFO] params restored")

	report = {"timestamp": stamp, "trials": results}
	report_path = os.path.join(out_dir, f"{args.label_prefix}report_{stamp}.json")
	with open(report_path, "w", encoding="utf-8") as f:
		json.dump(report, f, indent=2, ensure_ascii=False)

	print("\n========= shake-eval summary =========")
	header = f"{'label':22s} {'n':>4} {'mean_ex':>8} {'p95_ex':>7} {'st_rms':>7} {'d_st':>6} {'max_d':>6} {'wd':>5} {'sk%':>5}"
	print(header)
	print("-" * len(header))
	for r in results:
		print(f"{r['label']:22s} {r.get('samples', 0):4d} "
			  f"{r.get('mean_abs_ex_cm', 0):8.3f} {r.get('p95_abs_ex_cm', 0):7.3f} "
			  f"{r.get('steer_rms', 0):7.3f} {r.get('mean_d_steer', 0):6.2f} {r.get('max_d_steer', 0):6.2f} "
			  f"{r.get('mean_wheel_diff', 0):5.2f} {r.get('shake_active_ratio', 0):5.2f}")
	print(f"\n[INFO] full report: {os.path.relpath(report_path, REPO_ROOT)}")
	return 0


if __name__ == "__main__":
	sys.exit(main())
