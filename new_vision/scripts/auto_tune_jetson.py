"""jetson_bridge 参数自动搜索 — 环境变量传参，跑多组按跟踪质量排序"""
import os, subprocess, sys, time, re

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WORLD = os.path.join(REPO, "Webots", "worlds", "Robocup.wbt")
WEBOTS = r"D:\Webots\msys64\mingw64\bin\webots.exe"
LOG = os.path.join(REPO, "generated", "jetson_bridge_log.txt")
RUN_SEC = 20

PARAM_GRID = [
    # (KP, KI, KD, STEER_SCALE) — 压低 KP，拉高 STEER_SCALE 匹配老代码 steer 范围
    (0.4, 0.010, 0.10, 1.0),
    (0.5, 0.008, 0.10, 1.0),
    (0.3, 0.015, 0.08, 1.2),
    (0.45, 0.006, 0.12, 0.9),
    (0.6, 0.005, 0.10, 1.1),
    (0.35, 0.012, 0.08, 1.0),
    (0.55, 0.004, 0.12, 1.3),
]


def run_one(kp, ki, kd, steer_scale):
    """跑一次仿真，返回日志行列表"""
    for p in ["webots", "webots-bin"]:
        os.system(f'taskkill /f /im {p}.exe 2>nul')
    time.sleep(2)
    if os.path.exists(LOG):
        os.remove(LOG)

    env = os.environ.copy()
    env["LINE_FOLLOW_MAX_SECONDS"] = str(RUN_SEC)
    env["JETSON_PID_STRAIGHT_KP"] = str(kp)
    env["JETSON_PID_STRAIGHT_KI"] = str(ki)
    env["JETSON_PID_STRAIGHT_KD"] = str(kd)
    env["JETSON_STEER_SCALE"] = str(steer_scale)
    cmd = [WEBOTS, "--batch", "--mode=fast", "--stdout", "--stderr",
           "--no-rendering", "--minimize", WORLD]

    proc = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT, text=True,
                            encoding="utf-8", errors="ignore", env=env)
    try:
        proc.communicate(timeout=RUN_SEC + 30)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.communicate()

    if os.path.exists(LOG):
        with open(LOG, "r", encoding="utf-8", errors="ignore") as f:
            return f.readlines()
    return []


def score(log_lines):
    """评分：跟踪时间长 + 偏差小 + 不过饱和"""
    dev_vals = []
    steer_vals = []
    conf_vals = []
    for line in log_lines:
        m = re.search(r'dev=([-\d.]+)px', line)
        if m:
            dev_vals.append(abs(float(m.group(1))))
        m = re.search(r'steer=([-\d.]+)', line)
        if m:
            steer_vals.append(abs(float(m.group(1))))
        m = re.search(r'conf=([\d.]+)', line)
        if m:
            conf_vals.append(float(m.group(1)))

    if not dev_vals:
        return -999, {}

    n = len(dev_vals)
    avg_dev = sum(dev_vals) / n
    avg_steer = sum(steer_vals) / n if steer_vals else 0
    avg_conf = sum(conf_vals) / n if conf_vals else 0
    # 饱和率（steer > 40 算饱和）
    sat_rate = sum(1 for s in steer_vals if s > 40) / max(n, 1)

    # 分数：帧数多 + 平均偏差小 + 饱和少
    score_val = n * avg_conf * (1.0 - sat_rate) / max(avg_dev + 5, 1)
    return round(score_val, 2), {
        "frames": n, "avg_dev": round(avg_dev, 1),
        "avg_steer": round(avg_steer, 1), "avg_conf": round(avg_conf, 2),
        "sat_rate": round(sat_rate, 2),
    }


def main():
    results = []
    for i, (kp, ki, kd, ss, *_) in enumerate(PARAM_GRID):
        label = f"KP={kp} KI={ki} KD={kd} STEER_SCALE={ss}"
        print(f"[{i+1}/{len(PARAM_GRID)}] {label}")
        lines = run_one(kp, ki, kd, ss)
        s, stats = score(lines)
        stats["params"] = label
        stats["score"] = s
        results.append(stats)
        print(f"  score={s}  frames={stats['frames']}  avg_dev={stats['avg_dev']}  avg_steer={stats['avg_steer']}  sat={stats['sat_rate']}")

    print("\n=== ranked ===")
    results.sort(key=lambda r: r["score"], reverse=True)
    for r in results:
        print(f"  score={r['score']:5.1f}  {r['params']}")
        print(f"    frames={r['frames']}  avg_dev={r['avg_dev']}  avg_steer={r['avg_steer']}  conf={r['avg_conf']}  sat={r['sat_rate']}")


if __name__ == "__main__":
    main()
