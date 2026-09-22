"""Webots controller — V1 minimal. Detector does all vision/fusion; we do PID + motors."""

import math, os, sys, json, atexit
import numpy as np
import cv2

# ── Path ──
_CTRL_DIR = os.path.dirname(os.path.abspath(__file__))
_V1_DIR = os.path.join(_CTRL_DIR, "..", "..", "..", "jetson")
if _V1_DIR not in sys.path:
    sys.path.insert(0, _V1_DIR)

from controller import Supervisor
from line_detector_v1_warp import LineDetector


def clamp(v, lo, hi):
    if v < lo: return lo
    if v > hi: return hi
    return v


# ── Config helper ──
def _load_cfg():
    for p in [
        os.path.join(_CTRL_DIR, "line_follow_params.json"),
        os.path.join(_CTRL_DIR, "..", "..", "..", "line_follow_params.json"),
    ]:
        try:
            with open(p, encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}

CFG = _load_cfg()

def _g(path, default):
    cur = CFG
    for k in path.split("."):
        if not isinstance(cur, dict) or k not in cur:
            return default
        cur = cur[k]
    return cur


# ── PID params (JSON + env override) ──
KP_S = float(os.environ.get("JETSON_PID_STRAIGHT_KP", _g("pid.straight.kp", 0.83)))
KI_S = float(os.environ.get("JETSON_PID_STRAIGHT_KI", _g("pid.straight.ki", 0.004)))
KD_S = float(os.environ.get("JETSON_PID_STRAIGHT_KD", _g("pid.straight.kd", 0.095)))
KP_C = float(os.environ.get("JETSON_PID_CURVE_KP",   _g("pid.curve.kp", 0.98)))
KI_C = float(os.environ.get("JETSON_PID_CURVE_KI",   _g("pid.curve.ki", 0.002)))
KD_C = float(os.environ.get("JETSON_PID_CURVE_KD",   _g("pid.curve.kd", 0.115)))
I_MAX = float(_g("pid.i_clamp", 60.0))

# ── Steer ──
STEER_SAT = float(_g("steer.sat", 45.0))
STEER_SCL = float(_g("steer.scale", 0.6))
RATE_LIM  = float(os.environ.get("JETSON_STEER_RATE_LIMIT", _g("shake_robust.steer_rate_limit_per_frame", 5.0)))

# ── Speed ──
BASE_SPD = float(os.environ.get("LINE_FOLLOW_BASE_SPEED", _g("webots.base_speed", 6.28)))
ST2WHL   = float(_g("webots.steer_to_wheel", 0.0184))
MAX_SPD  = float(_g("webots.max_speed", 6.28))
LOST_SPD = float(_g("webots.speed_lost_scale", 0.92))
MIN_SPD  = float(_g("webots.speed_min", 2.6))

# ── Lost recovery ──
HOLD_FRAMES = int(_g("lost.hold_frames", 6))
SRCH_TURN   = float(_g("lost.search_turn", 11.0))

# ── Misc ──
CAM_NAME = str(_g("webots.camera.device_name", "camera_ext"))
MAX_SEC  = float(os.environ.get("LINE_FOLLOW_MAX_SECONDS", 120))


# ═══════════════════════════════════════════════════════════════════════
# Robot setup
# ═══════════════════════════════════════════════════════════════════════

robot = Supervisor()
TS = int(robot.getBasicTimeStep())
cam = robot.getDevice(CAM_NAME)
cam.enable(TS)
W, H = cam.getWidth(), cam.getHeight()

# Camera: pinhole lens + FOV from calibration (V1 detector needs pinhole input)
cal = _g("camera.calibration", None)
if isinstance(cal, dict):
    try:
        cn = robot.getFromDevice(cam)
        if cn is not None:
            cn.getField("lens").importSFNodeFromString(
                "Lens { center 0.5 0.5 radialCoefficients [ 0 0 ] tangentialCoefficients [ 0 0 ] }")
            fx = float(cal.get("fx", 0.0))
            if fx > 0:
                cn.getField("fieldOfView").setSFFloat(2.0 * math.atan(W / (2.0 * fx)))
    except Exception:
        pass

ld = LineDetector(W, H,
    cam_height_cm=float(_g("camera.height_cm", 38.0)),
    cam_pitch_deg=float(_g("camera.pitch_deg", 45.0)),
    cam_vfov_deg=float(_g("camera.vfov_deg", 43.6)))

L = robot.getDevice("left wheel motor")
R = robot.getDevice("right wheel motor")
for m in (L, R):
    m.setPosition(float("inf"))
    m.setVelocity(0.0)


# ═══════════════════════════════════════════════════════════════════════
# Logging
# ═══════════════════════════════════════════════════════════════════════

_log_fh = None
try:
    _log_dir = os.path.join(_CTRL_DIR, "..", "..", "..", "generated")
    os.makedirs(_log_dir, exist_ok=True)
    _log_fh = open(os.path.join(_log_dir, "jetson_bridge_v1_log.txt"), "w", encoding="utf-8")
except Exception:
    pass

def _log(msg):
    print(msg)
    if _log_fh:
        try:
            _log_fh.write(msg + "\n")
            _log_fh.flush()
        except Exception:
            pass

atexit.register(lambda: _log_fh.close() if _log_fh else None)


# ═══════════════════════════════════════════════════════════════════════
# State
# ═══════════════════════════════════════════════════════════════════════

integral = 0.0
last_err = 0.0
last_steer = 0.0
last_lock_ok = False
last_print_t = -99.0

_log("jetson_bridge_v1 minimal: %dx%d KP_s=%.3f KP_c=%.3f spd=%.2f max_sec=%.0f"
     % (W, H, KP_S, KP_C, BASE_SPD, MAX_SEC))


# ═══════════════════════════════════════════════════════════════════════
# Main loop
# ═══════════════════════════════════════════════════════════════════════

t0 = None
while robot.step(TS) != -1:
    if t0 is None:
        t0 = robot.getTime()
    t = robot.getTime()

    # Grab frame
    raw = cam.getImage()
    if raw is None:
        continue
    bgr = cv2.cvtColor(np.frombuffer(raw, np.uint8).reshape(H, W, 4), cv2.COLOR_BGRA2BGR)

    # Detector — already fused, smoothed, lock-blended
    _dev, _hdg, conf, _vis, dbg = ld.process(bgr)
    err      = float(dbg.get("fused_err", 0.0))      # EMA-smoothed fused error
    curve    = bool(dbg.get("curve_mode", False))     # dual-mode flag
    lost     = int(dbg.get("lost_frames", 0))
    lock_ok  = bool(dbg.get("bottom_lock_valid", False))

    # Reacquisition reset
    if lock_ok and not last_lock_ok:
        integral = 0.0
        last_err = 0.0
    last_lock_ok = lock_ok

    dt = TS / 1000.0

    # ── PID (dual-mode) or lost recovery ──
    if lost == 0:
        kp, ki, kd = (KP_C, KI_C, KD_C) if curve else (KP_S, KI_S, KD_S)
        integral += err * dt
        integral = clamp(integral, -I_MAX, I_MAX)
        derr = (err - last_err) / max(dt, 1e-3)
        last_err = err
        pid_out = kp * err + ki * integral + kd * derr
        steer = STEER_SAT * math.tanh(pid_out / STEER_SCL)
        if conf < 0.25:
            integral *= 0.85
    else:
        if lost <= HOLD_FRAMES:
            steer = last_steer * 0.85
        else:
            phase = (lost // 8) % 2
            steer = abs(SRCH_TURN) if phase == 0 else -abs(SRCH_TURN)
        integral *= 0.5

    # ── Steer rate limit ──
    ds = steer - last_steer
    if abs(ds) > RATE_LIM:
        steer = last_steer + math.copysign(RATE_LIM, ds)
    last_steer = steer

    # ── Speed control (conf + lost + min clamp) ──
    spd = BASE_SPD
    if lost > 0:
        spd *= LOST_SPD
    if conf < 0.5:
        spd *= (0.5 + 0.5 * conf)
    spd = max(spd, MIN_SPD)

    # ── Motors ──
    delta = clamp(steer * ST2WHL, -2.8, 2.8)
    left_spd  = clamp(spd - delta, -MAX_SPD, MAX_SPD)
    right_spd = clamp(spd + delta, -MAX_SPD, MAX_SPD)
    L.setVelocity(left_spd)
    R.setVelocity(right_spd)

    # ── Log ──
    if t - last_print_t > 0.5:
        last_print_t = t
        _log("t=%.1f steer=%+.2f err=%+.3f curve=%d lost=%d conf=%.2f spd=%.2f "
             "L=%.2f R=%.2f"
             % (t, steer, err, 1 if curve else 0, lost, conf, spd, left_spd, right_spd))

    # ── Time limit ──
    if t - t0 > MAX_SEC:
        _log("Done. %.1fs elapsed (MAX_SEC=%.0f)." % (t, MAX_SEC))
        robot.simulationQuit(0)
        break

# Cleanup
if _log_fh:
    _log_fh.close()
os._exit(0)
