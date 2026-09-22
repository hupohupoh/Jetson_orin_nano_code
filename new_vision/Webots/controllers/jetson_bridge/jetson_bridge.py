"""Webots 控制器 — 使用 jetson 模块（仿真+真机同一套代码）

与 OpenMV 的 line_follow_transfer 并列存在，不影响旧方案。
"""
from __future__ import annotations
import json, math, os, sys, time, atexit, random
import numpy as np

# ── 路径设置：让 Webots 找到 jetson 模块 ──
_CTRL_DIR = os.path.dirname(os.path.abspath(__file__))
_JETSON_DIR = os.path.abspath(os.path.join(_CTRL_DIR, "..", "..", "..", "jetson"))
if _JETSON_DIR not in sys.path:
    sys.path.insert(0, _JETSON_DIR)

from controller import Robot, Supervisor  # type: ignore
import cv2

from qr_detector import QRDetector
from line_detector import LineDetector


# ── 配置加载（与旧方案同一份 line_follow_params.json） ──
def _cfg_get(cfg, path, default):
    cur = cfg
    for key in path.split("."):
        if not isinstance(cur, dict) or key not in cur:
            return default
        cur = cur[key]
    return cur

def _load_cfg():
    for p in [
        os.path.join(_CTRL_DIR, "line_follow_params.json"),
        os.path.abspath(os.path.join(_CTRL_DIR, "..", "..", "..", "line_follow_params.json")),
        os.path.abspath("line_follow_params.json"),
    ]:
        try:
            with open(p, "r", encoding="utf-8") as f:
                return json.load(f)
        except Exception:
            pass
    return {}

CFG = _load_cfg()

CAM_PITCH = float(_cfg_get(CFG, "camera.pitch_deg", 45.0))
CAM_HEIGHT = float(_cfg_get(CFG, "camera.height_cm", 40.0))
TRACK_W_CM = 35.5  # 黑线宽3.5cm + 内距32cm → 中线距35.5cm

# PID（参数与旧算法对齐，deviation_px 归一化到 [-1,1] 后输入 PID）
# 参数：环境变量覆盖 > JSON 配置 > 默认值
def _env_or_cfg(cfg, key, default):
    env_key = "JETSON_" + key.upper().replace(".", "_")
    if env_key in os.environ:
        return float(os.environ[env_key])
    return float(_cfg_get(cfg, key, default))

KP = _env_or_cfg(CFG, "pid.straight.kp", 0.35)
KI = _env_or_cfg(CFG, "pid.straight.ki", 0.012)
KD = _env_or_cfg(CFG, "pid.straight.kd", 0.08)
I_CLAMP = float(_cfg_get(CFG, "pid.i_clamp", 60.0))
KP_C = _env_or_cfg(CFG, "pid.curve.kp", 0.55)
KI_C = _env_or_cfg(CFG, "pid.curve.ki", 0.006)
KD_C = _env_or_cfg(CFG, "pid.curve.kd", 0.10)
STEER_SAT = _env_or_cfg(CFG, "steer.sat", 45.0)
STEER_SCALE = _env_or_cfg(CFG, "steer.scale", 1.0)

# ── 抗抖动参数（环境变量覆盖）──
SHAKE_KD_SCALE = float(os.environ.get("SHAKE_KD_SCALE", 0.6))
SHAKE_STEER_RATE_LIMIT = float(os.environ.get("SHAKE_STEER_RATE_LIMIT", 14.0))

BASE_SPEED = float(_cfg_get(CFG, "webots.base_speed", 3.2))
STEER_W = float(_cfg_get(CFG, "webots.steer_to_wheel", 0.04))
MAX_SPEED = float(_cfg_get(CFG, "webots.max_speed", 6.28))

# ── 初始化（用 Supervisor 才能 shake + simulationQuit）──
robot = Supervisor()
TIMESTEP = int(robot.getBasicTimeStep())
MAX_SEC = float(os.environ.get("LINE_FOLLOW_MAX_SECONDS", 45))

camera = robot.getDevice("camera_ext")
camera.enable(TIMESTEP)
W, H = camera.getWidth(), camera.getHeight()

# ── 相机抖动（仿真双足行走晃动）──
CAM_SHAKE_PITCH = 0.025   # 俯仰抖动幅度 (rad)，~1.4°
CAM_SHAKE_YAW = 0.015     # 偏航抖动幅度 (rad)，~0.85°
CAM_BASE_PITCH = 0.7853981633974483  # 基础俯角 45°
cam_node = robot.getFromDef("CAM_POSE")
cam_field = cam_node.getField("rotation") if cam_node else None

left = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
left.setPosition(float("inf"))
right.setPosition(float("inf"))
left.setVelocity(0.0)
right.setVelocity(0.0)

# ── jetson 检测器 ──
qr = QRDetector(stable_frames=1, cooldown_ms=2000, min_edge_px=20, max_edge_px=300, debug=True)
ld = LineDetector(cam_height_cm=CAM_HEIGHT, cam_pitch_deg=CAM_PITCH, cam_w=W, cam_h=H,
                  track_width_cm=TRACK_W_CM,
                  inner_radius_cm=59.75, outer_radius_cm=95.25)

# ── PID 状态 ──
pid = {"integral": 0.0, "last_err": 0.0, "last_steer": 0.0,
       "lost_frames": 0, "smoothed_err": 0.0}
_start_t = None

# ── 日志 ──
_log_fh = None
try:
    _log_fh = open(os.path.join(_CTRL_DIR, "..", "..", "..", "generated", "jetson_bridge_log.txt"), "w")
except Exception:
    pass

def _log(msg):
    print(msg)
    if _log_fh:
        _log_fh.write(msg + "\n")
        _log_fh.flush()

_log(f"V4 [jetson_bridge] camera={W}x{H} pitch={CAM_PITCH}deg height={CAM_HEIGHT}cm")
_log(f"  PID kp={KP} ki={KI} kd={KD}  line_det: geometric warp 160x200  W={TRACK_W_CM}cm")

while robot.step(TIMESTEP) != -1:
    if _start_t is None:
        _start_t = robot.getTime()

    # ── 相机抖动 ──
    if cam_field is not None:
        pitch_jitter = random.uniform(-CAM_SHAKE_PITCH, CAM_SHAKE_PITCH)
        yaw_jitter = random.uniform(-CAM_SHAKE_YAW, CAM_SHAKE_YAW)
        # rotation field: [x, y, z, angle]
        cam_field.setSFRotation([0, 1, 0, CAM_BASE_PITCH + pitch_jitter])

    raw = camera.getImage()
    if raw is None:
        continue

    # BGRA -> BGR（和真机 USB 摄像头一样）
    buf = np.frombuffer(raw, dtype=np.uint8).reshape(H, W, 4)
    bgr = cv2.cvtColor(buf, cv2.COLOR_BGRA2BGR)

    # ── 巡线 ──
    dev_px, heading_deg, conf, vis, dbg = ld.process(bgr)

    # 调试：保存原始鸟瞰图到控制器目录
    t = robot.getTime()
    if not hasattr(ld, '_snap_cnt'): ld._snap_cnt = 0
    if t > 3.0 and ld._snap_cnt < 3:
        path = os.path.join(_CTRL_DIR, f"bird_raw_{ld._snap_cnt}.png")
        ok = cv2.imwrite(path, dbg["bird"])
        _log(f"  [debug] raw saved {ok} -> {path}")
        ld._snap_cnt += 1

    # 晃动检测已移到 line_detector.py 内部，dbg["shake_active"] 提供
    steer = 0.0
    fused_err = 0.0
    curve = False
    LOST_HOLD = 6                # 丢线保持帧数（参考老代码）
    LOST_SEARCH = 26.0          # 搜索转向力（参考老代码）

    if dev_px is not None and conf > 0.08:
        pid["lost_frames"] = 0
        dt = TIMESTEP / 1000.0

        # ── 误差归一化（老代码：near_norm = near_err_px / (0.5*img_w)）──
        near_norm = dev_px / 80.0
        heading_norm = heading_deg / 45.0
        fused_err = -near_norm                     # 主项
        fused_err += 0.06 * (-heading_norm)        # 朝向前馈（老代码角度增益）

        # ── EMA 强平滑（检测器已做 shake 检测 + 平滑，控制器只做最后 fusion）──
        SMOOTH_ALPHA = 0.65 if conf > 0.4 else 0.85
        pid["smoothed_err"] = SMOOTH_ALPHA * pid["smoothed_err"] + (1.0 - SMOOTH_ALPHA) * fused_err

        # ── 抗积分饱和 ──
        if abs(pid["last_steer"]) < STEER_SAT * 0.8:
            pid["integral"] += pid["smoothed_err"] * dt
        pid["integral"] = max(-I_CLAMP, min(I_CLAMP, pid["integral"]))

        # ── 低置信时清积分防 windup ──
        if conf < 0.25:
            pid["integral"] *= 0.85

        derr = (pid["smoothed_err"] - pid["last_err"]) / max(dt, 1e-3)
        pid["last_err"] = pid["smoothed_err"]

        kd_eff = KD * SHAKE_KD_SCALE if dbg.get("shake_active", 0) > 0 else KD
        pid_out = KP * pid["smoothed_err"] + KI * pid["integral"] + kd_eff * derr
        steer = STEER_SAT * math.tanh(pid_out / STEER_SCALE)

        # ── 转向变化率限制（老代码：clamp 14/帧）──
        max_ds = SHAKE_STEER_RATE_LIMIT * dt * 30  # 约 14/帧 ≈ 420/s
        ds = steer - pid["last_steer"]
        if abs(ds) > max_ds:
            steer = pid["last_steer"] + max_ds * (1 if ds > 0 else -1)
    else:
        # ── 丢线处理 ──
        pid["lost_frames"] = pid.get("lost_frames", 0) + 1
        lost_n = pid["lost_frames"]
        if lost_n <= LOST_HOLD:
            steer = pid["last_steer"] * 0.85
        else:
            phase = (lost_n // 8) % 2
            steer = LOST_SEARCH if phase == 0 else -LOST_SEARCH
            pid["integral"] *= 0.5
        pid["smoothed_err"] *= 0.9
    pid["last_steer"] = steer

    # ── 低置信降速（老代码有 speed_lost_scale=0.75）──
    speed_scale = 1.0 if conf > 0.3 else (0.75 if conf > 0.1 else 0.6)
    spd = BASE_SPEED * speed_scale

    # 速度 + 差速（spd 已在上方根据 conf 调整）
    delta = max(-2.8, min(2.8, steer * STEER_W))
    left.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, spd - delta)))
    right.setVelocity(max(-MAX_SPEED, min(MAX_SPEED, spd + delta)))

    # ── QR 检测 ──
    action, qdbg = qr.update(bgr)

    # ── 红条检测 ──
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 80, 80), (10, 255, 255)) | cv2.inRange(hsv, (160, 80, 80), (180, 255, 255))
    red_r = cv2.countNonZero(mask) / (W * H)

    # ── 日志（4Hz） ──
    t = robot.getTime()
    if int(t * 4) != int((t - TIMESTEP / 1000.0) * 4):
        _log(f"t={t:.1f}s steer={steer:.1f} dev={dev_px}px head={heading_deg}deg conf={conf:.2f} err={fused_err:.2f} shake={dbg.get('shake_active',0)} rms={dbg.get('shake_rms',0):.1f}")

    # 超时退出
    if t - (_start_t or 0) > MAX_SEC:
        _log(f"Done. {t:.1f}s elapsed.")
        robot.simulationQuit()
        break

if _log_fh:
    _log_fh.close()
os._exit(0)
