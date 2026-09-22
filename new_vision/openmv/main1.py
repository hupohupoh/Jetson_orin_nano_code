import sensor  # type: ignore
import image  # type: ignore
import time
import math
from pyb import UART, LED  # type: ignore

try:
	import ujson as json  # type: ignore
except ImportError:
	import json  # type: ignore

try:
	from protocol_v2 import VisionProtocolV2, quantize_to_step  # type: ignore
except Exception:
	VisionProtocolV2 = None

	def quantize_to_step(v, step):
		if step <= 1:
			return int(v)
		return int(round(float(v) / float(step)) * float(step))

try:
	from protocol_v2 import SafetyMonitor  # type: ignore
except Exception:
	SafetyMonitor = None

try:
	from protocol_v2 import (
		StreamParser, PendingAcks,
		MSG_ACK, MSG_ROBOT_STATE,
		MSG_QR_EVENT, MSG_OBSTACLE_EVENT, MSG_MODE_SWITCH_REQ, MSG_ESTOP,
		parse_ack, parse_line_ctrl, parse_heartbeat,
	)  # type: ignore
except Exception:
	StreamParser = None
	PendingAcks = None
	MSG_ACK = 0x80
	MSG_ROBOT_STATE = 0x81
	MSG_QR_EVENT = 0x10
	MSG_OBSTACLE_EVENT = 0x11
	MSG_MODE_SWITCH_REQ = 0x12
	MSG_ESTOP = 0xE0
	def parse_ack(p): return None
	def parse_line_ctrl(p): return None
	def parse_heartbeat(p): return None


def _cfg_get(cfg, path, default):
	cur = cfg
	parts = path.split('.')
	for p in parts:
		if not isinstance(cur, dict) or (p not in cur):
			return default
		cur = cur[p]
	return cur


def _load_shared_cfg():
	# 兼容本地仓库运行与拷贝到设备后的运行路径。
	candidates = (
		"line_follow_params.json",
		"../line_follow_params.json",
		"../../line_follow_params.json",
	)
	for p in candidates:
		try:
			with open(p, "r") as f:
				return json.load(f)
		except Exception:
			pass
	return {}


SHARED_CFG = _load_shared_cfg()


# ---------------------- 串口配置 ----------------------
UART_ID = int(_cfg_get(SHARED_CFG, "output.uart.id", 1))
UART_BAUD = int(_cfg_get(SHARED_CFG, "output.uart.baud", 115200))
uart = UART(UART_ID, UART_BAUD)

ROUTE_GO = str(_cfg_get(SHARED_CFG, "output.route.go", "1"))
ROUTE_LEFT = str(_cfg_get(SHARED_CFG, "output.route.left", "2"))
ROUTE_RIGHT = str(_cfg_get(SHARED_CFG, "output.route.right", "3"))
ROUTE_SLIGHT_LEFT = str(_cfg_get(SHARED_CFG, "output.route.slight_left", "4"))


# ---------------------- 相机与算法参数 ----------------------
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)  # 160x120，兼顾速度与细节。
sensor.skip_frames(time=1500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

IMG_W = int(_cfg_get(SHARED_CFG, "base_frame.width", 160))
IMG_H = int(_cfg_get(SHARED_CFG, "base_frame.height", 120))
IMG_CX = IMG_W // 2

# 相机几何模型（近似）：默认按45度俯视，建立像素行到地面距离映射。
CAM_PITCH_DEG = float(_cfg_get(SHARED_CFG, "camera.pitch_deg", 45.0))
CAM_HEIGHT_CM = float(_cfg_get(SHARED_CFG, "camera.height_cm", 40.0))
CAM_VFOV_DEG = float(_cfg_get(SHARED_CFG, "camera.vfov_deg", 43.3))
CAM_HFOV_DEG = float(_cfg_get(SHARED_CFG, "camera.hfov_deg", 56.1))
ROW_DIST_MIN_CM = float(_cfg_get(SHARED_CFG, "camera.row_dist_min_cm", 6.0))
ROW_DIST_MAX_CM = float(_cfg_get(SHARED_CFG, "camera.row_dist_max_cm", 300.0))

# 转弯触发距离模型：到达一定距离且偏移足够时提前入弯。
TURN_START_DIST_CM = float(_cfg_get(SHARED_CFG, "turn.start_dist_cm", 42.0))
TURN_END_DIST_CM = float(_cfg_get(SHARED_CFG, "turn.end_dist_cm", 16.0))
TURN_ERR_CM = float(_cfg_get(SHARED_CFG, "turn.err_cm", 3.5))

# 三层ROI：近端抑制抖动，中端主控，远端预判弯道。
_ROIS_DEFAULT = [
	[0, 84, 160, 30, 0.20],
	[0, 54, 160, 28, 0.55],
	[0, 24, 160, 24, 0.25],
]
ROIS = [tuple(r) for r in _cfg_get(SHARED_CFG, "roi.rois_px", _ROIS_DEFAULT)]

SCAN_LINES_PER_ROI = int(_cfg_get(SHARED_CFG, "roi.scan_lines_per_roi", 5))
MIN_TRACK_WIDTH = int(_cfg_get(SHARED_CFG, "roi.min_track_width", 18))
MAX_TRACK_WIDTH = int(_cfg_get(SHARED_CFG, "roi.max_track_width", 145))
MIN_LINE_WIDTH = int(_cfg_get(SHARED_CFG, "roi.min_line_width", 2))
MAX_LINE_WIDTH = int(_cfg_get(SHARED_CFG, "roi.max_line_width", 95))
LANE_WIDTH_TOL_PX = float(_cfg_get(SHARED_CFG, "roi.lane_width_tol_px", 40.0))

# 底部对称锁：与 Webots 控制器同源思路，轻量实现（无红/黑条带屏蔽以省算力）。
BOTTOM_LOCK_ENABLE = bool(_cfg_get(SHARED_CFG, "roi.bottom_lock_enable", True))
BOTTOM_LOCK_START_RATIO = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_start_ratio", 0.80))
BOTTOM_LOCK_ROWS = int(_cfg_get(SHARED_CFG, "roi.bottom_lock_rows", 8))
BOTTOM_LOCK_STEP = int(_cfg_get(SHARED_CFG, "roi.bottom_lock_step", 2))
BOTTOM_LOCK_MIN_PAIR_RATIO = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_min_pair_ratio", 0.55))
BOTTOM_LOCK_SYM_TOL_PX = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_sym_tol_px", 12.0))
BOTTOM_LOCK_BLEND = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_blend", 0.20))
BOTTOM_LOCK_CONF_PENALTY = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_conf_penalty", 0.45))

# 启动期：降低 conf 门槛，减少多 ROI 歧义（与 Webots 一致可调）。
STARTUP_SETTLE_FRAMES = int(_cfg_get(SHARED_CFG, "roi.startup_settle_frames", 60))
STARTUP_CONF_MIN_SCALE = float(_cfg_get(SHARED_CFG, "roi.startup_conf_min_scale", 0.45))
LOCK_REACQUIRE_RESET = bool(_cfg_get(SHARED_CFG, "roi.lock_reacquire_reset", True))

# 动态阈值参数：基于Otsu结果上移，适应光照变化。
TH_OFFSET = int(_cfg_get(SHARED_CFG, "threshold.offset", 8))
TH_MIN = int(_cfg_get(SHARED_CFG, "threshold.min", 25))
TH_MAX = int(_cfg_get(SHARED_CFG, "threshold.max", 120))

# 预处理与稳定性增强。
USE_HISTEQ = bool(_cfg_get(SHARED_CFG, "preprocess.use_histeq", True))
USE_BINARY = bool(_cfg_get(SHARED_CFG, "preprocess.use_binary", False))
USE_ERODE = bool(_cfg_get(SHARED_CFG, "preprocess.use_erode", True))
ERODE_ITER = int(_cfg_get(SHARED_CFG, "preprocess.erode_iter", 1))
ENABLE_DEBUG_DRAW = bool(_cfg_get(SHARED_CFG, "preprocess.enable_debug_draw", True))

MIN_VALID_LINES = int(_cfg_get(SHARED_CFG, "roi.min_valid_lines", 3))
WIDTH_STD_MAX = float(_cfg_get(SHARED_CFG, "roi.width_std_max", 14))
CONF_MIN = float(_cfg_get(SHARED_CFG, "roi.conf_min", 0.25))
SMOOTH_ALPHA = float(_cfg_get(SHARED_CFG, "fusion.smooth_alpha", 0.65))
CURVE_GAIN = float(_cfg_get(SHARED_CFG, "fusion.curve_gain", 0.25))
ANGLE_GAIN = float(_cfg_get(SHARED_CFG, "fusion.angle_gain", 0.22))
MIN_WEIGHT = float(_cfg_get(SHARED_CFG, "fusion.min_weight", 0.10))

# 前瞻增益：让远端信息提前参与控制。
LOOKAHEAD_GAIN = float(_cfg_get(SHARED_CFG, "fusion.lookahead_gain", 0.35))

# 双模式PID参数（直道/弯道）。
CURVE_SWITCH_CM = float(_cfg_get(SHARED_CFG, "pid.curve_switch_cm", 4.5))
KP_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.kp", 0.70))
KI_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.ki", 0.015))
KD_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.kd", 0.12))
KP_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.kp", 1.05))
KI_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.ki", 0.008))
KD_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.kd", 0.18))
I_CLAMP = float(_cfg_get(SHARED_CFG, "pid.i_clamp", 60))

# 非线性映射：避免边缘大误差导致过激转向。
STEER_SAT = float(_cfg_get(SHARED_CFG, "steer.sat", 70))
STEER_SCALE = float(_cfg_get(SHARED_CFG, "steer.scale", 22.0))
DEADBAND = float(_cfg_get(SHARED_CFG, "steer.deadband", 6))

# 丢线恢复。
LOST_HOLD_FRAMES = int(_cfg_get(SHARED_CFG, "lost.hold_frames", 6))
LOST_SEARCH_TURN = float(_cfg_get(SHARED_CFG, "lost.search_turn", 26))

# 输出节流。
SEND_INTERVAL_MS = int(_cfg_get(SHARED_CFG, "output.send_interval_ms", 100))
last_send_ms = 0

# 抗抖参数 (mirror Webots/.../line_follow_transfer.py shake_robust.* 段)。
# 与 Webots 不同的是, 真机走 cm 域控制 (没有 px), 所以 trigger 阈值用 cm.
ROBUST_CFG = _cfg_get(SHARED_CFG, "shake_robust", {}) or {}
ROBUST_ENABLE = bool(ROBUST_CFG.get("enable", True))
ROBUST_DIFF_WINDOW = int(ROBUST_CFG.get("diff_window", 5))
ROBUST_DIFF_RMS_TRIGGER_CM = float(ROBUST_CFG.get("diff_rms_trigger_cm", 1.5))
ROBUST_ALPHA_HIGH = float(ROBUST_CFG.get("alpha_high", 0.93))
ROBUST_KD_SHAKE_SCALE = float(ROBUST_CFG.get("kd_shake_scale", 0.45))
ROBUST_STEER_RATE_LIMIT = float(ROBUST_CFG.get("steer_rate_limit_per_frame", 6.0))
ROBUST_DECAY_FRAMES = int(ROBUST_CFG.get("decay_frames", 12))
ROBUST_BOTTOM_LOCK_BLEND_SCALE = float(ROBUST_CFG.get("bottom_lock_blend_scale", 1.5))

# 协议V2配置（优先使用完整帧；失败时退回单字符兼容发送）。
PROTOCOL_VERSION = int(_cfg_get(SHARED_CFG, "output.protocol.version", 2))
CTRL_HZ = int(_cfg_get(SHARED_CFG, "output.protocol.ctrl_hz", 10))
HEARTBEAT_HZ = int(_cfg_get(SHARED_CFG, "output.protocol.heartbeat_hz", 10))
CTRL_INTERVAL_MS = int(1000 / max(1, CTRL_HZ))
HEARTBEAT_INTERVAL_MS = int(1000 / max(1, HEARTBEAT_HZ))
CONF_STEP = int(_cfg_get(SHARED_CFG, "output.protocol.conf_quant_step", 5))
EX_STEP_MM = int(_cfg_get(SHARED_CFG, "output.protocol.ex_quant_step_mm", 10))
ANG_STEP_CDEG = int(_cfg_get(SHARED_CFG, "output.protocol.ang_quant_step_cdeg", 100))
MODE_IDLE = int(_cfg_get(SHARED_CFG, "output.protocol.mode_idle", 0))
MODE_LINE_FOLLOW = int(_cfg_get(SHARED_CFG, "output.protocol.mode_line_follow", 1))
MODE_LOST_SEARCH = int(_cfg_get(SHARED_CFG, "output.protocol.mode_lost_search", 2))
protocol = None
if VisionProtocolV2 is not None:
	try:
		protocol = VisionProtocolV2(version=PROTOCOL_VERSION)
	except Exception:
		protocol = None
last_ctrl_ms = 0
last_heartbeat_ms = 0


# ---------------------- 运行状态 ----------------------
integral_term = 0.0
last_err = 0.0
last_ms = time.ticks_ms()
last_steer = 0.0
lost_frames = 0
smoothed_err = 0.0

# 抗抖检测状态
last_base_err_cm = 0.0
base_err_history = []  # 最长 ROBUST_DIFF_WINDOW + 1
shake_active_frames = 0
diff_rms_cm = 0.0

frame_idx = 0
last_bottom_lock_valid = False
bottom_pair_ratio = 0.0
bottom_lock_valid = True
center_lock_quality = 1.0

# Safety monitor (仅做本机发送行为自检, 不会阻塞主循环)
safety = None
if SafetyMonitor is not None:
	try:
		safety = SafetyMonitor(
			ctrl_period_ms=CTRL_INTERVAL_MS,
			heartbeat_period_ms=HEARTBEAT_INTERVAL_MS,
			safe_stop_ms=int(_cfg_get(SHARED_CFG, "output.protocol.safety_timeout_ms", 300)),
			lost_recovery_ms=int(_cfg_get(SHARED_CFG, "output.protocol.lost_recovery_ms", 800)),
		)
	except Exception:
		safety = None

# RX stream parser + ACK retry queue
ACK_TIMEOUT_MS = int(_cfg_get(SHARED_CFG, "output.protocol.ack_timeout_ms", 80))
RETRY_MAX = int(_cfg_get(SHARED_CFG, "output.protocol.retry_max", 3))
rx_parser = StreamParser() if StreamParser is not None else None


def _uart_send(data):
	try:
		uart.write(data)
	except Exception:
		pass


pending_acks = PendingAcks(_uart_send, timeout_ms=ACK_TIMEOUT_MS, retry_max=RETRY_MAX) if PendingAcks is not None else None
last_robot_state = None
last_rx_ms = None


def send_event(msg_type, payload=b"", request_ack=True):
	"""Public helper for QR/OBSTACLE/MODE_SWITCH/ESTOP events.
	Goes through PendingAcks so it is auto-retried per PDF section 6.
	"""
	if protocol is None:
		try:
			uart.write(bytes(payload))
		except Exception:
			pass
		return
	frame = protocol.build_event(msg_type, payload, request_ack=request_ack)
	if pending_acks is not None and request_ack:
		# seq is the second-to-last byte of the frame header at index 5
		seq = frame[5]
		pending_acks.send(frame, seq, msg_type)
	else:
		_uart_send(frame)


def _handle_rx_frame(fr):
	global last_robot_state, last_rx_ms
	last_rx_ms = now_ms()
	if fr.msg_type == MSG_ACK:
		decoded = parse_ack(fr.payload)
		if decoded is not None and pending_acks is not None:
			pending_acks.on_ack(decoded["ack_seq"], decoded["ack_msg_type"])
	elif fr.msg_type == MSG_ROBOT_STATE:
		last_robot_state = fr.payload
	# 主控可能透传一些事件回视觉端 (例如 ESTOP), 这里留空 hook

# 行映射查找表：每行对应的前向距离与横向厘米/像素比例。
row_distance_cm = [0.0] * IMG_H
row_cm_per_px = [0.0] * IMG_H


def clamp(v, lo, hi):
	if v < lo:
		return lo
	if v > hi:
		return hi
	return v


def median(vals):
	n = len(vals)
	if n == 0:
		return None
	s = sorted(vals)
	m = n // 2
	if n & 1:
		return s[m]
	return (s[m - 1] + s[m]) / 2


def stdev(vals):
	n = len(vals)
	if n < 2:
		return 0.0
	mean = sum(vals) / n
	var = 0.0
	for v in vals:
		var += (v - mean) * (v - mean)
	return math.sqrt(var / (n - 1))


def line_fit(ys, xs):
	# 拟合 x = a*y + b。
	n = len(xs)
	if n < 2:
		return 0.0, xs[0]
	mean_y = sum(ys) / n
	mean_x = sum(xs) / n
	num = 0.0
	den = 0.0
	for i in range(n):
		dy = ys[i] - mean_y
		num += dy * (xs[i] - mean_x)
		den += dy * dy
	if den == 0:
		return 0.0, mean_x
	a = num / den
	b = mean_x - a * mean_y
	return a, b


def build_camera_lut():
	half_v = CAM_VFOV_DEG * 0.5
	half_h = CAM_HFOV_DEG * 0.5

	y = 0
	while y < IMG_H:
		# 把图像行映射为相对光轴的垂向夹角（上正下负）。
		v_deg = ((IMG_H * 0.5 - y) / (IMG_H * 0.5)) * half_v
		ray_deg = CAM_PITCH_DEG + v_deg
		if ray_deg < 1.0:
			ray_deg = 1.0

		# 地面交点前向距离：z = h / tan(ray_angle)。
		z_cm = CAM_HEIGHT_CM / math.tan(math.radians(ray_deg))
		z_cm = clamp(z_cm, ROW_DIST_MIN_CM, ROW_DIST_MAX_CM)
		row_distance_cm[y] = z_cm

		# 该距离下每个像素对应的横向厘米比例。
		row_cm_per_px[y] = (2.0 * z_cm * math.tan(math.radians(half_h))) / IMG_W
		y += 1


def px_to_ground_cm(x, y):
	if y < 0:
		y = 0
	elif y >= IMG_H:
		y = IMG_H - 1
	x_cm = (x - IMG_CX) * row_cm_per_px[y]
	z_cm = row_distance_cm[y]
	return x_cm, z_cm


def now_ms():
	return time.ticks_ms()


def send_route_cmd(ch):
	uart.write(ch)


def route_to_u8(cmd):
	try:
		return int(cmd)
	except Exception:
		pass
	if cmd == ROUTE_LEFT:
		return 2
	if cmd == ROUTE_RIGHT:
		return 3
	if cmd == ROUTE_SLIGHT_LEFT:
		return 4
	return 1


def adaptive_black_threshold(img):
	# 采用全图Otsu阈值，并做偏移，保留深色赛道线。
	th = img.get_histogram().get_threshold().value()
	return clamp(th + TH_OFFSET, TH_MIN, TH_MAX)


def pixel_is_black(pixel, black_th, binary_mode):
	if binary_mode:
		return pixel == 0
	return pixel <= black_th


def find_lr_edges_on_row(img, y, x0, x1, black_th, binary_mode):
	left = -1
	right = -1

	# 从左到右找左边界。
	x = x0
	while x <= x1:
		if pixel_is_black(img.get_pixel(x, y), black_th, binary_mode):
			left = x
			break
		x += 1

	# 从右到左找右边界。
	x = x1
	while x >= x0:
		if pixel_is_black(img.get_pixel(x, y), black_th, binary_mode):
			right = x
			break
		x -= 1

	if left < 0 or right < 0:
		return None

	width = right - left
	if width < MIN_TRACK_WIDTH or width > MAX_TRACK_WIDTH:
		return None
	return (left, right)


def collect_runs_on_row(img, y, x0, x1, black_th, binary_mode):
	runs = []
	run_start = -1
	x = x0
	while x <= x1:
		is_tr = pixel_is_black(img.get_pixel(x, y), black_th, binary_mode)
		if is_tr and run_start < 0:
			run_start = x
		elif (not is_tr) and run_start >= 0:
			run_end = x - 1
			ww = run_end - run_start + 1
			if ww >= MIN_LINE_WIDTH and ww <= MAX_LINE_WIDTH:
				runs.append((run_start, run_end))
			run_start = -1
		x += 1
	if run_start >= 0:
		run_end = x1
		ww = run_end - run_start + 1
		if ww >= MIN_LINE_WIDTH and ww <= MAX_LINE_WIDTH:
			runs.append((run_start, run_end))
	return runs


def choose_pair_center_from_runs_openmv(runs, hint_center, lane_width_hint, x0, x1):
	if len(runs) < 2:
		return None
	best = None
	best_score = 1e9
	i = 0
	while i < len(runs):
		li = 0.5 * (runs[i][0] + runs[i][1])
		j = i + 1
		while j < len(runs):
			rj = 0.5 * (runs[j][0] + runs[j][1])
			lane_w = rj - li
			if lane_w >= MIN_TRACK_WIDTH and lane_w <= MAX_TRACK_WIDTH:
				if lane_width_hint > 0:
					max_width_err = max(24.0, LANE_WIDTH_TOL_PX * 1.6)
					if abs(lane_w - lane_width_hint) > max_width_err:
						j += 1
						continue
				center = 0.5 * (li + rj)
				if center >= x0 and center <= x1:
					width_err = abs(lane_w - lane_width_hint) if lane_width_hint > 0 else 0.0
					center_err = abs(center - hint_center)
					score = 1.0 * center_err + 0.8 * width_err
					if score < best_score:
						best_score = score
						best = {
							"center_px": center,
							"lane_width_px": lane_w,
						}
			j += 1
		i += 1
	return best


def detect_bottom_center_lock_img(img, black_th, binary_mode):
	if not BOTTOM_LOCK_ENABLE:
		return {
			"valid": True,
			"quality": 1.0,
			"pair_ratio": 0.0,
			"center_px": float(IMG_CX),
			"center_err_px": 0.0,
			"symmetry_abs_px": 0.0,
		}
	x0 = 0
	x1 = IMG_W - 1
	y_start = int(clamp(BOTTOM_LOCK_START_RATIO * IMG_H, 0, IMG_H - 1))
	row_step = max(1, BOTTOM_LOCK_STEP)
	pair_rows = 0
	rows_done = 0
	centers = []
	y = y_start
	while y < IMG_H and rows_done < max(1, BOTTOM_LOCK_ROWS):
		runs = collect_runs_on_row(img, y, x0, x1, black_th, binary_mode)
		chosen = choose_pair_center_from_runs_openmv(runs, float(IMG_CX), 0.0, x0, x1)
		if chosen is not None:
			pair_rows += 1
			centers.append(float(chosen["center_px"]))
		rows_done += 1
		y += row_step
	if rows_done <= 0 or not centers:
		return {
			"valid": False,
			"quality": 0.0,
			"pair_ratio": 0.0,
			"center_px": float(IMG_CX),
			"center_err_px": 0.0,
			"symmetry_abs_px": float(IMG_W),
		}
	pair_ratio = pair_rows / float(rows_done)
	center_px = float(median(centers))
	center_err_px = center_px - float(IMG_CX)
	symmetry_abs_px = abs(center_err_px)
	pair_q = clamp(
		(pair_ratio - BOTTOM_LOCK_MIN_PAIR_RATIO) / max(1.0 - BOTTOM_LOCK_MIN_PAIR_RATIO, 1e-6),
		0.0,
		1.0,
	)
	sym_q = 1.0 - clamp(symmetry_abs_px / max(BOTTOM_LOCK_SYM_TOL_PX * 2.0, 1e-6), 0.0, 1.0)
	quality = clamp(0.65 * pair_q + 0.35 * sym_q, 0.0, 1.0)
	valid = (pair_ratio >= BOTTOM_LOCK_MIN_PAIR_RATIO) and (symmetry_abs_px <= BOTTOM_LOCK_SYM_TOL_PX)
	return {
		"valid": valid,
		"quality": quality,
		"pair_ratio": pair_ratio,
		"center_px": center_px,
		"center_err_px": center_err_px,
		"symmetry_abs_px": symmetry_abs_px,
	}


def roi_midline(img, roi, black_th, binary_mode, draw_color):
	x, y, w, h, weight = roi
	x0 = x
	x1 = x + w - 1

	lefts = []
	rights = []
	centers = []
	centers_cm = []
	widths = []
	ys = []
	zs_cm = []

	step = h // (SCAN_LINES_PER_ROI + 1)
	if step < 1:
		step = 1

	i = 1
	while i <= SCAN_LINES_PER_ROI:
		sy = y + i * step
		if sy >= IMG_H:
			sy = IMG_H - 1

		lr = find_lr_edges_on_row(img, sy, x0, x1, black_th, binary_mode)
		if lr is not None:
			l, r = lr
			lefts.append(l)
			rights.append(r)
			center_px = (l + r) / 2
			centers.append(center_px)
			x_cm, z_cm = px_to_ground_cm(center_px, sy)
			centers_cm.append(x_cm)
			zs_cm.append(z_cm)
			widths.append(r - l)
			ys.append(sy)
			if ENABLE_DEBUG_DRAW:
				img.draw_cross(l, sy, color=draw_color, size=3, thickness=1)
				img.draw_cross(r, sy, color=draw_color, size=3, thickness=1)
		i += 1

	if len(lefts) < MIN_VALID_LINES:
		return None

	l_med = median(lefts)
	r_med = median(rights)
	center = median(centers)
	center_cm = median(centers_cm)
	dist_cm = median(zs_cm)
	width_std = stdev(widths)
	conf = (len(centers) / SCAN_LINES_PER_ROI) * (1.0 - clamp(width_std / WIDTH_STD_MAX, 0.0, 1.0))

	# 画该ROI估计到的中位线点。
	cy = y + h // 2
	if ENABLE_DEBUG_DRAW:
		img.draw_cross(int(center), cy, color=draw_color, size=4, thickness=1)

	# 在地面坐标拟合中心线斜率，用于航向角误差。
	a, b = line_fit(zs_cm, centers_cm)
	angle = math.degrees(math.atan(a))

	return {
		"center": center,
		"center_cm": center_cm,
		"dist_cm": dist_cm,
		"left": l_med,
		"right": r_med,
		"weight": weight,
		"cy": cy,
		"angle": angle,
		"conf": conf,
		"count": len(centers),
		"width_std": width_std,
	}


def nonlinear_map(v):
	# S形映射，抑制大偏差时的过转。
	return STEER_SAT * math.tanh(v / STEER_SCALE)


def pid_step(err, far_err_cm, curve_force=False, kd_scale=1.0):
	global integral_term, last_err, last_ms

	now = now_ms()
	dt_ms = time.ticks_diff(now, last_ms)
	if dt_ms <= 0:
		dt_ms = 1
	dt = dt_ms / 1000.0
	last_ms = now

	if (not curve_force) and (abs(far_err_cm) < CURVE_SWITCH_CM):
		kp = KP_STRAIGHT
		ki = KI_STRAIGHT
		kd = KD_STRAIGHT
	else:
		kp = KP_CURVE
		ki = KI_CURVE
		kd = KD_CURVE

	kd = kd * float(kd_scale)
	integral_term += err * dt
	integral_term = clamp(integral_term, -I_CLAMP, I_CLAMP)

	derr = (err - last_err) / dt
	last_err = err

	return kp * err + ki * integral_term + kd * derr


def steer_to_cmd(steer):
	# steer > 0：赛道中线偏右，机器人应向右纠偏。
	if abs(steer) <= DEADBAND:
		return ROUTE_GO
	if steer > 0:
		return ROUTE_RIGHT
	if steer < -18:
		return ROUTE_LEFT
	return ROUTE_SLIGHT_LEFT


LED(1).on()
LED(2).on()
LED(3).on()
build_camera_lut()

while True:
	clock.tick()
	frame_idx += 1
	startup_active = (STARTUP_SETTLE_FRAMES > 0) and (frame_idx < STARTUP_SETTLE_FRAMES)
	startup_warmup = clamp(frame_idx / float(max(1, STARTUP_SETTLE_FRAMES)), 0.0, 1.0)
	conf_min_use = CONF_MIN * (STARTUP_CONF_MIN_SCALE if startup_active else 1.0)

	img = sensor.snapshot()
	avg_conf = 0.0
	angle_err = 0.0
	base_err_cm = 0.0
	far_dist_cm = 0.0
	turn_gate = 0.0
	curve_mode = 0

	if USE_HISTEQ:
		img = img.histeq()

	black_th = adaptive_black_threshold(img)
	binary_mode = False
	if USE_BINARY:
		img = img.binary([(0, black_th)], invert=False)
		binary_mode = True
		black_th = 0

	if USE_ERODE:
		img.erode(ERODE_ITER)

	draw_color = 1 if binary_mode else 255
	draw_dim = 1 if binary_mode else 80

	# 可视化中心参考线。
	if ENABLE_DEBUG_DRAW:
		img.draw_line(IMG_CX, 0, IMG_CX, IMG_H - 1, color=draw_dim)

	_bottom = detect_bottom_center_lock_img(img, black_th, binary_mode)
	bottom_pair_ratio = float(_bottom.get("pair_ratio", 0.0))
	bottom_sym_err_px = float(_bottom.get("center_err_px", 0.0))
	center_lock_quality = float(_bottom.get("quality", 0.0))
	bottom_lock_valid = bool(_bottom.get("valid", False))
	if BOTTOM_LOCK_ENABLE and LOCK_REACQUIRE_RESET and bottom_lock_valid and (not last_bottom_lock_valid):
		integral_term = 0.0
		last_err = 0.0
		smoothed_err *= 0.35
	last_bottom_lock_valid = bottom_lock_valid

	roi_results = []
	for roi in ROIS:
		x, y, w, h, _ = roi
		if ENABLE_DEBUG_DRAW:
			img.draw_rectangle(x, y, w, h, color=draw_dim)
		res = roi_midline(img, roi, black_th, binary_mode, draw_color)
		if res is not None and res["conf"] >= conf_min_use:
			roi_results.append(res)

	if roi_results:
		lost_frames = 0

		weighted_sum = 0.0
		weight_total = 0.0
		weight_nominal = 0.0
		angle_sum = 0.0
		angle_weight = 0.0

		for r in roi_results:
			err = r["center_cm"]
			w = r["weight"] * r["conf"]
			weighted_sum += err * w
			weight_total += w
			weight_nominal += r["weight"]
			angle_sum += r["angle"] * w
			angle_weight += w

		if weight_total <= MIN_WEIGHT:
			roi_results = []
		else:
			# 使用最上方（最远）有效ROI作为前瞻误差。
			far = roi_results[-1]
			near = roi_results[0]
			far_err_cm = far["center_cm"]
			near_err_cm = near["center_cm"]
			far_dist_cm = far["dist_cm"]

			base_err_cm = weighted_sum / weight_total
			angle_err = angle_sum / angle_weight if angle_weight > 0 else 0.0
			avg_conf = (weight_total / weight_nominal) if weight_nominal > 0 else 0.0
			curve_err = far_err_cm - near_err_cm

			# 距离与偏差联合触发提前入弯。
			dist_norm = clamp((TURN_START_DIST_CM - far_dist_cm) / (TURN_START_DIST_CM - TURN_END_DIST_CM), 0.0, 1.0)
			err_norm = clamp(abs(far_err_cm) / TURN_ERR_CM, 0.0, 1.0)
			turn_gate = dist_norm * err_norm
			lookahead_gain_dyn = LOOKAHEAD_GAIN * (0.70 + 0.90 * turn_gate)
			curve_mode_force = (turn_gate > 0.35) or (abs(curve_err) > CURVE_SWITCH_CM)
			if (not bottom_lock_valid) and (bottom_pair_ratio > 0.0) and (
				abs(bottom_sym_err_px) > BOTTOM_LOCK_SYM_TOL_PX
			):
				curve_mode_force = True
			curve_mode = 1 if curve_mode_force else 0

			if not bottom_lock_valid:
				avg_conf *= 1.0 - BOTTOM_LOCK_CONF_PENALTY * (1.0 - center_lock_quality)
				avg_conf = clamp(avg_conf, 0.0, 1.0)

			# Anti-shake: use prior-frame shake state for consistent gain within this frame.
			shake_active = ROBUST_ENABLE and (shake_active_frames > 0)
			alpha_eff = ROBUST_ALPHA_HIGH if shake_active else SMOOTH_ALPHA
			lock_blend_scale = ROBUST_BOTTOM_LOCK_BLEND_SCALE if shake_active else 1.0
			kd_scale_eff = ROBUST_KD_SHAKE_SCALE if shake_active else 1.0

			if bottom_pair_ratio > 0.0:
				lock_gain = (BOTTOM_LOCK_BLEND * lock_blend_scale) * (
					0.55 + 0.45 * center_lock_quality
				)
				lock_gain = clamp(lock_gain, 0.0, 0.95)
				if startup_active:
					lock_gain = clamp(lock_gain + 0.20 * (1.0 - startup_warmup), 0.0, 0.92)
				near = roi_results[0]
				near_err_px = float(near["center"]) - float(IMG_CX)
				near_err_px = (1.0 - lock_gain) * near_err_px + lock_gain * bottom_sym_err_px
				near_cm_blended = near_err_px * row_cm_per_px[IMG_H - 1]
				base_err_cm = base_err_cm - float(near["center_cm"]) + near_cm_blended
				curve_err = far_err_cm - near_cm_blended

			fused_err = base_err_cm
			fused_err += lookahead_gain_dyn * far_err_cm
			fused_err += CURVE_GAIN * curve_err
			fused_err += ANGLE_GAIN * angle_err

			smoothed_err = (alpha_eff * smoothed_err) + ((1.0 - alpha_eff) * fused_err)
			pid_out = pid_step(smoothed_err, far_err_cm, curve_mode_force, kd_scale=kd_scale_eff)
			steer = nonlinear_map(pid_out)

			# Update shake detector with the latest base_err_cm.
			base_err_history.append(float(base_err_cm))
			if len(base_err_history) > ROBUST_DIFF_WINDOW + 1:
				del base_err_history[0]
			if len(base_err_history) >= 3:
				_diffs_sq = 0.0
				_n = 0
				for _i in range(1, len(base_err_history)):
					_d = base_err_history[_i] - base_err_history[_i - 1]
					_diffs_sq += _d * _d
					_n += 1
				diff_rms_cm = math.sqrt(_diffs_sq / max(1, _n))
				if diff_rms_cm >= ROBUST_DIFF_RMS_TRIGGER_CM:
					shake_active_frames = ROBUST_DECAY_FRAMES
				elif shake_active_frames > 0:
					shake_active_frames -= 1
			last_base_err_cm = base_err_cm

	if not roi_results:
		# 丢线时短时保持历史转向，长时进入搜索。
		lost_frames += 1
		if lost_frames <= LOST_HOLD_FRAMES:
			steer = last_steer * 0.85
		else:
			if last_steer >= 0:
				steer = LOST_SEARCH_TURN
			else:
				steer = -LOST_SEARCH_TURN

	# Anti-shake: per-frame steer rate limit (applied to all branches).
	if ROBUST_ENABLE and ROBUST_STEER_RATE_LIMIT > 0.0:
		_d_st = steer - last_steer
		if abs(_d_st) > ROBUST_STEER_RATE_LIMIT:
			if _d_st > 0:
				steer = last_steer + ROBUST_STEER_RATE_LIMIT
			else:
				steer = last_steer - ROBUST_STEER_RATE_LIMIT
	last_steer = steer

	cmd = steer_to_cmd(steer)
	route_u8 = route_to_u8(cmd)
	mode_u8 = MODE_LOST_SEARCH if lost_frames > 0 else MODE_LINE_FOLLOW
	conf_u8 = int(clamp(avg_conf * 100.0, 0.0, 100.0))
	conf_u8 = int(clamp(quantize_to_step(conf_u8, CONF_STEP), 0, 100))
	lost_u8 = 1 if lost_frames > 0 else 0
	ex_mm = int(quantize_to_step(int(base_err_cm * 10.0), EX_STEP_MM))
	ang_cdeg = int(quantize_to_step(int(angle_err * 100.0), ANG_STEP_CDEG))

	# 发送：优先协议V2；若不可用则回退到单字符路由。
	n = now_ms()
	if protocol is not None:
		if time.ticks_diff(n, last_heartbeat_ms) >= HEARTBEAT_INTERVAL_MS:
			try:
				uart.write(protocol.build_heartbeat(mode_u8, ts_ms=n))
				last_heartbeat_ms = n
				if safety is not None:
					safety.mark_sent_hb(n)
			except Exception:
				pass
		if time.ticks_diff(n, last_ctrl_ms) >= CTRL_INTERVAL_MS:
			try:
				frame = protocol.build_line_ctrl(
					mode_u8=mode_u8,
					conf_u8=conf_u8,
					lost_u8=lost_u8,
					route_u8=route_u8,
					ex_mm_i16=ex_mm,
					ang_cdeg_i16=ang_cdeg,
					v_cmd_mmps_i16=0,
					w_cmd_mradps_i16=0,
					ts_ms=n,
				)
				uart.write(frame)
				last_ctrl_ms = n
				if safety is not None:
					safety.mark_sent_ctrl(n)
			except Exception:
				pass
	else:
		if time.ticks_diff(n, last_send_ms) >= SEND_INTERVAL_MS:
			send_route_cmd(cmd)
			last_send_ms = n

	# RX: pull any bytes from main board, decode ACK / ROBOT_STATE.
	if rx_parser is not None:
		try:
			n_avail = uart.any() if hasattr(uart, "any") else 0
			if n_avail > 0:
				chunk = uart.read(n_avail)
				if chunk:
					for fr in rx_parser.feed(chunk):
						_handle_rx_frame(fr)
		except Exception:
			pass

	# Periodic retry of unacked event frames.
	if pending_acks is not None:
		pending_acks.tick()

	print(
		"th=%d steer=%.1f ex=%.1fcm ang=%.1fdeg z=%.1fcm tg=%.2f mode=%d conf=%.2f route=%d lost=%d drms=%.2f sk=%d fps=%.1f"
		% (black_th, steer, base_err_cm, angle_err, far_dist_cm, turn_gate, curve_mode, avg_conf, route_u8, lost_frames, diff_rms_cm, shake_active_frames, clock.fps())
	)
