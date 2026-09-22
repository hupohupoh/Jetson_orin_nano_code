# main_webots_aligned.py
# 与 Webots/controllers/line_follow_transfer/line_follow_transfer.py 对齐：
# 三路带扫描、简单底区、底部锁、像素域融合、PID、shake_robust、协议输出。
# 集成赛事二维码 1~6：ROI/放大图解码 + lens_corr/无校正重试 + 防抖/冷却，发 protocol MSG_QR_EVENT（见 openmv_webots_aligned.*）。
# main1.py 保留为轻量三 ROI 方案；本脚本用于「与仿真同款流水线」对照 / 真机验证。
#
# 说明：OpenMV 默认灰度图时无法做红色识别，红块禁行视为关闭（仅保留跨线黑块启发）。

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
		StreamParser,
		PendingAcks,
		MSG_ACK,
		MSG_ROBOT_STATE,
		parse_ack,
	)
except Exception:
	StreamParser = None
	PendingAcks = None
	MSG_ACK = 0x80
	MSG_ROBOT_STATE = 0x81

	def parse_ack(p):
		return None


def _cfg_get(cfg, path, default):
	cur = cfg
	for key in path.split("."):
		if not isinstance(cur, dict) or key not in cur:
			return default
		cur = cur[key]
	return cur


def _load_shared_cfg():
	for p in (
		"line_follow_params.json",
		"../line_follow_params.json",
		"../../line_follow_params.json",
	):
		try:
			with open(p, "r") as f:
				return json.load(f)
		except Exception:
			pass
	return {}


SHARED_CFG = _load_shared_cfg()
OMV_WA = _cfg_get(SHARED_CFG, "openmv_webots_aligned", {}) or {}
BAND_ROWS_FACTOR = float(OMV_WA.get("band_rows_factor", 0.78))
USE_HISTEQ = bool(OMV_WA.get("use_histeq", False))
RED_ON_GRAY = bool(OMV_WA.get("red_detect_on_grayscale", False))

# 赛事二维码（规则：数字 1~6；5cm 码）。与巡线并行：ROI/放大解码图 + lens_corr + 可选无校正重试 + 多帧确认 + 冷却 + MSG_QR_EVENT。
QR_ENABLE = bool(OMV_WA.get("qr_enable", True))
QR_EVERY_N_FRAMES = max(1, int(OMV_WA.get("qr_every_n_frames", 3)))
QR_LENS_CORR = float(OMV_WA.get("qr_lens_corr_strength", 1.35))
QR_STABLE_FRAMES = max(1, int(OMV_WA.get("qr_stable_frames", 3)))
QR_COOLDOWN_MS = max(400, int(OMV_WA.get("qr_send_cooldown_ms", 3200)))
QR_REQUEST_ACK = bool(OMV_WA.get("qr_request_ack", True))
QR_ONLY_WHEN_TRACKING = bool(OMV_WA.get("qr_only_when_tracking", True))
QR_MIN_CONF = float(OMV_WA.get("qr_min_line_conf", 0.22))
QR_RETRY_NO_LENS = bool(OMV_WA.get("qr_retry_without_lens_corr", True))
QR_MIN_PIXELS = int(OMV_WA.get("qr_min_pixels", 12))
QR_MAX_PIXELS = int(OMV_WA.get("qr_max_pixels", 300))
QR_MIN_AREA = int(OMV_WA.get("qr_min_area", 60))
QR_DEBUG_LOG = bool(OMV_WA.get("qr_debug_log", True))
QR_ACTION_MAP = {"1": 1, "2": 2, "3": 3, "4": 4, "5": 5, "6": 6}


def _scale_i(n, factor):
	return max(3, int(round(float(n) * factor)))


# ---------------------- 串口 / 协议 ----------------------
UART_ID = int(_cfg_get(SHARED_CFG, "output.uart.id", 1))
UART_BAUD = int(_cfg_get(SHARED_CFG, "output.uart.baud", 115200))
uart = UART(UART_ID, UART_BAUD)

ROUTE_GO = str(_cfg_get(SHARED_CFG, "output.route.go", "1"))
ROUTE_LEFT = str(_cfg_get(SHARED_CFG, "output.route.left", "2"))
ROUTE_RIGHT = str(_cfg_get(SHARED_CFG, "output.route.right", "3"))
ROUTE_SLIGHT_LEFT = str(_cfg_get(SHARED_CFG, "output.route.slight_left", "4"))

PROTOCOL_VERSION = int(_cfg_get(SHARED_CFG, "output.protocol.version", 2))
CTRL_HZ = int(_cfg_get(SHARED_CFG, "output.protocol.ctrl_hz", 10))
HEARTBEAT_HZ = int(_cfg_get(SHARED_CFG, "output.protocol.heartbeat_hz", 10))
CTRL_INTERVAL_MS = int(1000 / max(1, CTRL_HZ))
HEARTBEAT_INTERVAL_MS = int(1000 / max(1, HEARTBEAT_HZ))
CONF_STEP = int(_cfg_get(SHARED_CFG, "output.protocol.conf_quant_step", 5))
EX_STEP_MM = int(_cfg_get(SHARED_CFG, "output.protocol.ex_quant_step_mm", 10))
ANG_STEP_CDEG = int(_cfg_get(SHARED_CFG, "output.protocol.ang_quant_step_cdeg", 100))
MODE_LINE_FOLLOW = int(_cfg_get(SHARED_CFG, "output.protocol.mode_line_follow", 1))
MODE_LOST_SEARCH = int(_cfg_get(SHARED_CFG, "output.protocol.mode_lost_search", 2))
ACK_TIMEOUT_MS = int(_cfg_get(SHARED_CFG, "output.protocol.ack_timeout_ms", 80))
RETRY_MAX = int(_cfg_get(SHARED_CFG, "output.protocol.retry_max", 3))
SEND_INTERVAL_MS = int(_cfg_get(SHARED_CFG, "output.send_interval_ms", 100))

protocol = None
if VisionProtocolV2 is not None:
	try:
		protocol = VisionProtocolV2(version=PROTOCOL_VERSION)
	except Exception:
		protocol = None
last_ctrl_ms = 0
last_heartbeat_ms = 0
last_send_ms = 0

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

rx_parser = StreamParser() if StreamParser is not None else None


def _uart_send(data):
	try:
		uart.write(data)
	except Exception:
		pass


pending_acks = PendingAcks(_uart_send, timeout_ms=ACK_TIMEOUT_MS, retry_max=RETRY_MAX) if PendingAcks is not None else None
last_robot_state = None
last_rx_ms = None


def now_ms():
	return time.ticks_ms()


def _handle_rx_frame(fr):
	global last_robot_state, last_rx_ms
	last_rx_ms = now_ms()
	if fr.msg_type == MSG_ACK:
		decoded = parse_ack(fr.payload)
		if decoded is not None and pending_acks is not None:
			pending_acks.on_ack(decoded["ack_seq"], decoded["ack_msg_type"])
	elif fr.msg_type == MSG_ROBOT_STATE:
		last_robot_state = fr.payload


# ---------------------- 相机 ----------------------
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=1500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

clock = time.clock()

IMG_W = int(_cfg_get(SHARED_CFG, "base_frame.width", 160))
IMG_H = int(_cfg_get(SHARED_CFG, "base_frame.height", 120))
IMG_CX = IMG_W // 2

CAM_PITCH_DEG = float(_cfg_get(SHARED_CFG, "camera.pitch_deg", 30.0))
CAM_HEIGHT_CM = float(_cfg_get(SHARED_CFG, "camera.height_cm", 17.0))
CAM_VFOV_DEG = float(_cfg_get(SHARED_CFG, "camera.vfov_deg", 52.0))
CAM_HFOV_DEG = float(_cfg_get(SHARED_CFG, "camera.hfov_deg", 70.0))
ROW_DIST_MIN_CM = float(_cfg_get(SHARED_CFG, "camera.row_dist_min_cm", 6.0))
ROW_DIST_MAX_CM = float(_cfg_get(SHARED_CFG, "camera.row_dist_max_cm", 300.0))

_ROIS_DEFAULT = [
	[0, 84, 160, 30, 0.20],
	[0, 54, 160, 28, 0.55],
	[0, 24, 160, 24, 0.25],
]
_ROIS_PX = _cfg_get(SHARED_CFG, "roi.rois_px", _ROIS_DEFAULT)
ROI_RATIOS = []
for r in _ROIS_PX:
	if not isinstance(r, (list, tuple)) or len(r) < 5:
		continue
	x, y, w, h, wt = r[0], r[1], r[2], r[3], r[4]
	ROI_RATIOS.append((
			float(x) / float(IMG_W),
			float(y) / float(IMG_H),
			float(w) / float(IMG_W),
			float(h) / float(IMG_H),
			float(wt),
		))
if not ROI_RATIOS:
	ROI_RATIOS = [
		(0.0, 0.70, 1.0, 0.25, 0.20),
		(0.0, 0.45, 1.0, 0.23, 0.55),
		(0.0, 0.20, 1.0, 0.20, 0.25),
	]

SCAN_LINES_PER_ROI = int(_cfg_get(SHARED_CFG, "roi.scan_lines_per_roi", 5))
MIN_TRACK_WIDTH = int(_cfg_get(SHARED_CFG, "roi.min_track_width", 12))
MAX_TRACK_WIDTH = int(_cfg_get(SHARED_CFG, "roi.max_track_width", 220))
MIN_LINE_WIDTH = int(_cfg_get(SHARED_CFG, "roi.min_line_width", 2))
MAX_LINE_WIDTH = int(_cfg_get(SHARED_CFG, "roi.max_line_width", 80))
LANE_WIDTH_INIT_PX = float(_cfg_get(SHARED_CFG, "roi.lane_width_init_px", 70.0))
LANE_WIDTH_TOL_PX = float(_cfg_get(SHARED_CFG, "roi.lane_width_tol_px", 40.0))
MAX_CENTER_JUMP_PX = float(_cfg_get(SHARED_CFG, "roi.max_center_jump_px", 55.0))
SIMPLE_BOTTOM_MODE = bool(_cfg_get(SHARED_CFG, "roi.simple_bottom_mode", True))
BOTTOM_START_RATIO = float(_cfg_get(SHARED_CFG, "roi.bottom_start_ratio", 0.75))
BOTTOM_ROWS = _scale_i(int(_cfg_get(SHARED_CFG, "roi.bottom_rows", 14)), BAND_ROWS_FACTOR)
BOTTOM_STEP = int(_cfg_get(SHARED_CFG, "roi.bottom_step", 2))
SINGLE_LINE_CONF = float(_cfg_get(SHARED_CFG, "roi.single_line_conf", 0.35))
ASSIST_ENABLE = bool(_cfg_get(SHARED_CFG, "roi.assist_enable", True))
ASSIST_START_RATIO = float(_cfg_get(SHARED_CFG, "roi.assist_start_ratio", 0.50))
ASSIST_END_RATIO = float(_cfg_get(SHARED_CFG, "roi.assist_end_ratio", 0.75))
ASSIST_ROWS = _scale_i(int(_cfg_get(SHARED_CFG, "roi.assist_rows", 12)), BAND_ROWS_FACTOR)
ASSIST_STEP = int(_cfg_get(SHARED_CFG, "roi.assist_step", 2))
THREE_BAND_MODE = bool(_cfg_get(SHARED_CFG, "roi.three_band_mode", True))
BAND_DOWN_START_RATIO = float(_cfg_get(SHARED_CFG, "roi.band_down_start_ratio", 2.0 / 3.0))
BAND_MID_START_RATIO = float(_cfg_get(SHARED_CFG, "roi.band_mid_start_ratio", 1.0 / 3.0))
BAND_UP_START_RATIO = float(_cfg_get(SHARED_CFG, "roi.band_up_start_ratio", 0.0))
BAND_ROWS_DOWN = _scale_i(int(_cfg_get(SHARED_CFG, "roi.band_rows_down", 16)), BAND_ROWS_FACTOR)
BAND_ROWS_MID = _scale_i(int(_cfg_get(SHARED_CFG, "roi.band_rows_mid", 14)), BAND_ROWS_FACTOR)
BAND_ROWS_UP = _scale_i(int(_cfg_get(SHARED_CFG, "roi.band_rows_up", 12)), BAND_ROWS_FACTOR)
BAND_STEP_DOWN = int(_cfg_get(SHARED_CFG, "roi.band_step_down", 2))
BAND_STEP_MID = int(_cfg_get(SHARED_CFG, "roi.band_step_mid", 2))
BAND_STEP_UP = int(_cfg_get(SHARED_CFG, "roi.band_step_up", 2))
BAND_WEIGHT_DOWN = float(_cfg_get(SHARED_CFG, "roi.band_weight_down", 0.58))
BAND_WEIGHT_MID = float(_cfg_get(SHARED_CFG, "roi.band_weight_mid", 0.30))
BAND_WEIGHT_UP = float(_cfg_get(SHARED_CFG, "roi.band_weight_up", 0.12))
CROSS_BLACK_RUN_RATIO = float(_cfg_get(SHARED_CFG, "roi.cross_black_run_ratio", 0.42))
CROSS_BLACK_COVER_RATIO = float(_cfg_get(SHARED_CFG, "roi.cross_black_cover_ratio", 0.56))
RED_DETECT_ENABLE = bool(_cfg_get(SHARED_CFG, "roi.red_detect_enable", True)) and RED_ON_GRAY
RED_MIN_R = int(_cfg_get(SHARED_CFG, "roi.red_min_r", 105))
RED_DOM_MARGIN = int(_cfg_get(SHARED_CFG, "roi.red_dom_margin", 28))
RED_ROW_RATIO = float(_cfg_get(SHARED_CFG, "roi.red_row_ratio", 0.35))

BOTTOM_LOCK_ENABLE = bool(_cfg_get(SHARED_CFG, "roi.bottom_lock_enable", True))
BOTTOM_LOCK_START_RATIO = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_start_ratio", 0.80))
BOTTOM_LOCK_ROWS = _scale_i(int(_cfg_get(SHARED_CFG, "roi.bottom_lock_rows", 12)), BAND_ROWS_FACTOR)
BOTTOM_LOCK_STEP = int(_cfg_get(SHARED_CFG, "roi.bottom_lock_step", 2))
BOTTOM_LOCK_MIN_PAIR_RATIO = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_min_pair_ratio", 0.55))
BOTTOM_LOCK_SYM_TOL_PX = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_sym_tol_px", 12.0))
BOTTOM_LOCK_BLEND = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_blend", 0.55))
BOTTOM_LOCK_CONF_PENALTY = float(_cfg_get(SHARED_CFG, "roi.bottom_lock_conf_penalty", 0.45))

STARTUP_SETTLE_FRAMES = int(_cfg_get(SHARED_CFG, "roi.startup_settle_frames", 25))
STARTUP_CONF_MIN_SCALE = float(_cfg_get(SHARED_CFG, "roi.startup_conf_min_scale", 0.70))
STARTUP_MIN_WEIGHT_SCALE = float(_cfg_get(SHARED_CFG, "roi.startup_min_weight_scale", 0.70))
STARTUP_FORCE_SIMPLE_BOTTOM = bool(_cfg_get(SHARED_CFG, "roi.startup_force_simple_bottom", True))
STARTUP_LOST_BIAS_FREE = bool(_cfg_get(SHARED_CFG, "roi.startup_lost_bias_free", True))
LOCK_REACQUIRE_RESET = bool(_cfg_get(SHARED_CFG, "roi.lock_reacquire_reset", True))

MIN_VALID_LINES = int(_cfg_get(SHARED_CFG, "roi.min_valid_lines", 2))
WIDTH_STD_MAX = float(_cfg_get(SHARED_CFG, "roi.width_std_max", 14))
CONF_MIN = float(_cfg_get(SHARED_CFG, "roi.conf_min", 0.12))
MIN_PAIR_RATIO = float(_cfg_get(SHARED_CFG, "roi.min_pair_ratio", 0.4))
MIN_WEIGHT = float(_cfg_get(SHARED_CFG, "fusion.min_weight", 0.10))
SMOOTH_ALPHA = float(_cfg_get(SHARED_CFG, "fusion.smooth_alpha", 0.65))

TH_OFFSET = int(_cfg_get(SHARED_CFG, "threshold.offset", 8))
TH_MIN = int(_cfg_get(SHARED_CFG, "threshold.min", 25))
TH_MAX = int(_cfg_get(SHARED_CFG, "threshold.max", 120))
DARK_MARGIN = int(_cfg_get(SHARED_CFG, "threshold.dark_margin", 12))
TRACK_COLOR_MODE = str(_cfg_get(SHARED_CFG, "threshold.track_color", "auto")).lower()

CURVE_SWITCH_CM = float(_cfg_get(SHARED_CFG, "pid.curve_switch_cm", 4.5))
KP_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.kp", 0.70))
KI_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.ki", 0.015))
KD_STRAIGHT = float(_cfg_get(SHARED_CFG, "pid.straight.kd", 0.12))
KP_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.kp", 1.05))
KI_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.ki", 0.008))
KD_CURVE = float(_cfg_get(SHARED_CFG, "pid.curve.kd", 0.18))
I_CLAMP = float(_cfg_get(SHARED_CFG, "pid.i_clamp", 60.0))

STEER_SAT = float(_cfg_get(SHARED_CFG, "steer.sat", 70.0))
STEER_SCALE = float(_cfg_get(SHARED_CFG, "steer.scale", 22.0))
STEER_DEADBAND = float(_cfg_get(SHARED_CFG, "steer.deadband", 6.0))
LOST_HOLD_FRAMES = int(_cfg_get(SHARED_CFG, "lost.hold_frames", 6))
LOST_SEARCH_TURN = float(_cfg_get(SHARED_CFG, "lost.search_turn", 26.0))
LOST_PREFER_LEFT = bool(_cfg_get(SHARED_CFG, "lost.prefer_left", True))

PIX_LOOKAHEAD_GAIN = float(_cfg_get(SHARED_CFG, "webots.pixel_lookahead_gain", 0.0))
PIX_CURVE_GAIN = float(_cfg_get(SHARED_CFG, "webots.pixel_curve_gain", 0.0))
PIX_ANGLE_GAIN = float(_cfg_get(SHARED_CFG, "webots.pixel_angle_gain", 0.06))
CURVE_SWITCH_PX = float(_cfg_get(SHARED_CFG, "webots.curve_switch_px", 18.0))
RIGHT_TURN_SCALE = float(_cfg_get(SHARED_CFG, "webots.right_turn_scale", 0.65))
LEFT_CURVE_OUTWARD_GAIN = float(_cfg_get(SHARED_CFG, "webots.left_curve_outward_gain", 0.35))
LEFT_CURVE_OUTWARD_PX = float(_cfg_get(SHARED_CFG, "webots.left_curve_outward_px", 6.0))

ROBUST_CFG = _cfg_get(SHARED_CFG, "shake_robust", {}) or {}
ROBUST_ENABLE = bool(ROBUST_CFG.get("enable", True))
ROBUST_DIFF_WINDOW = int(ROBUST_CFG.get("diff_window", 5))
ROBUST_DIFF_RMS_TRIGGER_PX = float(ROBUST_CFG.get("diff_rms_trigger_px", 6.0))
ROBUST_ALPHA_HIGH = float(ROBUST_CFG.get("alpha_high", 0.88))
ROBUST_BOTTOM_LOCK_BLEND_SCALE = float(ROBUST_CFG.get("bottom_lock_blend_scale", 1.5))
ROBUST_KD_SHAKE_SCALE = float(ROBUST_CFG.get("kd_shake_scale", 0.6))
ROBUST_STEER_RATE_LIMIT = float(ROBUST_CFG.get("steer_rate_limit_per_frame", 14.0))
ROBUST_DECAY_FRAMES = int(ROBUST_CFG.get("decay_frames", 8))

row_distance_cm = [0.0] * IMG_H
row_cm_per_px = [0.0] * IMG_H


def clamp(v, lo, hi):
	if v < lo:
		return lo
	if v > hi:
		return hi
	return v


def _qr_image_for_decode(src):
	"""裁 ROI（可选）再整数倍放大，提高 QQVGA 下二维码模块像素数。"""
	w0 = src.width()
	h0 = src.height()
	rw = float(OMV_WA.get("qr_roi_width_frac", 1.0))
	rh = float(OMV_WA.get("qr_roi_height_frac", 1.0))
	anchor = str(OMV_WA.get("qr_roi_y_anchor", "bottom"))
	rw = clamp(rw, 0.2, 1.0)
	rh = clamp(rh, 0.2, 1.0)
	if rw >= 0.999 and rh >= 0.999:
		qimg = src.copy()
	else:
		roi_w = max(8, min(w0, int(w0 * rw + 0.5)))
		roi_h = max(8, min(h0, int(h0 * rh + 0.5)))
		rx = max(0, (w0 - roi_w) // 2)
		if anchor == "center":
			ry = max(0, (h0 - roi_h) // 2)
		else:
			ry = max(0, h0 - roi_h)
		qimg = src.copy(roi=(rx, ry, roi_w, roi_h))
	ps = int(OMV_WA.get("qr_patch_scale", 2))
	ps = max(1, min(ps, 4))
	if ps > 1:
		nw = max(8, qimg.width() * ps)
		nh = max(8, qimg.height() * ps)
		try:
			qimg = qimg.resize(nw, nh)
		except Exception:
			pass
	if bool(OMV_WA.get("qr_decode_histeq", False)):
		try:
			qimg = qimg.histeq()
		except Exception:
			pass
	return qimg


def _qr_extract_payload(qimg):
	"""从图像提取可动作二维码 payload；
	按尺寸过滤（后 upsample 尺度），返回 (payload, debug_dict) 或 (None, None)。"""
	if qimg is None:
		return None, None
	try:
		qrs = qimg.find_qrcodes()
	except Exception:
		return None, None
	if not qrs:
		return None, None
	best = None
	best_area = 0
	best_dbg = None
	for qr in qrs:
		pl = qr.payload().strip()
		if pl not in QR_ACTION_MAP:
			continue
		w, h = qr.w(), qr.h()
		mx = max(w, h)
		mn = min(w, h)
		area = w * h
		if mx < QR_MIN_PIXELS or mx > QR_MAX_PIXELS:
			continue
		if mn <= 0:
			continue
		if area < QR_MIN_AREA:
			continue
		if area > best_area:
			best_area = area
			best = pl
			best_dbg = {"w": w, "h": h, "x": qr.x(), "y": qr.y(), "area": area}
	return best, best_dbg


def _qr_try_decode_from_base(qbase):
	"""先 lens_corr（若配置），失败则可再试未校正图（透视大时偶发更稳）。
	返回 (payload, debug_info) 或 (None, None)。"""
	if qbase is None:
		return None, None
	order = []
	if QR_LENS_CORR > 0.01:
		order.append(True)
	order.append(False)
	for use_lens in order:
		if use_lens is False and len(order) > 1 and (not QR_RETRY_NO_LENS):
			break
		try:
			qx = qbase.copy()
			if use_lens:
				qx = qx.lens_corr(QR_LENS_CORR)
			pl, dbg = _qr_extract_payload(qx)
			if pl is not None:
				if dbg is not None:
					dbg["lens"] = use_lens
				return pl, dbg
		except Exception:
			pass
	return None, None


def median(vals):
	n = len(vals)
	if n == 0:
		return None
	s = sorted(vals)
	m = n // 2
	if n & 1:
		return s[m]
	return 0.5 * (s[m - 1] + s[m])


def stdev(vals):
	n = len(vals)
	if n < 2:
		return 0.0
	mu = sum(vals) / n
	var = 0.0
	for v in vals:
		d = v - mu
		var += d * d
	return math.sqrt(var / (n - 1))


def line_fit(ys, xs):
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
		v_deg = ((IMG_H * 0.5 - y) / (IMG_H * 0.5)) * half_v
		ray_deg = CAM_PITCH_DEG + v_deg
		if ray_deg < 1.0:
			ray_deg = 1.0
		z_cm = CAM_HEIGHT_CM / math.tan(math.radians(ray_deg))
		z_cm = clamp(z_cm, ROW_DIST_MIN_CM, ROW_DIST_MAX_CM)
		row_distance_cm[y] = z_cm
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


def adaptive_black_threshold(img):
	th = img.get_histogram().get_threshold().value()
	return clamp(th + TH_OFFSET, TH_MIN, TH_MAX)


def pixel_is_track_val(g, black_th, track_is_dark):
	if track_is_dark:
		return g <= max(0, black_th - DARK_MARGIN)
	return g >= min(255, black_th + DARK_MARGIN)


def pixel_is_red_img(_img, _x, _y):
	return False


def detect_track_is_dark(img, black_th):
	if TRACK_COLOR_MODE == "dark":
		return True
	if TRACK_COLOR_MODE == "light":
		return False
	dark = 0
	light = 0
	step_y = max(1, IMG_H // 20)
	step_x = max(1, IMG_W // 20)
	y = 0
	while y < IMG_H:
		x = 0
		while x < IMG_W:
			g = img.get_pixel(x, y)
			if g <= black_th:
				dark += 1
			else:
				light += 1
			x += step_x
		y += step_y
	return dark <= light


def detect_row_blocker(img, y, x0, x1, black_th, track_is_dark):
	total = max(1, x1 - x0 + 1)
	track_count = 0
	red_count = 0
	longest_track_run = 0
	cur_run = 0
	x = x0
	while x <= x1:
		g = img.get_pixel(x, y)
		is_track = pixel_is_track_val(g, black_th, track_is_dark)
		if is_track:
			track_count += 1
			cur_run += 1
			if cur_run > longest_track_run:
				longest_track_run = cur_run
		else:
			cur_run = 0
		if RED_DETECT_ENABLE and pixel_is_red_img(img, x, y):
			red_count += 1
		x += 1
	red_ratio = red_count / float(total)
	cover_ratio = track_count / float(total)
	run_ratio = longest_track_run / float(total)
	red_block = red_ratio >= RED_ROW_RATIO
	black_block = (run_ratio >= CROSS_BLACK_RUN_RATIO) and (cover_ratio >= CROSS_BLACK_COVER_RATIO)
	return red_block, black_block


def collect_track_runs_on_row(img, y, x0, x1, black_th, track_is_dark):
	runs = []
	run_start = -1
	x = x0
	while x <= x1:
		g = img.get_pixel(x, y)
		is_tr = pixel_is_track_val(g, black_th, track_is_dark)
		if is_tr and run_start < 0:
			run_start = x
		elif (not is_tr) and run_start >= 0:
			run_end = x - 1
			w = run_end - run_start + 1
			if w >= MIN_LINE_WIDTH and w <= MAX_LINE_WIDTH:
				runs.append((run_start, run_end))
			run_start = -1
		x += 1
	if run_start >= 0:
		run_end = x1
		w = run_end - run_start + 1
		if w >= MIN_LINE_WIDTH and w <= MAX_LINE_WIDTH:
			runs.append((run_start, run_end))
	return runs


def choose_pair_center_from_runs(runs, hint_center, lane_width_hint, x0, x1):
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
							"conf": 1.0,
							"line_mode": 2,
						}
			j += 1
		i += 1
	return best


def choose_single_run_near_hint(runs, hint_center):
	if not runs:
		return None
	best = None
	best_err = 1e9
	for run in runs:
		c = 0.5 * (run[0] + run[1])
		err = abs(c - hint_center)
		if err < best_err:
			best_err = err
			best = run
	return best


def infer_center_from_single_run(run, hint_center, lane_width_hint, x0, x1):
	c = 0.5 * (run[0] + run[1])
	w = max(float(lane_width_hint), float(MIN_TRACK_WIDTH))
	cand_left = c + 0.5 * w
	cand_right = c - 0.5 * w
	if abs(cand_left - hint_center) < abs(cand_right - hint_center):
		center = cand_left
	elif abs(cand_left - hint_center) > abs(cand_right - hint_center):
		center = cand_right
	else:
		center = cand_left if c < IMG_CX else cand_right
	center = clamp(center, x0, x1)
	return {
		"center_px": center,
		"lane_width_px": w,
		"conf": SINGLE_LINE_CONF,
		"line_mode": 1,
	}


def find_lr_edges_on_row(img, y, x0, x1, black_th, track_is_dark, hint_x, lane_width_hint):
	hint = int(clamp(hint_x, x0, x1))
	left = -1
	x = hint
	while x >= x0:
		g = img.get_pixel(x, y)
		if pixel_is_track_val(g, black_th, track_is_dark):
			left = x
			break
		x -= 1
	right = -1
	x = hint
	while x <= x1:
		g = img.get_pixel(x, y)
		if pixel_is_track_val(g, black_th, track_is_dark):
			right = x
			break
		x += 1
	if left < 0 or right < 0:
		left = -1
		right = -1
	lane_w = 0
	if left >= 0 and right >= 0:
		lane_w = right - left
		if lane_w < MIN_TRACK_WIDTH or lane_w > MAX_TRACK_WIDTH:
			left = -1
			right = -1
	if left >= 0 and right >= 0:
		center = 0.5 * (left + right)
		if abs(center - hint_x) > MAX_CENTER_JUMP_PX:
			left = -1
			right = -1
	if left >= 0 and right >= 0 and lane_width_hint > 0 and abs(lane_w - lane_width_hint) > LANE_WIDTH_TOL_PX:
		left = -1
		right = -1
	if left >= 0 and right >= 0:
		return left, right
	runs = collect_track_runs_on_row(img, y, x0, x1, black_th, track_is_dark)
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
			lane_w2 = rj - li
			if lane_w2 >= MIN_TRACK_WIDTH and lane_w2 <= MAX_TRACK_WIDTH:
				center = 0.5 * (li + rj)
				if abs(center - hint_x) <= (MAX_CENTER_JUMP_PX * 1.8):
					width_err = abs(lane_w2 - lane_width_hint) if lane_width_hint > 0 else 0.0
					if lane_width_hint <= 0 or width_err <= (LANE_WIDTH_TOL_PX * 2.0):
						score = abs(center - hint_x) + 0.7 * width_err
						if score < best_score:
							best_score = score
							best = (int(li), int(rj))
			j += 1
		i += 1
	return best


def scan_band_midline(
		img,
		black_th,
		track_is_dark,
		hint_x,
		lane_width_hint,
		y_start_ratio,
		y_end_ratio,
		max_rows,
		row_step,
):
	row_step = max(1, row_step)
	x0 = 0
	x1 = IMG_W - 1
	y_start = int(clamp(y_start_ratio * IMG_H, 0, IMG_H - 1))
	y_end = int(clamp(y_end_ratio * IMG_H, 0, IMG_H - 1))
	if y_end < y_start:
		y_end = y_start
	centers_px = []
	centers_cm = []
	lane_widths = []
	ys = []
	zs_cm = []
	conf_sum = 0.0
	pair_rows = 0
	single_rows = 0
	red_block_rows = 0
	black_block_rows = 0
	last_center = hint_x
	last_width = lane_width_hint
	rows_done = 0
	y = y_start
	while y <= y_end and rows_done < max_rows:
		red_block, black_block = detect_row_blocker(img, y, x0, x1, black_th, track_is_dark)
		if red_block:
			red_block_rows += 1
			rows_done += 1
			y += row_step
			continue
		if black_block:
			black_block_rows += 1
			rows_done += 1
			y += row_step
			continue
		runs = collect_track_runs_on_row(img, y, x0, x1, black_th, track_is_dark)
		chosen = choose_pair_center_from_runs(runs, last_center, last_width, x0, x1)
		if chosen is None and len(runs) >= 1:
			best_run = choose_single_run_near_hint(runs, last_center)
			if best_run is not None:
				chosen = infer_center_from_single_run(best_run, last_center, last_width, x0, x1)
		if chosen is not None:
			center_px = chosen["center_px"]
			lane_w = chosen["lane_width_px"]
			if abs(center_px - last_center) > (MAX_CENTER_JUMP_PX * 2.2):
				rows_done += 1
				y += row_step
				continue
			x_cm, z_cm = px_to_ground_cm(center_px, y)
			centers_px.append(center_px)
			centers_cm.append(x_cm)
			lane_widths.append(lane_w)
			ys.append(y)
			zs_cm.append(z_cm)
			conf_sum += chosen["conf"]
			if int(chosen.get("line_mode", 1)) >= 2:
				pair_rows += 1
			else:
				single_rows += 1
			last_center = center_px
			last_width = lane_w
		rows_done += 1
		y += row_step
	if len(centers_px) < 3:
		return None
	center_px = median(centers_px)
	center_cm = median(centers_cm)
	lane_width_px = median(lane_widths)
	dist_cm = median(zs_cm)
	width_std = stdev(lane_widths)
	a, _ = line_fit(ys, centers_px)
	angle = math.degrees(math.atan(a))
	hit_ratio = len(centers_px) / float(max(1, max_rows))
	conf_raw = (conf_sum / float(max(1, len(centers_px)))) * hit_ratio
	conf = conf_raw * (1.0 - clamp(width_std / max(WIDTH_STD_MAX, 1e-6), 0.0, 1.0))
	blocker_ratio = (red_block_rows + black_block_rows) / float(max(1, max_rows))
	if blocker_ratio > 0.25:
		conf *= (1.0 - 0.55 * clamp((blocker_ratio - 0.25) / 0.75, 0.0, 1.0))
	valid_rows = max(1, pair_rows + single_rows)
	pair_ratio = pair_rows / float(valid_rows)
	single_ratio = single_rows / float(valid_rows)
	return {
		"center_cm": center_cm,
		"center_px": center_px,
		"dist_cm": dist_cm,
		"lane_width_px": lane_width_px,
		"weight": 1.0,
		"angle": angle,
		"conf": conf,
		"pair_ratio": pair_ratio,
		"single_ratio": single_ratio,
		"red_block_ratio": red_block_rows / float(max(1, max_rows)),
		"black_block_ratio": black_block_rows / float(max(1, max_rows)),
	}


def bottom_quarter_midline(img, black_th, track_is_dark, hint_x, lane_width_hint):
	base = scan_band_midline(
		img,
		black_th,
		track_is_dark,
		hint_x,
		lane_width_hint,
		BOTTOM_START_RATIO,
		1.0,
		BOTTOM_ROWS,
		BOTTOM_STEP,
	)
	if base is None:
		return None
	if ASSIST_ENABLE:
		assist = scan_band_midline(
			img,
			black_th,
			track_is_dark,
			base["center_px"],
			base["lane_width_px"],
			ASSIST_START_RATIO,
			ASSIST_END_RATIO,
			ASSIST_ROWS,
			ASSIST_STEP,
		)
		if assist is not None:
			base["assist_center_px"] = assist["center_px"]
			base["assist_center_cm"] = assist["center_cm"]
			base["assist_dist_cm"] = assist["dist_cm"]
			base["assist_angle_deg"] = assist["angle"]
			base["assist_conf"] = assist["conf"]
	return base


def detect_three_band_lanes(img, black_th, track_is_dark, hint_x, lane_width_hint):
	band_specs = [
		("down", BAND_DOWN_START_RATIO, 1.0, BAND_ROWS_DOWN, BAND_STEP_DOWN, BAND_WEIGHT_DOWN),
		("mid", BAND_MID_START_RATIO, BAND_DOWN_START_RATIO, BAND_ROWS_MID, BAND_STEP_MID, BAND_WEIGHT_MID),
		("up", BAND_UP_START_RATIO, BAND_MID_START_RATIO, BAND_ROWS_UP, BAND_STEP_UP, BAND_WEIGHT_UP),
	]
	results = []
	last_center = hint_x
	last_width = lane_width_hint
	for name, ys, ye, rows, step, weight in band_specs:
		res = scan_band_midline(
			img,
			black_th,
			track_is_dark,
			last_center,
			last_width,
			ys,
			ye,
			rows,
			step,
		)
		if res is None:
			continue
		res["weight"] = weight
		res["band_name"] = name
		results.append(res)
		last_center = res["center_px"]
		last_width = res["lane_width_px"]
	return results


def band_bit(name):
	if name == "down":
		return 0x1
	if name == "mid":
		return 0x2
	if name == "up":
		return 0x4
	return 0


def pick_result_by_band(results, order):
	for name in order:
		for r in results:
			if str(r.get("band_name", "")) == name:
				return r
	return None


def _pick_min_dist_cm(rs):
	best = rs[0]
	bd = best["dist_cm"]
	i = 1
	while i < len(rs):
		if rs[i]["dist_cm"] < bd:
			best = rs[i]
			bd = best["dist_cm"]
		i += 1
	return best


def _pick_max_dist_cm(rs):
	best = rs[0]
	bd = best["dist_cm"]
	i = 1
	while i < len(rs):
		if rs[i]["dist_cm"] > bd:
			best = rs[i]
			bd = best["dist_cm"]
		i += 1
	return best


def result_quality_weight(r):
	pair_ratio = float(r.get("pair_ratio", 0.0))
	single_ratio = float(r.get("single_ratio", 1.0 - pair_ratio))
	q = 0.60 + 0.40 * pair_ratio
	if pair_ratio < MIN_PAIR_RATIO:
		q *= 0.80
	q *= (1.0 - 0.12 * clamp(single_ratio, 0.0, 1.0))
	return clamp(q, 0.20, 1.00)


def detect_bottom_center_lock(img, black_th, track_is_dark):
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
		red_block, black_block = detect_row_blocker(img, y, x0, x1, black_th, track_is_dark)
		if red_block or black_block:
			rows_done += 1
			y += row_step
			continue
		runs = collect_track_runs_on_row(img, y, x0, x1, black_th, track_is_dark)
		chosen = choose_pair_center_from_runs(runs, IMG_CX, 0.0, x0, x1)
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


def single_band_mask(mask):
	return mask in (0x1, 0x2, 0x4)


def build_rois():
	rois = []
	for rx, ry, rw, rh, w in ROI_RATIOS:
		x = int(rx * IMG_W)
		y = int(ry * IMG_H)
		ww = int(rw * IMG_W)
		hh = int(rh * IMG_H)
		if x < 0:
			x = 0
		if y < 0:
			y = 0
		if x + ww > IMG_W:
			ww = IMG_W - x
		if y + hh > IMG_H:
			hh = IMG_H - y
		rois.append((x, y, ww, hh, w))
	return rois


ROIS_BUILT = build_rois()


def roi_midline(img, roi, black_th, track_is_dark, hint_x, lane_width_hint):
	x, y, w, h, weight = roi
	x0 = x
	x1 = x + w - 1
	centers_cm = []
	centers_px = []
	lane_widths = []
	ys = []
	zs_cm = []
	step = h // (SCAN_LINES_PER_ROI + 1)
	if step < 1:
		step = 1
	i = 1
	last_center = hint_x
	last_width = lane_width_hint
	while i <= SCAN_LINES_PER_ROI:
		sy = y + i * step
		if sy >= IMG_H:
			sy = IMG_H - 1
		lr = find_lr_edges_on_row(img, sy, x0, x1, black_th, track_is_dark, last_center, last_width)
		if lr is not None:
			left, right = lr
			lane_w = right - left
			center_px = 0.5 * (left + right)
			x_cm, z_cm = px_to_ground_cm(center_px, sy)
			centers_cm.append(x_cm)
			centers_px.append(center_px)
			zs_cm.append(z_cm)
			lane_widths.append(lane_w)
			ys.append(sy)
			last_center = center_px
			last_width = lane_w
		i += 1
	if len(centers_cm) < MIN_VALID_LINES:
		return None
	center_cm = median(centers_cm)
	center_px = median(centers_px)
	dist_cm = median(zs_cm)
	lane_width_px = median(lane_widths)
	width_std = stdev(lane_widths)
	conf = (len(centers_cm) / float(SCAN_LINES_PER_ROI)) * (
		1.0 - clamp(width_std / WIDTH_STD_MAX, 0.0, 1.0)
	)
	a, _ = line_fit(ys, centers_px)
	angle = math.degrees(math.atan(a))
	return {
		"center_cm": center_cm,
		"center_px": center_px,
		"dist_cm": dist_cm,
		"lane_width_px": lane_width_px,
		"weight": weight,
		"angle": angle,
		"conf": conf,
	}


def nonlinear_map(v):
	return STEER_SAT * math.tanh(v / STEER_SCALE)


def pid_step(err, dt, st, curve_mode=False, kd_scale=1.0):
	if not curve_mode:
		kp, ki, kd = KP_STRAIGHT, KI_STRAIGHT, KD_STRAIGHT
	else:
		kp, ki, kd = KP_CURVE, KI_CURVE, KD_CURVE
	kd = kd * float(kd_scale)
	st["integral"] += err * dt
	st["integral"] = clamp(st["integral"], -I_CLAMP, I_CLAMP)
	derr = (err - st["last_err"]) / max(dt, 1e-3)
	st["last_err"] = err
	return kp * err + ki * st["integral"] + kd * derr


def steer_to_cmd(steer):
	if abs(steer) <= STEER_DEADBAND:
		return ROUTE_GO
	if steer > 0:
		return ROUTE_RIGHT
	if steer < -18:
		return ROUTE_LEFT
	return ROUTE_SLIGHT_LEFT


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


build_camera_lut()
LED(1).on()
LED(2).on()
LED(3).on()

state = {
	"integral": 0.0,
	"last_err": 0.0,
	"last_steer": 0.0,
	"smoothed_err": 0.0,
	"lost_frames": 0,
	"last_base_err": 0.0,
	"last_angle_err": 0.0,
	"last_far_dist": 0.0,
	"last_lane_center_x": float(IMG_CX),
	"last_lane_width_px": float(LANE_WIDTH_INIT_PX),
	"track_dark_score": 0,
	"last_band_mask": 0,
	"startup_frames": 0,
	"last_bottom_lock_valid": False,
	"near_err_history": [],
	"shake_active_frames": 0,
	"diff_rms_px": 0.0,
}
last_frame_ms = now_ms()

_qr_tick = 0
_qr_cand = None
_qr_cand_n = 0
_last_qr_ms = None

while True:
	clock.tick()
	state["startup_frames"] += 1
	now = now_ms()
	dt = time.ticks_diff(now, last_frame_ms) / 1000.0
	if dt <= 0:
		dt = 0.001
	last_frame_ms = now

	img = sensor.snapshot()
	if USE_HISTEQ:
		img = img.histeq()

	black_th = adaptive_black_threshold(img)
	track_dark_candidate = detect_track_is_dark(img, black_th)
	if track_dark_candidate:
		state["track_dark_score"] = int(clamp(state["track_dark_score"] + 1, -6, 6))
	else:
		state["track_dark_score"] = int(clamp(state["track_dark_score"] - 1, -6, 6))
	track_is_dark = state["track_dark_score"] >= 0

	startup_active = (STARTUP_SETTLE_FRAMES > 0) and (state["startup_frames"] < STARTUP_SETTLE_FRAMES)
	if startup_active:
		conf_min_dyn = CONF_MIN * clamp(STARTUP_CONF_MIN_SCALE, 0.20, 1.00)
		min_weight_dyn = MIN_WEIGHT * clamp(STARTUP_MIN_WEIGHT_SCALE, 0.20, 1.00)
	else:
		conf_min_dyn = CONF_MIN
		min_weight_dyn = MIN_WEIGHT

	if startup_active:
		scan_hint_center = float(IMG_CX)
		scan_hint_width = 0.0
	else:
		scan_hint_center = state["last_lane_center_x"]
		scan_hint_width = state["last_lane_width_px"]

	roi_results = []
	if startup_active and STARTUP_FORCE_SIMPLE_BOTTOM:
		res = bottom_quarter_midline(img, black_th, track_is_dark, scan_hint_center, scan_hint_width)
		if res is not None and res["conf"] >= conf_min_dyn:
			roi_results.append(res)
		elif THREE_BAND_MODE:
			roi_results = detect_three_band_lanes(img, black_th, track_is_dark, scan_hint_center, scan_hint_width)
			roi_results = [r for r in roi_results if r["conf"] >= conf_min_dyn]
	elif THREE_BAND_MODE:
		roi_results = detect_three_band_lanes(img, black_th, track_is_dark, scan_hint_center, scan_hint_width)
		roi_results = [r for r in roi_results if r["conf"] >= conf_min_dyn]
	elif SIMPLE_BOTTOM_MODE:
		res = bottom_quarter_midline(img, black_th, track_is_dark, scan_hint_center, scan_hint_width)
		if res is not None and res["conf"] >= conf_min_dyn:
			roi_results.append(res)
	else:
		for roi in ROIS_BUILT:
			res = roi_midline(img, roi, black_th, track_is_dark, scan_hint_center, scan_hint_width)
			if res is not None and res["conf"] >= conf_min_dyn:
				roi_results.append(res)

	steer = 0.0
	base_err_cm = 0.0
	base_err_px = 0.0
	angle_err = 0.0
	far_dist_cm = 0.0
	turn_gate = 0.0
	curve_mode = 0
	avg_conf = 0.0
	band_mask = 0
	red_block_score = 0.0
	black_block_score = 0.0
	bottom_pair_ratio = 0.0
	bottom_sym_err_px = 0.0
	center_lock_quality = 1.0
	bottom_lock_valid = True

	bottom_lock = detect_bottom_center_lock(img, black_th, track_is_dark)
	bottom_pair_ratio = float(bottom_lock.get("pair_ratio", 0.0))
	bottom_sym_err_px = float(bottom_lock.get("center_err_px", 0.0))
	center_lock_quality = float(bottom_lock.get("quality", 0.0))
	bottom_lock_valid = bool(bottom_lock.get("valid", False))
	if BOTTOM_LOCK_ENABLE and LOCK_REACQUIRE_RESET and bottom_lock_valid and (not state["last_bottom_lock_valid"]):
		state["integral"] = 0.0
		state["last_err"] = 0.0
		state["smoothed_err"] *= 0.35
	state["last_bottom_lock_valid"] = bottom_lock_valid

	if roi_results:
		score_total = 0.0
		for r in roi_results:
			score_total += r["weight"] * r["conf"] * result_quality_weight(r)
		if score_total <= min_weight_dyn:
			roi_results = []

	if roi_results:
		state["lost_frames"] = 0
		near = pick_result_by_band(roi_results, ("down", "mid", "up"))
		if near is None:
			near = _pick_min_dist_cm(roi_results)
		far = pick_result_by_band(roi_results, ("up", "mid", "down"))
		if far is None:
			far = _pick_max_dist_cm(roi_results)
		band_mask = 0
		red_block_score = 0.0
		black_block_score = 0.0
		for r in roi_results:
			bn = str(r.get("band_name", ""))
			band_mask |= band_bit(bn)
			red_block_score = max(red_block_score, float(r.get("red_block_ratio", 0.0)))
			black_block_score = max(black_block_score, float(r.get("black_block_ratio", 0.0)))
		state["last_band_mask"] = band_mask

		near_err_cm = near["center_cm"]
		far_err_cm = far["center_cm"]
		near_err_px = near["center_px"] - IMG_CX
		far_err_px = far["center_px"] - IMG_CX

		if str(near.get("band_name", "")) != "down":
			near_err_cm = 0.68 * near_err_cm + 0.32 * state["last_base_err"]
			near_err_px = 0.68 * near_err_px + 0.32 * (state["last_lane_center_x"] - IMG_CX)

		shake_active = ROBUST_ENABLE and (state["shake_active_frames"] > 0)
		alpha_eff = ROBUST_ALPHA_HIGH if shake_active else SMOOTH_ALPHA
		lock_blend_scale = ROBUST_BOTTOM_LOCK_BLEND_SCALE if shake_active else 1.0
		kd_scale_eff = ROBUST_KD_SHAKE_SCALE if shake_active else 1.0

		if bottom_pair_ratio > 0.0:
			lock_gain = (BOTTOM_LOCK_BLEND * lock_blend_scale) * (0.55 + 0.45 * center_lock_quality)
			lock_gain = clamp(lock_gain, 0.0, 0.95)
			near_err_px = (1.0 - lock_gain) * near_err_px + lock_gain * bottom_sym_err_px
			near_err_cm = near_err_px * row_cm_per_px[IMG_H - 1]

		far_dist_cm = far["dist_cm"]
		state["last_lane_center_x"] = clamp(float(IMG_CX + near_err_px), 0.0, float(IMG_W - 1))
		state["last_lane_width_px"] = clamp(
			float(near["lane_width_px"]), float(MIN_TRACK_WIDTH), float(MAX_TRACK_WIDTH)
		)

		base_err_cm = near_err_cm
		base_err_px = near_err_px
		use_assist = False
		if SIMPLE_BOTTOM_MODE and ("assist_center_px" in near):
			assist_conf = float(near.get("assist_conf", 0.0))
			assist_delta = abs(float(near["assist_center_px"]) - float(near["center_px"]))
			if assist_conf >= max(CONF_MIN, 0.28) and assist_delta <= (MAX_CENTER_JUMP_PX * 1.2):
				use_assist = True
		if use_assist:
			far_err_px = float(near["assist_center_px"]) - IMG_CX
			far_err_cm = float(near.get("assist_center_cm", near_err_cm))
			far_dist_cm = float(near.get("assist_dist_cm", far_dist_cm))
			angle_err = 0.5 * (near["angle"] + float(near.get("assist_angle_deg", near["angle"])))
		else:
			angle_err = 0.5 * (near["angle"] + far["angle"])

		curve_px = far_err_px - near_err_px
		avg_conf = sum((r["conf"] * result_quality_weight(r)) for r in roi_results) / float(len(roi_results))
		if not bottom_lock_valid:
			avg_conf *= (1.0 - BOTTOM_LOCK_CONF_PENALTY * (1.0 - center_lock_quality))
			avg_conf = clamp(avg_conf, 0.0, 1.0)

		near_norm = near_err_px / max(0.5 * IMG_W, 1.0)
		far_norm = far_err_px / max(0.5 * IMG_W, 1.0)
		curve_norm = curve_px / max(0.5 * IMG_W, 1.0)

		turn_gate = clamp(abs(curve_px) / max(CURVE_SWITCH_PX, 1.0), 0.0, 1.0)
		curve_mode_force = abs(curve_px) >= CURVE_SWITCH_PX
		if single_band_mask(band_mask):
			curve_mode_force = curve_mode_force or (abs(angle_err) >= 8.0)
		if (not bottom_lock_valid) and (bottom_pair_ratio > 0.0) and (abs(bottom_sym_err_px) > BOTTOM_LOCK_SYM_TOL_PX):
			curve_mode_force = True
		curve_mode = 1 if curve_mode_force else 0

		fused_err = -near_norm
		fused_err += PIX_LOOKAHEAD_GAIN * (-far_norm)
		fused_err += PIX_CURVE_GAIN * (-curve_norm)
		fused_err += PIX_ANGLE_GAIN * (-angle_err / 45.0)
		if curve_px < -LEFT_CURVE_OUTWARD_PX:
			fused_err += LEFT_CURVE_OUTWARD_GAIN * curve_norm

		state["smoothed_err"] = alpha_eff * state["smoothed_err"] + (1.0 - alpha_eff) * fused_err
		pid_out = pid_step(state["smoothed_err"], dt, state, curve_mode_force, kd_scale=kd_scale_eff)
		steer = nonlinear_map(pid_out)
		if steer < 0.0:
			steer *= RIGHT_TURN_SCALE

		hist = state["near_err_history"]
		hist.append(float(near_err_px))
		if len(hist) > ROBUST_DIFF_WINDOW + 1:
			del hist[0]
		if len(hist) >= 3:
			diffs = [hist[i] - hist[i - 1] for i in range(1, len(hist))]
			rms = math.sqrt(sum(d * d for d in diffs) / len(diffs))
			state["diff_rms_px"] = rms
			if rms >= ROBUST_DIFF_RMS_TRIGGER_PX:
				state["shake_active_frames"] = ROBUST_DECAY_FRAMES
			elif state["shake_active_frames"] > 0:
				state["shake_active_frames"] -= 1

		state["last_base_err"] = base_err_cm
		state["last_angle_err"] = angle_err
		state["last_far_dist"] = far_dist_cm
	else:
		state["lost_frames"] += 1
		base_err_cm = state["last_base_err"]
		base_err_px = state["last_lane_center_x"] - IMG_CX
		angle_err = state["last_angle_err"]
		far_dist_cm = state["last_far_dist"]
		avg_conf = 0.0
		band_mask = state["last_band_mask"]
		turn_gate = 1.0
		curve_mode = 1
		if state["lost_frames"] <= LOST_HOLD_FRAMES:
			steer = state["last_steer"] * 0.85
		else:
			if startup_active and STARTUP_LOST_BIAS_FREE:
				phase = (state["lost_frames"] // 8) % 2
				steer = abs(LOST_SEARCH_TURN) if phase == 0 else -abs(LOST_SEARCH_TURN)
			else:
				steer = abs(LOST_SEARCH_TURN) if LOST_PREFER_LEFT else -abs(LOST_SEARCH_TURN)

	if ROBUST_ENABLE and ROBUST_STEER_RATE_LIMIT > 0.0:
		prev_steer = float(state["last_steer"])
		d_steer = steer - prev_steer
		if abs(d_steer) > ROBUST_STEER_RATE_LIMIT:
			steer = prev_steer + math.copysign(ROBUST_STEER_RATE_LIMIT, d_steer)
	state["last_steer"] = steer

	cmd = steer_to_cmd(steer)
	route_u8 = route_to_u8(cmd)
	mode_u8 = MODE_LOST_SEARCH if state["lost_frames"] > 0 else MODE_LINE_FOLLOW
	conf_u8 = int(clamp(round(avg_conf * 100.0), 0, 100))
	conf_u8 = int(clamp(quantize_to_step(conf_u8, CONF_STEP), 0, 100))
	lost_u8 = 1 if state["lost_frames"] > 0 else 0
	ex_mm = int(quantize_to_step(int(round(base_err_cm * 10.0)), EX_STEP_MM))
	ang_cdeg = int(quantize_to_step(int(round(angle_err * 100.0)), ANG_STEP_CDEG))

	n = now_ms()

	if QR_ENABLE:
		_qr_tick += 1
		track_ok = (state["lost_frames"] == 0) if QR_ONLY_WHEN_TRACKING else True
		conf_ok = avg_conf >= QR_MIN_CONF
		if track_ok and conf_ok and (_qr_tick % QR_EVERY_N_FRAMES == 0):
			try:
				qbase = _qr_image_for_decode(img)
				qr_pl, qr_dbg = _qr_try_decode_from_base(qbase)
				if qr_pl is not None:
					if qr_pl == _qr_cand:
						_qr_cand_n += 1
					else:
						_qr_cand = qr_pl
						_qr_cand_n = 1
						if QR_DEBUG_LOG and qr_dbg is not None:
							print("qr new pl=%s w=%d h=%d lens=%s"
								% (qr_pl, qr_dbg["w"], qr_dbg["h"], qr_dbg.get("lens", "?")))
					if _qr_cand_n >= QR_STABLE_FRAMES:
						if _last_qr_ms is None or time.ticks_diff(n, _last_qr_ms) >= QR_COOLDOWN_MS:
							u = QR_ACTION_MAP[qr_pl]
							if protocol is not None:
								fr = protocol.build_qr_event(u, request_ack=QR_REQUEST_ACK)
								if pending_acks is not None and QR_REQUEST_ACK:
									pending_acks.send(fr, fr[5], fr[3])
								else:
									_uart_send(fr)
							else:
								_uart_send(bytes([u]))
							_last_qr_ms = n
							if QR_DEBUG_LOG:
								print("qr send=%d (%s)" % (u, qr_pl))
							_qr_cand_n = 0
							LED(2).toggle()
				else:
					_qr_cand = None
					_qr_cand_n = 0
			except Exception:
				pass

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
				uart.write(
					protocol.build_line_ctrl(
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
				)
				last_ctrl_ms = n
				if safety is not None:
					safety.mark_sent_ctrl(n)
			except Exception:
				pass
	else:
		if time.ticks_diff(n, last_send_ms) >= SEND_INTERVAL_MS:
			try:
				uart.write(cmd)
			except Exception:
				pass
			last_send_ms = n

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
	if pending_acks is not None:
		pending_acks.tick()

	print(
		"wa th=%d st=%.1f ex=%.2f expx=%.1f ang=%.1f z=%.1f tg=%.2f cm=%d cf=%.2f lost=%d drms=%.2f sk=%d fps=%.1f"
		% (
			black_th,
			steer,
			base_err_cm,
			base_err_px,
			angle_err,
			far_dist_cm,
			turn_gate,
			curve_mode,
			avg_conf,
			state["lost_frames"],
			state["diff_rms_px"],
			state["shake_active_frames"],
			clock.fps(),
		)
	)
