# qr_test_hardened.py — QR 专项测试（算法/参数与 main_webots_aligned.py 一致）
#
# 用法：OpenMV IDE 终端运行，观察实时输出。
# 输出内容：
#   启动 — 打印当前 QR 全部参数
#   每帧 — 检测到几号码 / 尺寸 / 哪种解码路径命中 / 帧率
#   触发 — 候选累积 → 稳定确认 → 发送（含首帧→发送延迟 ms）
#   定期 — 每秒统计摘要（帧数/检测次数/发送次数/平均帧率）
#
# 调试参数可在 line_follow_params.json → openmv_webots_aligned 中调整：
#   qr_patch_scale, qr_roi_height_frac, qr_lens_corr_strength,
#   qr_stable_frames, qr_send_cooldown_ms, qr_min_pixels, qr_max_pixels, qr_min_area

import sensor
import image
import time
import gc
from pyb import UART, LED

try:
	import ujson as json
except ImportError:
	import json


# ── 配置加载（与主程序同一套逻辑）──────────────────────────────
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

# ── QR 参数（与主程序完全一致）──────────────────────────────────
QR_ENABLE = bool(OMV_WA.get("qr_enable", True))
QR_EVERY_N_FRAMES = max(1, int(OMV_WA.get("qr_every_n_frames", 3)))
QR_LENS_CORR = float(OMV_WA.get("qr_lens_corr_strength", 1.35))
QR_STABLE_FRAMES = max(1, int(OMV_WA.get("qr_stable_frames", 3)))
QR_COOLDOWN_MS = max(400, int(OMV_WA.get("qr_send_cooldown_ms", 3200)))
QR_RETRY_NO_LENS = bool(OMV_WA.get("qr_retry_without_lens_corr", True))
QR_MIN_PIXELS = int(OMV_WA.get("qr_min_pixels", 12))
QR_MAX_PIXELS = int(OMV_WA.get("qr_max_pixels", 300))
QR_MIN_AREA = int(OMV_WA.get("qr_min_area", 60))
QR_ROI_W = float(OMV_WA.get("qr_roi_width_frac", 1.0))
QR_ROI_H = float(OMV_WA.get("qr_roi_height_frac", 0.55))
QR_ROI_ANCHOR = str(OMV_WA.get("qr_roi_y_anchor", "bottom"))
QR_HISTEQ = bool(OMV_WA.get("qr_decode_histeq", True))
QR_ACTION_MAP = {"1": 1, "2": 2, "3": 3, "4": 4, "5": 5, "6": 6}

# 测试模式控制
SEND_UART = True              # 是否实际发送 UART（False=纯观察模式）
DRAW_DEBUG = True             # 是否在图像上画框和文字
LOG_EVERY_FRAME = True        # True=逐帧打日志，False=仅摘要
LOG_TO_FILE = True            # 是否把日志写到板载 flash 文件
LOG_FILE = "qr_test_log.txt"  # 日志文件名（存 OpenMV 板载 /flash/）

# VGA 全帧 640×480=307KB → OOM。改用 VGA sensor 开窗口只读下半区：
# 640×200=128KB（1.7x QVGA），有 VGA 级横向分辨率且省掉天空区域。
# QR 码在地面上，本来就在下半区，裁掉上半区不损失信息。
RESOLUTION = str(OMV_WA.get("qr_test_resolution", "QVGA")).upper()
if RESOLUTION in ("QQVGA",):
	SENSOR_FRAMESIZE = sensor.QQVGA
	SENSOR_WINDOW = None
elif RESOLUTION in ("QVGA",):
	SENSOR_FRAMESIZE = sensor.QVGA
	SENSOR_WINDOW = None
else:  # VGA-windowed: 640×200 bottom strip
	SENSOR_FRAMESIZE = sensor.VGA
	SENSOR_WINDOW = (0, 280, 640, 200)  # x, y, w, h — bottom 200 rows

# 按分辨率自动适配 patch_scale
if SENSOR_FRAMESIZE == sensor.QQVGA:
	QR_PATCH_SCALE = 3
elif SENSOR_FRAMESIZE == sensor.QVGA:
	QR_PATCH_SCALE = 2
else:
	QR_PATCH_SCALE = 1  # VGA-windowed: 640px 够宽，不需要放大

# QQVGA 裁下半区聚焦地面；QVGA/VGA 全图（或窗口已裁）搜索
if SENSOR_FRAMESIZE == sensor.QQVGA:
	QR_ROI_H = 0.55
else:
	QR_ROI_H = 1.0

# 测试时不跳帧 + 不要求连续稳定（远距离下帧间检测不稳定，1 帧命中即发）
QR_EVERY_N_FRAMES = 1
QR_STABLE_FRAMES = 1


# ── 工具函数 ────────────────────────────────────────────────────
def clamp(v, lo, hi):
	if v < lo:
		return lo
	if v > hi:
		return hi
	return v


def now_ms():
	return time.ticks_ms()


# ── QR 管线（与主程序 _qr_image_for_decode / _qr_extract_payload / _qr_try_decode_from_base 完全一致）──

def _qr_image_for_decode(src):
	"""准备解码用图。若无裁剪/放大/直方图需求则直接返回原图（省一次拷贝）。"""
	w0 = src.width()
	h0 = src.height()
	rw = clamp(QR_ROI_W, 0.2, 1.0)
	rh = clamp(QR_ROI_H, 0.2, 1.0)
	need_roi = rw < 0.999 or rh < 0.999
	need_scale = QR_PATCH_SCALE > 1
	if not need_roi and not need_scale and not QR_HISTEQ:
		return src  # 省拷贝，调用方只读
	if need_roi:
		roi_w = max(8, min(w0, int(w0 * rw + 0.5)))
		roi_h = max(8, min(h0, int(h0 * rh + 0.5)))
		rx = max(0, (w0 - roi_w) // 2)
		ry = max(0, h0 - roi_h) if QR_ROI_ANCHOR != "center" else max(0, (h0 - roi_h) // 2)
		qimg = src.copy(roi=(rx, ry, roi_w, roi_h))
	else:
		qimg = src.copy()
	ps = max(1, min(QR_PATCH_SCALE, 4))
	if ps > 1:
		try:
			qimg = qimg.resize(max(8, qimg.width() * ps), max(8, qimg.height() * ps))
		except Exception:
			pass
	if QR_HISTEQ:
		try:
			qimg = qimg.histeq()
		except Exception:
			pass
	return qimg


def _qr_extract_payload(qimg):
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
	all_info = []
	for qr in qrs:
		pl = qr.payload().strip()
		ok = pl in QR_ACTION_MAP
		w, h = qr.w(), qr.h()
		all_info.append((pl, w, h, ok, qr.x(), qr.y()))
		if not ok:
			continue
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
			best_dbg = {"w": w, "h": h, "x": qr.x(), "y": qr.y(), "area": area, "all": all_info}
	return best, best_dbg


def _qr_try_decode_from_base(qbase):
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


def _qr_scan_core(qx, label, lens_flag):
	"""对已准备好的图搜码，返回 (payload, debug) 或 (None, None)。不拷贝。"""
	pl, dbg = _qr_extract_payload(qx)
	if pl is not None:
		if dbg is not None:
			dbg["lens"] = lens_flag
			dbg["strategy"] = label
		return pl, dbg
	return None, None


def _qr_decode_multi_strategy(img):
	"""Multi-strategy decoder with explicit GC between attempts to keep heap low.
	Strategies ordered by expected hit-rate: pipe → raw_lens → raw."""
	# S1: production pipeline (crop → upscale → lens_corr/raw)
	qbase = _qr_image_for_decode(img)
	pl, dbg = _qr_try_decode_from_base(qbase)
	if pl is not None:
		if dbg is not None:
			dbg["strategy"] = "pipe"
		return pl, dbg
	del qbase  # 释放中间图
	gc.collect()

	# S2: lens_corr on full original → scan
	if QR_LENS_CORR > 0.01:
		try:
			qx = img.copy()
			qx.lens_corr(QR_LENS_CORR)
			pl2, dbg2 = _qr_scan_core(qx, "raw_lens", True)
			del qx
			gc.collect()
			if pl2 is not None:
				return pl2, dbg2
		except Exception:
			pass

	# S3: raw full image — 不拷贝，原图直接扫（_qr_extract_payload 只读）
	pl3, dbg3 = _qr_scan_core(img, "raw", False)
	if pl3 is not None:
		return pl3, dbg3

	return None, None
def draw_qr_debug(img, all_qr_info):
	"""在主图像上绘制所有检出二维码的框和标签。"""
	for pl, w, h, ok, qx, qy in all_qr_info:
		color = 255 if ok else 100
		try:
			img.draw_rectangle(qx, qy, w, h, color=color, thickness=2)
			label = "%s %dx%d" % (pl, w, h)
			img.draw_string(qx, max(0, qy - 10), label, color=color, scale=1)
		except Exception:
			pass


# ── 初始化硬件 ──────────────────────────────────────────────────
UART_ID = int(_cfg_get(SHARED_CFG, "output.uart.id", 1))
UART_BAUD = int(_cfg_get(SHARED_CFG, "output.uart.baud", 115200))
try:
	uart = UART(UART_ID, UART_BAUD, timeout_char=100)
except TypeError:
	uart = UART(UART_ID, UART_BAUD)

LED(1).on()
LED(2).on()
LED(3).off()

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(SENSOR_FRAMESIZE)
if SENSOR_WINDOW is not None:
	try:
		sensor.set_windowing(*SENSOR_WINDOW)
	except Exception:
		pass  # 窗口设置失败则用全帧
sensor.skip_frames(time=1500)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)

IMG_W = sensor.width()
IMG_H = sensor.height()


# ── 文件日志 ────────────────────────────────────────────────────
_log_fh = None

def _log_open():
	global _log_fh
	if not LOG_TO_FILE:
		return
	try:
		_log_fh = open(LOG_FILE, "w")  # 新会话覆盖旧日志
	except Exception:
		_log_fh = None


def _log_write(msg):
	"""同时写终端和日志文件。"""
	print(msg)
	if _log_fh is not None:
		try:
			_log_fh.write(msg + "\n")
			_log_fh.flush()
		except Exception:
			pass


def _log_close():
	global _log_fh
	if _log_fh is not None:
		try:
			_log_fh.close()
		except Exception:
			pass
		_log_fh = None


_log_open()

# ── 打印启动信息 ────────────────────────────────────────────────
def fmt_bool(v):
	return "1" if v else "0"


_log_write("")
_log_write("=" * 60)
_log_write("  QR Test — Multi-Strategy (pipe + raw_lens + raw)")
_log_write("  Sensor: %dx%d %s  |  No frame skip  |  Log: %s"
	% (IMG_W, IMG_H, RESOLUTION, LOG_FILE if LOG_TO_FILE else "off"))
_log_write("=" * 60)
_log_write("  patch_scale    : %d  (auto-adapted to resolution)" % QR_PATCH_SCALE)
_log_write("  roi            : w=%.2f  h=%.2f  anchor=%s" % (QR_ROI_W, QR_ROI_H, QR_ROI_ANCHOR))
_log_write("  histeq         : %s" % fmt_bool(QR_HISTEQ))
_log_write("  lens_corr      : %.2f  (retry_no_lens=%s)" % (QR_LENS_CORR, fmt_bool(QR_RETRY_NO_LENS)))
_log_write("  stable_frames  : %d" % QR_STABLE_FRAMES)
_log_write("  cooldown_ms    : %d" % QR_COOLDOWN_MS)
_log_write("  size_filter    : min=%dpx  max=%dpx  min_area=%d" % (QR_MIN_PIXELS, QR_MAX_PIXELS, QR_MIN_AREA))
_log_write("  send_uart      : %s  draw_debug=%s" % (fmt_bool(SEND_UART), fmt_bool(DRAW_DEBUG)))
if not SHARED_CFG:
	_log_write("  WARNING: could not load line_follow_params.json — using defaults")
_log_write("-" * 60)
_log_write("  Ready. Point camera at a QR code (1-6).")
_log_write("")
_log_write("  LEGEND:  . = no QR   3! = detected   3* = confirming")
_log_write("           3>>> = SENT via UART         3? = filter rejected")
_log_write("=" * 60)
_log_write("")

# ── 运行时状态 ──────────────────────────────────────────────────
clock = time.clock()
frame_idx = 0
qr_tick = 0
last_qr_send_ms = None
qr_candidate = None
qr_candidate_count = 0
first_candidate_ms = None       # 候选首次出现的时间（测延迟）

# 统计
stat_total_frames = 0
stat_detect_frames = 0          # 有 QR 检出的帧
stat_sent_count = 0             # 发送次数
stat_reject_count = 0           # 被尺寸过滤的次数
last_stat_ms = now_ms()
stat_latency_ms = []            # 延迟样本

# ── 主循环 ──────────────────────────────────────────────────────
while True:
	clock.tick()
	frame_idx += 1
	stat_total_frames += 1
	n = now_ms()

	img = sensor.snapshot()

	# 跳帧（与主程序一致）
	qr_tick += 1
	if qr_tick % QR_EVERY_N_FRAMES != 0:
		if LOG_EVERY_FRAME:
			print(".", end="")
		continue

	# ── QR 解码（与主程序 100% 一致） ──
	t0 = time.ticks_us()
	qr_pl, qr_dbg = _qr_decode_multi_strategy(img)
	t_decode_us = time.ticks_diff(time.ticks_us(), t0)

	# ── 提取全部检出信息用于调试绘制 ──
	all_qr_info = []
	if qr_dbg is not None and "all" in qr_dbg:
		all_qr_info = qr_dbg["all"]
	if DRAW_DEBUG and all_qr_info:
		draw_qr_debug(img, all_qr_info)

	# ── 终端输出 ──
	if qr_pl is not None:
		stat_detect_frames += 1
		w = qr_dbg["w"] if qr_dbg else 0
		h = qr_dbg["h"] if qr_dbg else 0
		lens = qr_dbg.get("lens", "?") if qr_dbg else "?"
		lens_str = "corr" if lens is True else ("raw" if lens is False else str(lens))
		strategy = qr_dbg.get("strategy", "?") if qr_dbg else "?"

		# 去重 + 稳定计数
		if qr_pl == qr_candidate:
			qr_candidate_count += 1
		else:
			qr_candidate = qr_pl
			qr_candidate_count = 1
			first_candidate_ms = n
			_log_write("")
			_log_write("  [%06d] NEW  pl=%s  %dx%d px  lens=%s  strat=%s  decode=%dus"
				% (frame_idx, qr_pl, w, h, lens_str, strategy, t_decode_us))

		# 稳定确认 → 发送
		sent = False
		if qr_candidate_count >= QR_STABLE_FRAMES:
			ready = (last_qr_send_ms is None
				or time.ticks_diff(n, last_qr_send_ms) >= QR_COOLDOWN_MS)
			if ready:
				latency = time.ticks_diff(n, first_candidate_ms) if first_candidate_ms else -1
				u = QR_ACTION_MAP[qr_pl]
				if SEND_UART:
					try:
						uart.write(bytes([u]))
					except Exception:
						pass
				last_qr_send_ms = n
				qr_candidate_count = 0
				stat_sent_count += 1
				stat_latency_ms.append(latency)
				sent = True
				LED(2).toggle()
				_log_write("")
				_log_write("  [%06d] >>> SEND action=%d (pl=%s)  latency=%dms  %dx%d px  lens=%s  strat=%s"
					% (frame_idx, u, qr_pl, latency, w, h, lens_str, strategy))
				_log_write("")

		# 逐帧符号
		if not sent and LOG_EVERY_FRAME:
			if qr_candidate_count > 0:
				print("%s*" % qr_pl, end="")      # 候选累积中
			else:
				print("%s!" % qr_pl, end="")      # 刚检测到
	else:
		# 未检出有效 QR
		if qr_candidate is not None:
			# 之前有候选，现在丢了
			if qr_dbg is None and LOG_EVERY_FRAME:
				print("_", end="")                # 彻底没检出
			else:
				if LOG_EVERY_FRAME:
					print("?", end="")            # 有检出但被过滤
				stat_reject_count += 1
		else:
			if LOG_EVERY_FRAME:
				print(".", end="")
		qr_candidate = None
		qr_candidate_count = 0
		first_candidate_ms = None

	# ── 每秒统计摘要 ──
	if time.ticks_diff(n, last_stat_ms) >= 1000:
		elapsed_s = time.ticks_diff(n, last_stat_ms) / 1000.0
		fps = stat_total_frames / max(1, elapsed_s) if elapsed_s > 0 else 0
		detect_pct = 100.0 * stat_detect_frames / max(1, stat_total_frames)
		avg_lat = int(sum(stat_latency_ms) / max(1, len(stat_latency_ms)))
		_log_write("")
		_log_write("  --- stats: frames=%d  detect=%.1f%%  sent=%d  reject=%d  latency_avg=%dms  fps=%.1f ---"
			% (stat_total_frames, detect_pct, stat_sent_count, stat_reject_count, avg_lat, fps))
		# 重置每秒计数
		stat_total_frames = 0
		stat_detect_frames = 0
		stat_sent_count = 0
		stat_reject_count = 0
		stat_latency_ms = []
		last_stat_ms = n
