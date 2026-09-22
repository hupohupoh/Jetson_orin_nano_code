# qr_uart_test.py — 仅测二维码识别 + UART1/115200
# 串口与 uart_link_test.py 一致。接线：P1(TX)→主控 RX，P0(RX)←主控 TX，GND 共地。
#
# 仅白名单 "1"~"6"；UART(P1) 每帧发 1 字节 uint8_t：取值 1~6（线路上 0x01~0x06，不是 ASCII 的 0x31~0x36）。
# 5 s 内至多发 1 次；不读串口。IDE 终端 print 与所发字节数值一致。

import sensor
import time
from pyb import UART, LED

UART_ID = 1
BAUD = 115200
try:
	uart = UART(UART_ID, BAUD, timeout_char=100)
except TypeError:
	uart = UART(UART_ID, BAUD)

QR_ACTION_MAP = {
	"1": 1,
	"2": 2,
	"3": 3,
	"4": 4,
	"5": 5,
	"6": 6,
}

QR_STABLE_FRAMES = 2
QR_SEND_COOLDOWN_MS = 5000
LENS_CORR_STRENGTH = 1.5
SENSOR_SKIP_MS = 2000

last_qr_sent_ms = None
qr_candidate = None
qr_candidate_count = 0


def now_ms():
	return time.ticks_ms()


def send_qr_action(payload):
	if payload not in QR_ACTION_MAP:
		return False
	u = QR_ACTION_MAP[payload]
	uart.write(bytes([u]))
	print(u)
	return True


def draw_qr_debug(img, qrs):
	"""绘制全部二维码框；返回首个白名单 payload 或 None。"""
	first_ok = None
	for qr in qrs:
		rect = qr.rect()
		payload = qr.payload().strip()
		box_color = 255 if payload in QR_ACTION_MAP else 80
		img.draw_rectangle(rect, color=box_color, thickness=3)
		cx = rect[0] + (rect[2] // 2)
		cy = rect[1] + (rect[3] // 2)
		img.draw_cross(cx, cy, color=box_color, size=8)
		img.draw_string(rect[0], max(0, rect[1] - 12), payload, color=box_color, scale=1)
		if payload in QR_ACTION_MAP and first_ok is None:
			first_ok = payload
	return first_ok


LED(1).on()
LED(2).on()

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time=SENSOR_SKIP_MS)

clock = time.clock()
#print("qr_uart_test: UART{} @ {}".format(UART_ID, BAUD))
#print("白名单 1~6；稳定 {} 帧 + 冷却 {} ms 后发送".format(QR_STABLE_FRAMES, QR_SEND_COOLDOWN_MS))

while True:
	clock.tick()
	img = sensor.snapshot().lens_corr(LENS_CORR_STRENGTH)
	now = now_ms()

	qrs = img.find_qrcodes()
	qr_payload = draw_qr_debug(img, qrs) if qrs else None

	if qr_payload is not None:
		if qr_payload == qr_candidate:
			qr_candidate_count += 1
		else:
			qr_candidate = qr_payload
			qr_candidate_count = 1

		if qr_candidate_count >= QR_STABLE_FRAMES:
			ready = last_qr_sent_ms is None or time.ticks_diff(now, last_qr_sent_ms) >= QR_SEND_COOLDOWN_MS
			if ready:
				if send_qr_action(qr_payload):
					last_qr_sent_ms = now
					qr_candidate_count = 0
					LED(2).toggle()
	else:
		qr_candidate = None
		qr_candidate_count = 0

#	print("fps={:.1f}".format(clock.fps()))
