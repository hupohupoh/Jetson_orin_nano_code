# uart_link_test.py — 白底黑块 + UART1/115200 联通小测
# 识别到黑色区域 → 发一次简单信号；没有黑色 → 不发任何字节。
# 接线：P1(TX)→主控 RX，P0(RX)←主控 TX，GND 共地。
# 调参：BLACK_TH、MIN_AREA 按现场光强改。

import sensor
import image
import time
from pyb import UART, LED

UART_ID = 1
BAUD = 115200
try:
	uart = UART(UART_ID, BAUD, timeout_char=100)
except TypeError:
	uart = UART(UART_ID, BAUD)

# 灰度低于此值当作「黑」（白底上墨块、黑纸等）
BLACK_TH = 90
MIN_AREA = 150

# 检测到后发出的固定短信号（主控按字节解析即可）
TX_SIGNAL = b"*\n"

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=500)

LED(1).on()
LED(2).on()

had_blob = False

while True:
	img = sensor.snapshot()
	# 白背景上的暗物体：灰度落在 [0, BLACK_TH]
	thresholds = [(0, BLACK_TH)]
	blobs = img.find_blobs(
		thresholds,
		pixels_threshold=MIN_AREA,
		area_threshold=MIN_AREA,
		merge=True,
	)

	now = len(blobs) > 0

	# 上升沿：从无黑块 → 有黑块时发一次，避免按住物体时疯狂刷屏
	if now and not had_blob:
		uart.write(TX_SIGNAL)
		LED(2).toggle()
		print("tx:", TX_SIGNAL.strip())

	had_blob = now

	if uart.any():
		try:
			raw = uart.read()
			if raw:
				print("rx:", raw)
		except Exception as e:
			print("read err:", e)

	time.sleep_ms(30)
