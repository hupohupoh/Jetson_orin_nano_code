import time, sensor, image
from image import SEARCH_EX, SEARCH_DS
from pyb import LED
from machine import UART
sensor.reset()
sensor.set_contrast(1)
sensor.set_gainceiling(16)
sensor.set_framesize(sensor.QVGA)
sensor.set_pixformat(sensor.RGB565)
uart = UART(1, 115200)
temp = image.Image("./white.pgm")
flag = 2
thresholds = [(0, 100, 9, 64, 7, 34),
			  (0, 100, -53, -12, -4, 26),
			  (0, 100, -81, 24, -55, -8),
			  (0, 36, -22, 3, -2, 14)]
clock = time.clock()
green_flag = 0
red_flag = 0
blue_flag = 0
black_flag = 0
white_flag = 0
templates = ["./black.pgm","./white.pgm"]
red_led   = LED(1)
green_led = LED(2)
blue_led  = LED(3)
while(True):
	red_led.on()
	green_led.on()
	blue_led.on()
	clock.tick()
	img = sensor.snapshot().lens_corr(1.5)
	text = uart.read()
	if text is not None:
		print("text:",text)
	if flag == 1:
		img.draw_rectangle(100,20, 125,150,color=(0,0,255))
		if img.find_blobs(thresholds,roi=(100,20,125,150),pixels_threshold=3000, area_threshold=3000):
			for blob in img.find_blobs(thresholds,roi=(100,20,125,150),pixels_threshold=3000, area_threshold=3000):
				img.draw_rectangle(blob.rect())
				img.draw_cross(blob.cx(), blob.cy())
				color = blob.code()
				c = ''
				if color == 2:
					c = "a" + str(3) + "b"
					if green_flag == 0:
						uart.write(c)
						green_flag = 1
					print(c)
				elif color == 4:
					c = "a" + str(4) + "b"
					if blue_flag == 0:
						uart.write(c)
						blue_flag = 1
					print(c)
				elif color == 1:
					c = "a" + str(2) + "b"
					if red_flag == 0:
						uart.write(c)
						red_flag = 1
					print(c)
				elif color == 8:
					img = img.to_grayscale()
					for t in templates:
						template = image.Image(t)
						r = img.find_template(template, 0.70, step=4, search=SEARCH_EX)
						if r:
							img.draw_rectangle(r, color=0)
							if t == "./black.pgm":
								c = "a" + str(0) + "b"
								if black_flag == 0:
									uart.write(c)
									black_flag =1
								print(c)
								print(t)
		else:
			img = img.to_grayscale()
			for t in templates:
				template = image.Image(t)
				r = img.find_template(template, 0.70, step=4, search=SEARCH_EX)
				if r:
					img.draw_rectangle(r, color=0)
					if t == "./white.pgm":
						c = "a" + str(1) + "b"
						if white_flag == 0:
							uart.write(c)
							white_flag = 1
						print(c)
						print(t)
	elif flag == 2:
		if text == b"c1d" or text == b"c2d" or text == b"c3d" or text ==  b"c4d" or text == b"c5d" or text == b"c6d":
			start_time = time.time()
			flag = 3
		pass
	elif flag == 4:
		if text == b"c1d" or text == b"c2d" or text == b"c3d" or text ==  b"c4d" or text == b"c5d" or text == b"c6d" or text == b"c7d" or text == b"c8d" or text == b"c9d" or text == b"c10d" or text == b"c11d" or text == b"c12d" or text == b"c13d" or text == b"c14d" or text == b"c15d" or text == b"c16d":
		   flag = 1
		   print("开启颜色识别")
		pass
	elif flag  == 3:
		end_time = time.time()
		tim = end_time - start_time
		print("时间间隔",end_time - start_time)
		if  tim > 200:
			flag = 4
		img = img.to_grayscale()
		img.draw_rectangle(100,0,120,240,color = (255, 255, 0))
		r = img.find_template(temp, 0.75,step=4, search=SEARCH_EX ,roi=(100,0,120,240))
		if r:
			uart.write("stop")
			print("stop")
			time.sleep_ms(100)
			img.draw_rectangle(r)