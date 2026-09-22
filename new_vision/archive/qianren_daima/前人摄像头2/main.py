import sensor, image, time
from pyb import LED,UART
LED(1).on()
LED(2).on()
LED(3).on()
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)
clock = time.clock()
left_flag = 0
right_flag = 0
flag = 0
uart = UART(3,9600)
start_time = time.time()
right_flag = 0
flag1 = 0
theLast = ''
i = 0
zhaidao_flag = 0
while(True):
    clock.tick()
    end_time = time.time()

    print(abs(end_time - start_time))
    if abs(end_time - start_time) >= 90:
        zhaidao_flag = 1

    img = sensor.snapshot().lens_corr(1.5)
    line = img.get_regression([(0, 40)], robust = True,roi=(0,85,118,112),pixel_threshold=600)
    line2 = img.get_regression([(0, 40)], robust = True,roi=(207,49,105,133),pixel_threshold=800)
    img.draw_rectangle((0,85,118,112),thickness = 2, fill = False)
    img.draw_rectangle((207,49,105,133),thickness = 2, fill = False)
    if (line) or (line2):
        img.draw_line(line.line(), color=127) if line else None
        img.draw_line(line2.line(), color=127) if line2 else None
        if (line) and (line2):
            angle = line.theta()
            angle2 = line2.theta()
            print(angle,angle2,sep = '\n')
            uart.write('a')
            uart_read = uart.readline()
            if uart_read:
                print("直走")
                print("uart_read:",uart_read)
        elif (line) and not (line2):
            right_start_time = time.time()
            uart.write('c')
            uart_read = uart.readline()
            if uart_read:
                print("右转")
                print("uart_read:",uart_read)
        elif not (line) and (line2):
            left_start_time = time.time()
            uart.write('b')
            uart_read = uart.readline()
            if uart_read:
                print("左转")
                print("uart_read:",uart_read)
    else:
        uart.write('c')
        uart_read = uart.readline()
        if uart_read:
            print("左转")
            print("uart_read:",uart_read)

