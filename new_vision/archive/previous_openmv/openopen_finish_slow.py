import sensor, image, time  # 导入摄像头、图像处理和时间模块。# type: ignore
from pyb import LED, UART  # 导入LED和串口类。# type: ignore

LED(1).on()  # 点亮红灯，表示程序启动。
LED(2).on()  # 点亮绿灯，表示程序启动。
LED(3).on()  # 点亮蓝灯，表示程序启动。

sensor.reset()  # 复位摄像头。
sensor.set_pixformat(sensor.GRAYSCALE)  # 设置灰度模式。
sensor.set_framesize(sensor.QVGA)  # 设置分辨率为 QVGA。
sensor.skip_frames(time=2000)  # 等待参数稳定。

clock = time.clock()  # 创建时钟对象。

left_flag = 0  # 预留变量：左状态（当前逻辑未使用）。
right_flag = 0  # 预留变量：右状态（当前逻辑未使用）。
flag = 0  # 预留变量：流程状态（当前逻辑未使用）。

uart = UART(1, 9600)  # 初始化UART1，波特率9600。

while True:  # 主循环：持续做巡线检测。
    clock.tick()  # 更新时钟。
    img = sensor.snapshot().lens_corr(1.5)  # 拍一帧并做畸变校正。

    line = img.get_regression([(0, 79)], robust=True, roi=(0, 70, 70, 100), pixel_threshold=300)  # 左ROI线回归。
    line2 = img.get_regression([(0, 79)], robust=True, roi=(190, 50, 320, 120), pixel_threshold=300)  # 右ROI线回归。

    img.draw_rectangle((0, 70, 70, 100), thickness=2, fill=False)  # 可视化左ROI。
    img.draw_rectangle((190, 50, 320, 120), thickness=2, fill=False)  # 可视化右ROI。

    if (line) or (line2):  # 有任意一侧检测到线则进入判断。
        img.draw_line(line.line(), color=127) if line else None  # 左线存在则画左线。
        img.draw_line(line2.line(), color=127) if line2 else None  # 右线存在则画右线。
#        if line:
#            line.theta()
#        print(,line2.theta())
        if (line) and (line2):  # 左右都检测到线。
            print(line.theta(), line2.theta())  # 打印左右线角度。
#            if  30 <line.theta() < 90  and 30 < line2.theta() <90:
#                flag = 1
#                print("右转")
#                uart.write('3')
#            elif line.theta() > 90 and line2.theta() > 90:
#                flag = 2
#                print("left:%d,right:%d",line.theta(),line2.theta())
#                print("左转")
#                uart.write('2')
#            else:
#                flag = 0
            print("直走")  # 方向提示：直走。
            uart.write('1')  # 串口发'1'。
        elif (line) and not (line2):  # 仅左线存在。
            print("left", line.theta())  # 打印左线角度。
#            flag = 3
            print("右转")  # 方向提示：右转。
            uart.write('3')  # 串口发'3'。
        elif not (line) and (line2):  # 仅右线存在。
            print("right", line2.theta())  # 打印右线角度。
#            flag = 2
            print("左转")  # 方向提示：左转。
            uart.write('2')  # 串口发'2'。
    else:  # 两侧都未检测到线。
        print("右转")  # 保守策略：先右转找线。
        uart.write('1')  # 串口发'1'。
#        pass
