import sensor, image, time  # 导入摄像头、图像处理和时间模块。# type: ignore
from pyb import LED, UART  # 导入LED和串口类。# type: ignore

LED(1).on()  # 点亮红灯，表示程序启动。
LED(2).on()  # 点亮绿灯，表示程序启动。
LED(3).on()  # 点亮蓝灯，表示程序启动。

sensor.reset()  # 复位摄像头。
sensor.set_pixformat(sensor.GRAYSCALE)  # 设置为灰度模式，便于巡线回归。
sensor.set_framesize(sensor.QVGA)  # 设置分辨率为 QVGA(320x240)。
sensor.skip_frames(time=2000)  # 等待摄像头参数稳定。

clock = time.clock()  # 创建时钟对象（可用于FPS统计）。

left_flag = 0  # 预留变量：左侧状态标记（当前逻辑未使用）。
right_flag = 0  # 预留变量：右侧状态标记（当前逻辑未使用）。
flag = 0  # 预留变量：流程标记（当前逻辑未使用）。

uart = UART(1, 9600)  # 初始化UART1，波特率9600，给主控发方向指令。

while True:  # 主循环：持续采图并输出巡线方向。
    clock.tick()  # 更新时钟。
    img = sensor.snapshot().lens_corr(1.5)  # 拍一帧并做镜头畸变校正。

    line = img.get_regression([(0, 79)], robust=True, roi=(0, 70, 70, 100), pixel_threshold=300)  # 左ROI线回归。
    line2 = img.get_regression([(0, 79)], robust=True, roi=(190, 50, 320, 120), pixel_threshold=300)  # 右ROI线回归。

    img.draw_rectangle((0, 70, 70, 100), thickness=2, fill=False)  # 可视化左侧ROI。
    img.draw_rectangle((190, 50, 320, 120), thickness=2, fill=False)  # 可视化右侧ROI。

    if (line) or (line2):  # 左右任意检测到线就进入方向判断。
        img.draw_line(line.line(), color=127) if line else None  # 若左线存在则画线。
        img.draw_line(line2.line(), color=127) if line2 else None  # 若右线存在则画线。
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
            uart.write('1')  # 串口输出'1'给主控。
        elif (line) and not (line2):  # 仅左线存在。
            print("left", line.theta())  # 打印左线角度。
#           flag = 3
            print("右转")  # 方向提示：右转。
            uart.write('1')  # 当前实现里右转仍发送'1'（前人逻辑原样保留）。
        elif not (line) and (line2):  # 仅右线存在。
            if 140 < line2.theta() < 160:  # 右线角度落在特定区间。
                print("边走边左转")  # 方向提示：缓慢左纠偏。
                uart.write('4')  # 发'4'给主控。
            else:  # 右线存在但角度不在特定区间。
                print("right", line2.theta())  # 打印右线角度。
#               flag = 2
                print("左转")  # 方向提示：左转。
                uart.write('2')  # 发'2'给主控。
    else:  # 左右都没有检测到线。
        print("右转")  # 保守策略：先右转寻找赛道。
        uart.write('1')  # 发'1'给主控。
#        pass
