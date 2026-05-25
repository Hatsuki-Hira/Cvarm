import wiringpi, time

wiringpi.wiringPiSetup()
pin = 6
wiringpi.pinMode(pin, wiringpi.GPIO.OUTPUT)
'''
for i in range(int(1.0*1000), int(1.5*1000), 100):
    high_time = i
    low_time = 20 * 1000 - high_time

    for _ in range(10):  # 10个周期
        wiringpi.digitalWrite(pin, 1)
        time.sleep(high_time / 1000.0 / 1000.0)
        wiringpi.digitalWrite(pin, 0)
        time.sleep(low_time / 1000.0 / 1000.0)

    time.sleep(0.000_1)  # 稍微停顿一下，防止过快
'''

# 机械臂当前角度
ROTATE_angle = 0

def set_angle(pin, angle, update_frame=False):
    global ROTATE_angle
    # 读取角度
    angle_old = ROTATE_angle

    # 计算当前角度
    angle_old = max(0, min(180, angle_old))
    angle_ms_old = int(20 * (angle_old / 180) + 5)
    angle_ms_old = max(0, min(24, angle_ms_old))

    angle = max(0, min(180, angle))
    angle_ms = int(20 * (angle / 180) + 5)
    angle_ms = max(0, min(24, angle_ms))

    wiringpi.softPwmCreate(pin, 0, 200) # 周期:20ms
    step = 1 if angle_ms > angle_ms_old else -1
    for i in range(angle_ms_old, angle_ms + step, step):
        wiringpi.softPwmWrite(pin, i)
        time.sleep(0.1)
    wiringpi.softPwmStop(pin)

    # 更新当前角度
    ROTATE_angle = angle

set_angle(4, 150)
