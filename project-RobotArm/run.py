import cv2
import cv2.aruco as aruco
#from ultralytics import YOLO
import time
import numpy as np
import math
import threading

import wiringpi



# 初始化摄像头
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cap.set(cv2.CAP_PROP_CONTRAST, 100)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 10)
frame = None

# 设置字典和参数
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
#parameters.maxErroneousBitsInBorderRate = 0.5

# 实例化 ArucoDetector 对象
detector = aruco.ArucoDetector(dictionary, parameters)

# 初始化yolo
#model = YOLO("yolo11n.pt")

# 原点坐标
center_x1, center_y1 = 0, 0
center_x2, center_y2 = 0, 0
center_x3, center_y3 = 0, 0
temp_origin_x, temp_origin_y = [0, 0, 0, 0, 0], [0, 0, 0, 0, 0]
origin = np.array([0.0, 0.0])
vec_origin = np.array([0.0, 0.0])
run = 0

# object坐标
object = np.array([0.0, 0.0])
object_direction = 0
object_angle = 0
object_pixel = 0.001
object_cm = 0.001

# ArUco码边长
CM_OF_ARUCO = 2.1
aruco_length_pixel = 0.001

# soft pwm
ARM1_PIN=3
ARM2_PIN=4
ROTATE_PIN=6
HAND_PIN=9
wiringpi.wiringPiSetup()
wiringpi.pinMode(ARM1_PIN, wiringpi.GPIO.OUTPUT)
wiringpi.pinMode(ARM2_PIN, wiringpi.GPIO.OUTPUT)
wiringpi.pinMode(ROTATE_PIN, wiringpi.GPIO.OUTPUT)
wiringpi.pinMode(HAND_PIN, wiringpi.GPIO.OUTPUT)



# 机械臂当前角度
ARM1_angle = 90
ARM2_angle = 30 + (30)
ROTATE_angle = 90
HAND_angle = 30

def set_angle(pin, angle, stop: bool = True, update_frame: bool = False):
    global ARM1_angle, ARM2_angle, ROTATE_angle, HAND_angle
    # 读取角度
    if pin == ARM1_PIN:
        angle_old = ARM1_angle
    elif pin == ARM2_PIN:
        angle_old = ARM2_angle
    elif pin == ROTATE_PIN:
        angle_old = ROTATE_angle
    elif pin == HAND_PIN:
        angle_old = HAND_angle

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
        time.sleep(0.087)
    if stop:
        wiringpi.softPwmStop(pin)

    # 更新当前角度
    if pin == ARM1_PIN:
        ARM1_angle = angle
    elif pin == ARM2_PIN:
        ARM2_angle = angle
    elif pin == ROTATE_PIN:
        ROTATE_angle = angle
    elif pin == HAND_PIN:
        HAND_angle = angle




def set_angle2(pin, angle, stop: bool = True, update_frame: bool = False):
    angle = max(0, min(180, angle))
    angle_ms = int(20 * (angle / 180) + 5)
    angle_ms = max(0, min(24, angle_ms))

    wiringpi.softPwmCreate(pin, 0, 200) # 周期:20ms
    wiringpi.softPwmWrite(pin, angle_ms)
    if update_frame:
        for i in range(4):
            cv2.imshow("UI", frame)
            time.sleep(0.15)
    else:
        time.sleep(0.6)
    if stop:
        wiringpi.softPwmStop(pin)




# 方向初始化
set_angle(ARM1_PIN, 90 - (15), update_frame=False)
set_angle(ARM2_PIN, 30 + (30), update_frame=False)
set_angle(ROTATE_PIN, 90, update_frame=False)
set_angle(HAND_PIN, 30, update_frame=False)



# 解三角形
def calculate_angles(x, y):
    L1 = 11
    L2 = 11
    D_sq = x**2 + y**2
    D = math.sqrt(D_sq)
    
    # 距离检查
    if D > (L1 + L2) or D < abs(L1 - L2):
        print("[error]目标超出机械臂工作范围")
        return None

    # 计算 theta2 (小臂与大臂)
    cos_theta2 = (D_sq - L1**2 - L2**2) / (2 * L1 * L2)
    # 限制范围防止精度溢出
    cos_theta2 = max(-1, min(1, cos_theta2))
    theta2 = math.acos(cos_theta2)

    # 计算 theta1 (大臂与基座)
    alpha = math.atan2(y, x)
    
    cos_beta = (L1**2 + D_sq - L2**2) / (2 * L1 * D)
    cos_beta = max(-1, min(1, cos_beta))
    beta = math.acos(cos_beta)
    
    theta1 = -(alpha - beta) # 肘部向上
    
    #print(f"小臂相对于竖直的角度:{(90 - math.degrees(theta1)):.1f}, 大臂相对于竖直的角度:{(90 - math.degrees(theta2)):.1f}")
    print(f"[log]{(90 - math.degrees(theta1)):.1f}, {(90 - math.degrees(theta2)):.1f}")
    return 90 - math.degrees(theta1), 90 - math.degrees(theta2)


# 机械臂动作运行
def execute_arm_sequence(theta1, theta2):
    # 初始化
    set_angle(ARM2_PIN, 30 + (30))
    set_angle(ARM1_PIN, 90 - (15))

    set_angle(ROTATE_PIN, int(90 + object_direction * object_angle + 15))

    # 机械臂运动
    #wiringpi.softPwmCreate(ARM1_PIN, 0, 200) # 周期:20ms
    set_angle(ARM1_PIN, 90 + theta2 + 40 - (15))
    #wiringpi.softPwmCreate(ARM2_PIN, 0, 200) # 周期:20ms
    set_angle(ARM2_PIN, theta1 + 0 + (30))
    #wiringpi.softPwmStop(ARM2_PIN)
    #wiringpi.softPwmStop(ARM1_PIN)
    cv2.waitKey(400)

    # 夹爪抓取
    set_angle(HAND_PIN, 100)

    # 机械臂复位&放下
    set_angle(ARM1_PIN, 90 - (15))
    set_angle(ARM2_PIN, 30 + (30))
    set_angle(ROTATE_PIN, 90 + (15))
    set_angle(ARM1_PIN, 90 + 60 - (15))

    set_angle(HAND_PIN, 30)



while True:
    ret, frame = cap.read()
    if not ret:
        break


    frame = cv2.convertScaleAbs(frame, alpha=1.0, beta=0)  # 曝光

    ###########################底座坐标ArUco检测部分###########################
    corners, ids, rejected = detector.detectMarkers(frame)

    if ids is not None:
        aruco.drawDetectedMarkers(frame, corners, ids)  # ArUco画框框
        
        for corner, id in zip(corners, ids):
            id = id[0]
            pts = corner[0]
            
            # 计算所有ArUco的中心点
            center_x = pts[:, 0].mean()
            center_y = pts[:, 1].mean()
            
            # ArUco画点点
            cv2.circle(frame, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
            
            
            # ArUco id分类
            if id == 1:
                center_x1, center_y1 = center_x, center_y

                # 计算平均边长
                # 点错位排列，计算 AB, BC, CD, DA
                # pts: [A, B, C, D]
                # rolled_pts: [B, C, D, A]
                rolled_pts = np.roll(pts, -1, axis=0)

                # 计算两点间距离
                edge_lengths = np.linalg.norm(pts - rolled_pts, axis=1)

                # 平均边长
                aruco_length_pixel = np.mean(edge_lengths)
                #print(f"[log]平均边长: {aruco_length_pixel:.2f}")
            
            if id == 2:
                center_x2, center_y2 = center_x, center_y
            
            if id == 1 or id == 2:
                # 原点偏移计算
                VEC_SCALE = 4
                vec_origin = VEC_SCALE * (pts[2] - pts[1])
                
                # 原点计算
                temp_origin_x = temp_origin_x[1:]
                temp_origin_x.append((center_x1 + center_x2) / 2 + vec_origin[0])
                temp_origin_y = temp_origin_y[1:]
                temp_origin_y.append((center_y1 + center_y2) / 2 + vec_origin[1])
                origin[0] = sum(temp_origin_x) / len(temp_origin_x)
                origin[1] = sum(temp_origin_y) / len(temp_origin_y)
                cv2.circle(frame, (int(origin[0]), int(origin[1])), 5, (0, 0, 255), -1)
                #print(f"[log]原点坐标({origin[0]:.1f}, {origin[1]:.1f})")
            

            if id == 3:
                object[0], object[1] = center_x, center_y
                #print(f"[log]object坐标({object[0]:.1f}, {object[1]:.1f})")




    ###########################yolo物体检测部分###########################
#    results = model(frame, stream=True)
#    for r in results:
#        boxes = r.boxes
#        for box in boxes:
#            # 获取类别ID
#            cls = int(box.cls[0])
#            conf = float(box.conf[0])
#            
#            # 由类别过滤物体
#            class_name = model.names[cls]
#            
#            if class_name in ['yellow', 'white'] and conf > 0.2:
#                x1, y1, x2, y2 = map(int, box.xyxy[0])
#                
#                # 画框框
#                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
#                cv2.putText(frame, f"{class_name} {conf:.2f}", (x1, y1-10),
#                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow("UI", frame)

    key = cv2.waitKey(1)
    if key == 13:
        # 计算物体相对于原点的信息
        vec_object = object - origin

        # object角度
        _dot_product = np.dot(vec_origin, vec_object)
        _length_origin = np.linalg.norm(vec_origin)
        _length_object = np.linalg.norm(vec_object)
        _cos_theta = _dot_product / (_length_origin * _length_object)
        _cos_theta = np.clip(_cos_theta, -1.0, 1.0)

        object_angle = np.degrees(np.arccos(_cos_theta))
        #print(f"[log]object角度({object_angle})")

        # object距离
        object_pixel = _length_object + 0.5 * np.linalg.norm(vec_origin)
        object_cm = object_pixel * (CM_OF_ARUCO / aruco_length_pixel) - 0
        print(f"[log]object距离({object_cm:.1f})")

        # 角度计算
        _cross_product = vec_origin[0] * vec_object[1] - vec_origin[1] * vec_object[0]
        if _cross_product > 0:
            object_direction = -1
        elif _cross_product < 0:
            object_direction = 1
        else:
            object_direction = 0


        # 计算角度
        if (result := calculate_angles(object_cm, 2.5)) is None:
            continue
        else:
            theta1, theta2 = result

        # 执行机械臂动作
        #arm_thread = threading.Thread(target=execute_arm_sequence, args=(theta1, theta2))
        #arm_thread.daemon = True
        #arm_thread.start()
        execute_arm_sequence(theta1, theta2)


    # ESC退出
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()