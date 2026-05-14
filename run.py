import cv2
import cv2.aruco as aruco
from ultralytics import YOLO
import time
import numpy as np
import math

import wiringop



# 初始化摄像头
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# 设置字典和参数
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# 实例化 ArucoDetector 对象
detector = aruco.ArucoDetector(dictionary, parameters)

# 初始化yolo
#model = YOLO("yolo11n.pt")

# 原点坐标
center_x1, center_y1 = 0, 0
center_x2, center_y2 = 0, 0
center_x3, center_y3 = 0, 0
center_x4, center_y4 = 0, 0

# ArUco码边长
CM_OF_ARUCO = 1.5
aruco_length_pixel = 0

# soft pwm
ARM1_PIN=1
ARM2_PIN=1
ROTATE_PIN=1
HAND_PIN=1
wiringpi.wiringPiSetup()
wiringpi.pinMode(ARM1_PIN, OUTPUT)
wiringpi.softPwmCreate(ARM1_PIN, 0, 200) # 周期:200ms



def set_angle(pin, angle):
    angle = max(0, min(180, angle))
    agnle_ms = 20 * (angle / 180) + 5
    wiringpi.softPwmWrite(pin, angle_ms)



def calculate_angles(x, y):  # 输入目标坐标
    L1 = 10
    L2 = 10
    D_sq = x**2 + y**2
    D = math.sqrt(D_sq)
    
    # 距离检查
    if D > (L1 + L2) or D < abs(L1 - L2):
        print("[error]目标超出机械臂工作范围")
        return None

    # 计算 theta2 (小臂相对于大臂的角度)
    cos_theta2 = (D_sq - L1**2 - L2**2) / (2 * L1 * L2)
    # 限制范围防止精度溢出
    cos_theta2 = max(-1, min(1, cos_theta2))
    theta2 = math.acos(cos_theta2)

    # 计算 theta1 (大臂相对于基座的角度)
    alpha = math.atan2(y, x)
    
    cos_beta = (L1**2 + D_sq - L2**2) / (2 * L1 * D)
    cos_beta = max(-1, min(1, cos_beta))
    beta = math.acos(cos_beta)
    
    theta1 = alpha - beta # 肘部向上模式
    
    return math.degrees(theta1), math.degrees(theta2)



while True:
    ret, frame = cap.read()
    if not ret:
        break

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
                # 将点错位排列，方便一次性计算 AB, BC, CD, DA
                # pts:     [A, B, C, D]
                # rolled:  [B, C, D, A]
                rolled_pts = np.roll(pts, -1, axis=0)

                # np.linalg.norm 计算欧几里得范数，即两点间距离
                edge_lengths = np.linalg.norm(pts - rolled_pts, axis=1)

                # 平均边长
                aruco_length_pixel = np.mean(edge_lengths)
                print(f"[log]平均边长: {aruco_length_pixel:.2f}")
                
            if id == 2:
                center_x2, center_y2 = center_x, center_y
            
            if id == 3:
                center_x3, center_y3 = center_x, center_y

            if id == 4:
                center_x4, center_y4 = center_x, center_y
            
            # 原点偏移计算
            VEC_SCALE = 2
            vec = VEC_SCALE * (pts[2] - pts[1])
            
            # 原点计算
            origin_x = (center_x1 + center_x2) / 2 + vec[0]
            origin_y = (center_y1 + center_y2) / 2 + vec[1]
            cv2.circle(frame, (int(origin_x), int(origin_y)), 5, (255, 0, 0), -1)
            print(f"[log]原点坐标({origin_x}, {origin_y})")



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

    if cv2.waitKey(1) == 13:
        set_angle(ROTATE_PIN, 0)
        cv2.waitKey(500)
        distance_id3 = math.sqrt((center_x3-center_x)**2+(center_y3-center_y)**2)
        



    cv2.imshow("UI", frame)

    # ESC退出
    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()