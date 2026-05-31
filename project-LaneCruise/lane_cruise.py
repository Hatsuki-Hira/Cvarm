import cv2
import time
import numpy as np



# 初始化摄像头
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)



# 显示fps
class FpsTimer:
    def __init__(self):
        self.start_time = time.time_ns()
        self.end_time = None
        self.frame = 0  # 总帧数
        self.fps = 0

    def count(self):
        self.frame += 1
        self.end_time = time.time_ns()
        total = (self.end_time - self.start_time) / 1_000_000_000
        # 0.5s刷新一次fps
        if total > 0.5:
            # 更新记录
            self.start_time = self.end_time
            self.fps = round(1 / total * self.frame, 1)
            self.frame = 0

fpstimer = FpsTimer()



# 二值化图像
def er_zhi(frame_input):
    # 彩色转灰度图
    img = cv2.cvtColor(frame_input, cv2.COLOR_BGR2GRAY)
    # 高斯模糊
    img = cv2.GaussianBlur(img, (5, 5), 0)
    # OTSU自动阈值二值化
    ret, img = cv2.threshold(
        img,
        0,
        255,
        cv2.THRESH_BINARY + cv2.THRESH_OTSU
    )
    # 黑点降噪（闭运算）
    kernel = np.ones((5,5), np.uint8)
    frame_output = cv2.morphologyEx(
        img,
        cv2.MORPH_CLOSE,
        kernel
    )
    # 白点降噪（开运算）
#    kernel = np.ones((5,5), np.uint8)
#    frame_output = cv2.morphologyEx(
#        img,
#        cv2.MORPH_OPEN,
#        kernel
#    )

    cv2.imshow("二值化", frame_output)

    return frame_output



def tou_shi_bian_huan(frame_input):
    # 未矫正的梯形
    origin = np.float32([
        [200, 300], [440, 300],  # 左上 右上
        [100, 480], [540, 480]  # 左下 右下
    ])

    # 目标已矫正矩形
    converted = np.float32([
        [0, 0], [640, 0],
        [0, 480], [640, 480]
    ])
    # 计算透视矩阵
    matrix = cv2.getPerspectiveTransform(origin, converted)
    # 透视变换
    frame_output = cv2.warpPerspective(
        frame_input,
        matrix,
        (640, 480)
    )
    cv2.imshow("透视矫正", frame_output)

    return frame_output



# 寻找最长白线
def find_white_line(gray, frame):
    center_points_list = []
    for y in range(360, 720, 7):
        line = gray[y]  # 画面行
        max_length = 0
        current_length = 0

        start = 0

        best_left = 0
        best_right = 0

        for x in range(len(line)):
            pixel = line[x]
            if pixel == 255:
                # 新白线的开始端像素
                if current_length == 0:
                    start = x

                current_length += 1

                if current_length > max_length:
                    max_length = current_length

                    best_left = start
                    best_right = x

            else:
                current_length = 0
        # 画边缘
        cv2.circle(frame, (best_left, y), 5, (0, 255, 0), -1)
        cv2.circle(frame, (best_right, y), 5, (0, 255, 0), -1)

        #####记录中点#####
        center_point = int((best_left+best_right)/2)
        cv2.circle(frame, (center_point, y), 5, (0, 0, 255), -1)
        # 只记录下面1/4的点用来拟合直线
        if y < 540:
            continue
        # 第一个点直接加入
        if len(center_points_list) == 0:
            center_points_list.append([center_point, y])
        else:
            last_x = center_points_list[-1][0]
            # 偏差大的点
            if abs(center_point - last_x) < 40:
                center_points_list.append([center_point, y])
        ###############



    #####拟合朝向直线#####
    # 点太少，不拟合
    if len(center_points_list) < 2:
        return
    # 拟合直线
    points = np.array(center_points_list,dtype=np.int32)
    line = cv2.fitLine(points, cv2.DIST_L2, 0, 0.01, 0.01)
    # 求点
    vx, vy, x0, y0 = line
    vx = vx[0]
    vy = vy[0]
    x0 = x0[0]
    y0 = y0[0]
    # 直线(x-x0)/vx = (y-y0)/vy
    near_y = 720
    far_y = 360

    near_x = int(((near_y - y0) * vx / vy) + x0)
    far_x = int(((far_y - y0) * vx / vy) + x0)
    # 所有点的中点
    center_points = np.array(center_points_list)

    center_x = int(np.mean(center_points[:,0]))
    center_y = int(np.mean(center_points[:,1]))
    # 画指向
    cv2.line(frame, (near_x, near_y), (center_x, center_y), (255,0,0), 3)
    cv2.arrowedLine(frame, (center_x, center_y), (far_x, far_y), (255,0,0), 3)
    ###############
    
            
    



while True:
    # 摄像头非正常退出
    ret, frame = cap.read()
    if not ret:
        break
    frame = cv2.convertScaleAbs(frame, alpha=1.0, beta=0)  # 曝光调整
    
    gray = er_zhi(frame)
    find_white_line(gray, frame)

    # UI绘制
    fpstimer.count()
    cv2.putText(frame, f"FPS:{fpstimer.fps}", (10, 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.imshow("UI", frame)

    # ESC退出
    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()