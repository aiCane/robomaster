# 导入需要用到的库
import cv2

# import numpy as np
# import wave
# import re
# import socket
# import sys
from robomaster import robot

from Thu_Aug_29 import ep_gimbal

# 创建新的对象
ep_robot = robot.Robot()
ep_camera = ep_robot.camera
ep_gimbal = ep_robot.gimbal

# 指定连接方式为AP 直连模式，初始化
ep_robot.initialize(conn_type='ap')

# 开始获取视频流，但是不播放
ep_camera.start_video_stream(display=False)

# 设置模式为自由模式
ep_robot.set_robot_mode(mode=robot.FREE)

# 获取官方提供的特征库，根据自己电脑设置路径
face_cascade = cv2.CascadeClassifier(
    "D://python38//Lib//site-packages//cv2//data//haarcascade_frontalface_default.xml"
)
eye_cascade = cv2.CascadeClassifier(
    "D://python38//Lib//site-packages//cv2//data//haarcascade_eye.xml"
)
KP = 0.15  # 比例系数，让云台转的慢一点


def shi_bei():
    global RLzhongxi_x, RLzhongxi_y  # 定义两个全局变量，用来储存人脸的坐标
    cv2.namedWindow("img", 1)  # 新建一个显示窗口
    cv2.resizeWindow("img", 800, 400)  # 图像框的大小
    img = ep_camera.read_cv2_image()  # 获取视频流的一帧图像
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 将读取到的值转化为灰度图
    faces = face_cascade.detectMultiScale(
        gray, 1.1, 5
    )  # 检测出图片中所有的人脸，并将人脸的各个坐标保持到faces里
    if len(faces) > 0:  # 判断是否检测到人脸
        for faceRect in faces:  # 依次读取faces的值
            x, y, w, h = faceRect
            cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 3)  # 绘制人脸框
            roi_gray = gray[y : y + h // 2, x : x + w]
            roi_color = img[y : y + h // 2, x : x + w]
            RLzhongxi_x = x + w / 2  # 获取人脸中心坐标
            RLzhongxi_y = y + h / 2
            eyes = eye_cascade.detectMultiScale(
                roi_gray, 1.1, 1, cv2.CASCADE_SCALE_IMAGE, (2, 2)
            )  # 检测眼睛
            for ex, ey, ew, eh in eyes:
                cv2.rectangle(
                    roi_color, (ex, ey), (ex + ew, ey + eh), (0, 255, 0), 2
                )  # 绘制眼睛框
    cv2.imshow("img", img)  # 显示图像
    cv2.waitKey(2)  # 每两帧的间隔时间

    return RLzhongxi_x, RLzhongxi_y  # 返回人脸中心坐标


def error():
    RLzhongxi_x_1 = centre[0]  # 将上面的值传到这里
    RLzhongxi_y_1 = centre[1]
    TXzhongxi_x = 400  # 整个图像的中心坐标
    TXzhongxi_y = 200
    error_x = TXzhongxi_x - RLzhongxi_x_1  # 偏航轴的的误差
    error_y = TXzhongxi_y - RLzhongxi_y_1  # 俯仰轴的误差
    if abs(error_x) < 10:  # 判断误差是否小于10，如果小于默认为到达人脸中心
        yaw0_speed = 0
    else:
        yaw0_speed = KP * error_x  # 输出偏航轴的速度
    if abs(error_y) < 10:
        pitch0_speed = 0
    else:
        pitch0_speed = -KP * error_y  # 输出俯仰轴的速度
    print("yaw的速度：", yaw0_speed)
    print("Pitch的速度：", pitch0_speed)
    ep_gimbal.drive_speed(
        pitch_speed='{pitch}',
        yaw_speed='{yaw}'.format(pitch=pitch0_speed, yaw=yaw0_speed),
    )  # 控制云台


while True:
    shi_bei()
    centre = shi_bei()
    error()
