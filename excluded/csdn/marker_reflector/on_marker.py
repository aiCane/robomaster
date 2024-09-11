# 导入需要的模块
import pygame
import sys
from pygame.locals import *

from robomaster import *

import cv2


def show_video():
    # 获取机器人第一视角图像帧
    img = ep_robot.camera.read_cv2_image(strategy="newest")
    # 转换图像格式，转换为pygame的surface对象
    # if img.any():
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.transpose(img)  # 行列互换
    img = pygame.surfarray.make_surface(img)
    screen.blit(img, (0, 0))  # 绘制图象


def on_detect_marker(marker_info):
    """智能识别标签的回调函数"""
    global markers
    markers = marker_info


def draw_rect(_aim_marker):
    """给目标标签画正方形"""
    for marker in markers:
        if marker[4] == _aim_marker:
            # 计算左上角点坐标(a, b)、宽(w)、高(h)的像素值
            a = int((marker[0] - marker[2] / 2) * width)
            b = int((marker[1] - marker[3] / 2) * height)
            w = int(marker[2] * width)
            h = int(marker[3] * height)
            pygame.draw.rect(screen, (0, 255, 0), (a, b, w, h), 3)


def marker_tracking(_aim_marker):
    find = False  # 标记是否找到目标标签
    for marker in markers:
        if marker[4] == _aim_marker:
            find = True
            x = marker[0]  # 标签中心点x的比例
            y = marker[1]  # 标签中心点y的比例

            if x < 0.45:
                yaw_speed = -20
            elif x > 0.55:
                yaw_speed = 20
            else:
                yaw_speed = 0
            if y < 0.50:
                pitch_speed = 20
            elif y > 0.6:
                pitch_speed = -20
            else:
                pitch_speed = 0
            # print(yaw_speed,pitch_speed)
            ep_robot.gimbal.drive_speed(
                pitch_speed=pitch_speed, yaw_speed=yaw_speed
            )  # 移动云台到中心点
    if not find:  # 如果没有找到目标标签，就让云台停止移动
        ep_robot.gimbal.drive_speed(pitch_speed=0, yaw_speed=0)


pygame.init()
screen_size = width, height = 1280, 720
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("联合行动")
pygame.display.update()

# 创建机器人对象，连接机器人
ep_robot = robot.Robot()
ep_robot.initialize(conn_type='ap')
ep_robot.set_robot_mode(mode='gimbal_lead')
ep_robot.led.set_led(comp="all", r=0, g=255, b=0)  # 亮绿灯
print("连接机器人成功")

# 打开视频流
ep_robot.camera.start_video_stream(display=False)
pygame.time.wait(100)
ep_robot.led.set_led(comp="all", r=0, g=255, b=0)  # 亮绿灯

# 启动摄像头智能识别
markers = []  # 存储识别到的标签数据
ep_robot.vision.sub_detect_info(name="marker", callback=on_detect_marker)

# 瞄准的标签
aim_marker = '4'


clock = pygame.time.Clock()
while True:
    clock.tick(25)  # 将帧数设置为25帧
    for event in pygame.event.get():
        if event.type == QUIT:
            ep_robot.close()
            pygame.quit()
            sys.exit()

    show_video()
    draw_rect(aim_marker)
    marker_tracking(aim_marker)

    print(markers)

    pygame.display.update()
