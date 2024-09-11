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


pygame.init()
screen_size = width, height = 1280, 720
screen = pygame.display.set_mode(screen_size)
pygame.display.set_caption("标签识别")
bg = pygame.image.load("background.png").convert()
screen.blit(bg, (0, 0))
pygame.display.update()

# 创建机器人对象，连接机器人
ep_robot = robot.Robot()
ep_robot.initialize(conn_type='ap')
ep_robot.led.set_led(comp="all", r=0, g=255, b=0)  # 亮绿灯
print("连接机器人成功")

# 打开视频流
ep_robot.camera.start_video_stream(display=False)
pygame.time.wait(100)
ep_robot.led.set_led(comp="all", r=255, g=255, b=255)  # 亮绿灯

# 启动摄像头智能识别
markers = []  # 存储识别到的标签数据
ep_robot.vision.sub_detect_info(name="marker", callback=on_detect_marker)

clock = pygame.time.Clock()
while True:
    clock.tick(25)  # 将帧数设置为25帧
    for event in pygame.event.get():
        if event.type == QUIT:
            ep_robot.close()
            pygame.quit()
            sys.exit()

    show_video()

    pygame.display.update()

    print(markers)
