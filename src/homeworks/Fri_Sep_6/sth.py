from typing import List
from robomaster.robot import Robot

import cv2 as cv
import numpy as np

# global vals
status:       int = 0
count_line:   int = 0
lines:       List = []
markers:     List = []
aim_markers: List = ['1', '2', '3', '4', '5']
ep_robot:   Robot = Robot()

def draw_square(image, x, y, w, h, color=(0, 255, 255), thickness=5):
    """
    在图像上绘制正方形。

    参数:
    - image: 要绘制的图像。
    - center: 正方形的中心点坐标 (x, y)。
    - side_length: 正方形的边长。
    - color: 正方形的颜色，默认为绿色 (0, 255, 0)。
    - thickness: 线条的厚度，默认为2。
    """
    # 计算正方形的左上角和右下角坐标
    top_left     = ( int(x - w / 2), int(y - h / 2) )
    bottom_right = ( int(x + w / 2), int(y + h / 2) )

    # 绘制正方形
    cv.rectangle(image, top_left, bottom_right, color, thickness)

    return image


def init_robot():
    ep_robot.initialize(conn_type='ap')
    ep_robot.set_robot_mode(mode='chassis_lead')
    ep_robot.gimbal.moveto(pitch=-15, yaw=0, pitch_speed=90, yaw_speed=90).wait_for_completed()
    ep_robot.camera.start_video_stream(display=False)

image_2 = "../../../build/markers/marker_2"
image_5 = "../../../build/markers/marker_5"


image_2 = 255 * np.array(image_2).astype('uint8')
# image_2 = np.asarray(image_2)
image_5 = np.asarray(image_5)

image_2 = draw_square(image_2,100, 100, 100, 100)
image_5 = draw_square(image_5,100, 100, 100, 100)
