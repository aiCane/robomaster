"""
status=0: go(), find_light(), find_marker()
status=1:          go() -> deal_line()
status=2:  find_light() -> deal_light()
status=3: find_marker() -> deal_marker()

at any time: get_image()

go():          status: 0 -> 1
find_light():  status: 0 -> 2
find_marker(): status: 0 -> 3

deal_line():   status: 1 -> 0
deal_light():  status: 2 -> 0
deal_marker(): status: 3 -> 0

status: int
"""
from typing import List
from robomaster.robot import Robot
from pid_control import PID
from cv2.typing import MatLike

import cv2 as cv
import numpy as np

# global vals
status:     int = 0
lines:     List = []
markers:   List = []
ep_robot: Robot = Robot()
image:  MatLike = MatLike

chassis_speed_z_pid    = PID()
marker_yaw_speed_pid   = PID()
marker_pitch_speed_pid = PID()

# 回调函数
def on_detect_lines(line_info: List) -> None:
    global lines
    lines = line_info

def get_image():
    global image
    image = ep_robot.camera.read_cv2_image(strategy="newest") # 100%能读取到

def go(precision):
    global status, lines, ep_robot
    if status == 0:
        # simply go
        y = 0.0
        if not lines[0]: # lines = [1, [], [], ...]; not lines = [0]
            for line in lines[1:precision + 1]:
                y += line[0] / precision
            ep_robot.chassis.drive_speed(x=0.2, z=chassis_speed_z_pid.update(y, 0.5))
        else: # no lines at all
            status = 1
            deal_line()

def find_light():
    global status
    if status == 0:
        # finding red light
        if is_red_light():
            status = 2
            deal_light()

def find_marker():
    global status
    if status == 0:
        # finding marker
        if is_aim_marker():
            status = 3
            deal_marker()

def is_red_light() -> bool:
    return detect_traffic_light()

def is_aim_marker() -> bool:
    for marker in markers:
        return marker[4] == '1' or marker[4] == '2' or marker[4] == '3' or marker[4] == '4' or marker[4] == '5'

def deal_line():
    global status, ep_robot, lines
    while True:
        ep_robot.gimbal.moveto(pitch=-15, yaw=-30, pitch_speed=90, yaw_speed=90).wait_for_completed()
        if not lines[0]:
            break
        ep_robot.gimbal.moveto(pitch=-15, yaw=30, pitch_speed=90, yaw_speed=90).wait_for_completed()
        if not lines[0]:
            break
        ep_robot.chassis.move(x=0.1).wait_for_completed()
    status = 0 # when found line

def deal_light():
    global status
    ep_robot.chassis.stop()
    ep_robot.gimbal.stop()
    while True:
        if not is_red_light():
            break
    status = 0 # when red light disappear

def deal_marker():
    global status
    for marker in markers:
        marker_x = marker[0]  # 标签中心点x的比例
        marker_y = marker[1]  # 标签中心点y的比例
        while True:
            ep_robot.gimbal.drive_speed(
                pitch_speed=marker_pitch_speed_pid.update(marker_y, 0.5),
                yaw_speed=marker_yaw_speed_pid.update(marker_x, 0.5)
            )
            if 0.45 < marker_x < 0.55 and 0.45 < marker_y < 0.55:
                cv.imwrite(
                    filename=f"build/markers/marker{marker[4]}",
                    img=image
                )
                break
    status = 0 # when detected and took a photo

# traffic_light_function
def detect_traffic_light() -> bool:
    # 读取图片
    # image = cv.imread(image_path)
    global image
    if image is None:
        print("Error: Unable to load image.")
        return False

    # 转换到HSV色彩空间
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # 定义HSV范围
    # 注意：这些值可能需要根据你的具体图片进行调整
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    lower_green = np.array([36, 25, 25])
    upper_green = np.array([70, 255, 255])

    # 红色掩码
    mask_red = cv.inRange(hsv, lower_red, upper_red)
    # 绿色掩码
    mask_green = cv.inRange(hsv, lower_green, upper_green)

    # 形态学操作，去除噪点
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv.dilate(mask_red, kernel, iterations=1)
    mask_green = cv.dilate(mask_green, kernel, iterations=1)

    # 检测红色或绿色区域
    if cv.countNonZero(mask_red) > cv.countNonZero(mask_green):
        color = 'red'
        print("Red light detected, car stops.")
        # 保存图片
        cv.imwrite(f'detected_{color}_light.jpg', image)
        return True
    else:
        color = 'unknown'
        print("Unknown light color.")
        # 保存图片
        cv.imwrite(f'detected_{color}_light.jpg', image)
        return False

def init_robot():
    ep_robot.initialize(conn_type='ap')
    ep_robot.set_robot_mode(mode='chassis_lead')
    ep_robot.gimbal.moveto(pitch=-15, yaw=0, pitch_speed=90, yaw_speed=90).wait_for_completed()
    ep_robot.vision.sub_detect_info(name='line', color='red', callback=on_detect_lines)

init_robot()
while True:
    get_image()
    go(3)
    find_light()
    find_marker()
