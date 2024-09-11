from typing import List
from time import sleep
from robomaster.robot import Robot
from pid_control import PID

import cv2 as cv
import numpy as np

from threading import Thread

# 全局变量
lines: List = [0]
markers: List = []
ep_robot: Robot = Robot()
if_go: bool = True
if_line: bool = False
if_light: bool = False
if_marker: bool = False

# 回调函数
def on_detect_lines(line_info: List) -> None:
    global lines
    lines = line_info

def on_detect_markers(marker_info: List) -> None:
    global markers
    markers = marker_info

# traffic_light_function
def detect_traffic_light(image_path):
    # 读取图片
    image = cv.imread(image_path)
    if image is None:
        print("Error: Unable to load image.")
        return

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
    elif cv.countNonZero(mask_green) > 0:
        color = 'green'
        print("Green light detected, car moves forward.")
    else:
        color = 'unknown'
        print("Unknown light color.")

        # 保存图片
    cv.imwrite(f'detected_{color}_light.jpg', image)

def a():
    while True:
        if noline:
            if_go = False
        
        #dealing noline
    if_go = True
    pass
def b():

    pass
def c():
    pass

if __name__ == '__main__':
    ep_robot.initialize(conn_type='ap')
    ep_robot.set_robot_mode(mode='chassis_lead')
    ep_robot.gimbal.moveto(pitch=-15, yaw=0, pitch_speed=90, yaw_speed=90).wait_for_completed()
    ep_robot.vision.sub_detect_info(name='line', color='red', callback=on_detect_lines)

    speed_z_pid = PID(kp=250, ki=0, kd=0)
    marker_x_pid = PID(kp=50, ki=0, kd=0.1)
    marker_y_pid = PID(kp=50, ki=0, kd=0.1)

    detect_traffic_light('build/traffic_lights/traffic_light_image.jpg')

    thread_find_line = Thread() # if_go == False, finished, if_go == True
    thread_find_traffic_light = Thread()
    thread_find_markers = Thread()

    flag = False
    precision = 2
    while True:
        if_go: bool = False
        if_line: bool = False
        if_light: bool = False
        if_marker: bool = False

        image = ep_robot.camera.read_cv2_image(strategy='newest') # 拍照

        while if_go:

            # go
            Thread().start()
            Thread().start()
            Thread().start()

            # if if_line:
            #     pass
            # elif if_light:
            #     pass
            # elif if_marker:
            #     pass
            pass

        y = 0.0
        if not lines[0]:
            for line in lines[1:precision + 1]:
                y += line[0] / precision
            speed_z = speed_z_pid.update(feedback=y, set_point=0.5)
            ep_robot.chassis.drive_speed(x=0.2, z=speed_z)
            sleep(0.05)

        if ep_robot.vision.unsub_detect_info(name='line'):
            ep_robot.vision.sub_detect_info(name='marker', callback=on_detect_markers)
            sleep(0.01)

        for marker in markers:
            ep_robot.set_robot_mode(mode='free')
            print(
                f"{markers}\n"
                f"{marker}\n"
                f"{marker[4]}\n"
            )
            if marker[4] == '1' or marker[4] == '2' or marker[4] == '3' or marker[4] == '4' or marker[4] == '5':
                flag = True
            while flag:
                print(f"marker: {marker}")
                marker_x = marker[0]
                marker_y = marker[1]
                yaw_speed = marker_x_pid.update(feedback=marker_x, set_point=0.5)
                pitch_speed = marker_y_pid.update(feedback=marker_y, set_point=0.5)
                ep_robot.gimbal.drive_speed(
                    pitch_speed=pitch_speed, yaw_speed=yaw_speed
                )
                sleep(0.5)
                if 0.45 < marker_x < 0.55 and 0.45 < marker_y < 0.55:
                    cv.imwrite(
                        filename=f"build/markers/marker{marker[4]}",
                        img=ep_robot.camera.read_cv2_image(strategy='newest')
                    )
                    flag = False
        else:
            ep_robot.set_robot_mode(mode='chassis_lead')
            ep_robot.gimbal.moveto(pitch=-15, yaw=0, pitch_speed=90, yaw_speed=90).wait_for_completed()

        if ep_robot.vision.unsub_detect_info(name='marker'):
            ep_robot.vision.sub_detect_info(name='line', color='red', callback=on_detect_lines)
            sleep(0.01)
