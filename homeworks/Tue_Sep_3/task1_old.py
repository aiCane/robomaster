from typing import List
from robomaster.robot import Robot
from time import sleep
from threading import Thread
import cv2

from pid_control import IncrementalPID

# 全局变量声明
speed: float = 0.0
distance: int = 0
distances: List[int] = []

# 机器人检测到距离时的回调函数
def on_detect_distance(distance_info: List[int]) -> None:
    global distances
    distances = distance_info

# 机器人移动线程
def move_robot(_pid: IncrementalPID, _ep_chassis: Robot.chassis) -> None:
    global distance, distances, speed
    while True:
        if distances:
            distance = distances[0]
            speed = _pid.update(distance)
            _ep_chassis.drive_speed(x=speed)

def move_gimbal(_ep_gimbal, _pid_pitch, _pid_yaw):
    while True:
        _ep_gimbal.drive_speed(
            pitch_speed=_pid_pitch,
            yaw_speed=_pid_yaw
        )

def print_value() -> None:
    global distances, speed
    while True:
        if distances:
            print(f"distance: {distance} mm; speed: {speed} m/s.")
        else:
            print("none value")
        sleep(0.5)  # 每半秒打印一次

def start_threads() -> None:
    move_robot_thread = Thread(target=move_robot, args=(pid_speed, ep_chassis,))
    print_value_thread = Thread(target=print_value, args=())

    move_robot_thread.start()
    print_value_thread.start()

# initialize robot
ep_robot = Robot()
ep_robot.initialize(conn_type='ap')
ep_robot.set_robot_mode(mode='gimbal_lead')
ep_chassis = ep_robot.chassis
ep_sensor = ep_robot.sensor
ep_camera = ep_robot.camera

ep_sensor.sub_distance(freq=5, callback=on_detect_distance)

pid_speed = IncrementalPID(set_point=500, kp=0.0025, ki=0.000000001, kd=0.0001, max_output=0.5)
pid_pitch = IncrementalPID(set_point=370, kp=0.025, ki=0, kd=0)
pid_yaw = IncrementalPID(set_point=665, kp=0.025, ki=0, kd=0)

# robot读取一帧视频流帧
ep_camera.start_video_stream(display=False)

goal_img = cv2.imread('img.png',0)
goal_ret, goal_thresh = cv2.threshold(goal_img, 127, 255,0)
goal_contours, goal_hierarchy = cv2.findContours(goal_thresh, 2, 1)
goal_cnt = goal_contours[0]


if __name__ == '__main__':
    start_threads()

    while True:
        # 读取图像帧
        image = ep_camera.read_cv2_image(strategy='newest')

        # 预处理图像
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

        cv2.imshow("Robot", thresh)
        cv2.waitKey(1)
        sleep(0.1)

        # 查找轮廓
        contours, _ = cv2.findContours(thresh, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        # 对于每个轮廓，计算中心点
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            center_x, center_y = x + w // 2, y + h // 2

            perimeter = cv2.arcLength(contour, True)  # 计算轮廓周长
            approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)  # 获取轮廓角点坐标
            cornerNum = len(approx)  # 轮廓角点的数量

            ret = cv2.matchShapes(contour, goal_cnt, 1, 0.0)
            if ret < 7e-5:
                print(f"{ret} is square")
                print(f"Center of contour: ({center_x}, {center_y})\n")

# 665, 370