from typing import List
from robomaster.robot import Robot
from time import sleep
from threading import Thread
import cv2 as cv

from pid_control import PID

# 全局变量声明
distance: int = 0
distances: List[int] = []
marker: List[float] = []
markers: List[List[float]] = []

# 智能识别标签的回调函数
def on_detect_marker(marker_info: List[List[float]]) -> None:
    global markers
    markers = marker_info

# 机器人检测到距离时的回调函数
def on_detect_distance(distance_info: List[int]) -> None:
    global distances
    distances = distance_info

def move_gimbal(_ep_gimbal, _pid_pitch, _pid_yaw):
    global marker, markers
    while True:
        if markers:
            marker = markers[0]
            p_speed = -_pid_pitch.update(marker[1])
            y_speed = _pid_yaw.update(marker[0])
            _ep_gimbal.drive_speed(
                pitch_speed=p_speed,
                yaw_speed=y_speed
            )

def move_chassis(_pid_speed: PID, _ep_chassis: Robot.chassis) -> None:
    global distance, distances
    if distances:
        distance = distances[0]
        x_speed = _pid_speed.update(distance, 500)
        _ep_chassis.drive_speed(x=x_speed)

# 机器人移动线程
def move_robot(
        _pid_speed,
        _pid_pitch,
        _pid_yaw,
        _ep_chassis,
        _ep_gimbal
):
    global marker, markers
    while True:
        sleep(0.1)
        ep_robot.set_robot_mode(mode='gimbal_lead')
        sleep(0.1)
        if markers:
            marker = markers[0]
            x = marker[0]
            y = marker[1]
            _ep_gimbal.drive_speed(
                pitch_speed=-_pid_pitch.update(y, 0.5),
                yaw_speed=_pid_yaw.update(x, 0.5)
            )
            move_chassis(_pid_speed, _ep_chassis)
        else:
            _ep_gimbal.drive_speed(pitch_speed=0, yaw_speed=0)

# initialize robot
ep_robot = Robot()
ep_robot.initialize(conn_type='ap')
ep_robot.set_robot_mode(mode='gimbal_lead')

ep_robot.sensor.sub_distance(freq=5, callback=on_detect_distance)
ep_robot.vision.sub_detect_info(name="marker", callback=on_detect_marker)

pid_speed = PID(kp=0.0035, ki=0.000000005, kd=0.0001, max_output=0.5)
pid_pitch = PID(kp=25, ki=0, kd=0.1)
pid_yaw = PID(kp=50, ki=0, kd=0.1)

# robot读取一帧视频流帧
ep_robot.camera.start_video_stream(display=False)

if __name__ == '__main__':
    sleep(1)
    move_robot_thread = Thread(
        target=move_robot,
        args=(pid_speed, pid_pitch, pid_yaw, ep_robot.chassis, ep_robot.gimbal,)
    ).start()

    while True:
        image = ep_robot.camera.read_cv2_image(strategy='newest')
        cv.imshow("Robot", image)
        cv.waitKey(1)

        if distances:
            print(f"distance: {distance} mm.")
        sleep(0.15)
