from typing import List
from time import sleep
from robomaster.robot import Robot
from pid_control import PID

# 全局变量
lines: List = [0]
markers: List = []
ep_robot: Robot = Robot()

# 回调函数
def on_detect_lines(line_info: List) -> None:
    global lines
    lines = line_info

def on_detect_markers(marker_info: List) -> None:
    global markers
    markers = marker_info

if __name__ == '__main__':
    ep_robot.initialize(conn_type='ap')
    ep_robot.set_robot_mode(mode='chassis_lead')
    ep_robot.gimbal.moveto(pitch=-15, yaw=0, pitch_speed=90, yaw_speed=90).wait_for_completed()
    ep_robot.vision.sub_detect_info(name='line', color='red', callback=on_detect_lines)

    speed_z_pid = PID(kp=250, ki=0, kd=0)
    marker_x_pid = PID(kp=50, ki=0, kd=0.1)
    marker_y_pid = PID(kp=50, ki=0, kd=0.1)

    flag = False
    precision = 2
    while True:
        y = 0.0
        if lines[0] >= 1:
            for line in lines[1:precision + 1]:
                y += line[0] / precision
            speed_z = speed_z_pid.update(feedback=y, set_point=0.5)
            ep_robot.chassis.drive_speed(x=0.2, z=speed_z)
            sleep(0.05)

        if ep_robot.vision.unsub_detect_info(name='line'):
            ep_robot.vision.sub_detect_info(name='marker', color='red', callback=on_detect_markers)
            sleep(0.02)

        for marker in markers:
            ep_robot.set_robot_mode(mode='free')
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
                    flag = False
        else:
            ep_robot.set_robot_mode(mode='chassis_lead')

        if ep_robot.vision.unsub_detect_info(name='marker'):
            ep_robot.vision.sub_detect_info(name='line', color='red', callback=on_detect_lines)
            sleep(0.02)
