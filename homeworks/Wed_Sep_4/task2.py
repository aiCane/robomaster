from typing import List
from time import time
from robomaster.robot import Robot

from pid_control import PID

# 全局变量
lines: List = [0]
ep_robot: Robot = Robot()
last_time: float = time()


# 回调函数
def on_detect_lines(line_info: List) -> None:
    global lines
    lines = line_info


if __name__ == '__main__':

    ep_robot.initialize(conn_type='ap')
    ep_robot.set_robot_mode(mode='chassis_lead')
    ep_robot.gimbal.moveto(pitch=-25, yaw=0).wait_for_completed()
    ep_robot.vision.sub_detect_info(name='line', color='red', callback=on_detect_lines)

    x_pid_pre = PID(kp=1, ki=0, kd=0)
    x_pid = PID(kp=1.75, ki=0, kd=0.5)
    y_pid = PID(kp=100, ki=0, kd=10)
    z_pid_y = PID(kp=175, ki=5, kd=25)
    z_pid_theta = PID(kp=1.25, ki=0, kd=0)

    precision = 3
    base_speed = 275

    while True:
        this_time = time()
        y = theta = 0.0
        if lines[0] >= 1 and this_time > last_time + 0.05:
            last_time = this_time
            for line in lines[1:precision + 1]:
                y += line[0] / precision
                theta += line[2] / precision

            x_wheels_pre = base_speed - abs( x_pid_pre.update(feedback=theta, set_point=0) )
            x_wheels = base_speed - abs( x_pid.update(feedback=x_wheels_pre, set_point=base_speed) )
            y_wheels = y_pid.update(feedback=y, set_point=0.5)
            z_wheels_y = z_pid_y.update(feedback=y, set_point=0.5)
            z_wheels_theta = z_pid_theta.update(feedback=theta, set_point=0)
            z_wheels = z_wheels_y + z_wheels_theta

            print(
                f" x_wheels_pre: {round(x_wheels_pre, 3)};"
                f" x_wheels: {round(x_wheels, 3)};"
                f" y_wheels: {round(y_wheels, 3)};"
                f" z_wheels_y: {round(z_wheels_y, 3)};"
                f" z_wheels_theta: {round(z_wheels_theta, 3)};"
            )
            ep_robot.chassis.drive_wheels(
                w1=(x_wheels - y_wheels - z_wheels),
                w2=(x_wheels + y_wheels + z_wheels),
                w3=(x_wheels - y_wheels + z_wheels),
                w4=(x_wheels + y_wheels - z_wheels)
            )
