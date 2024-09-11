from sys import exit
from time import sleep

from Thu_Aug_29.__init__ import ep_robot, ep_chassis


def task_2_main() -> None:
    """第二项任务

    :return: None

    def dodge():
        while True:
            chassis_ctrl.set_wheel_speed(-120,120,180,-180)
            time.sleep(1.2)
            chassis_ctrl.move_with_time(0,0.2)
            chassis_ctrl.set_wheel_speed(120,-120,-180,180)
            time.sleep(1.2)
            chassis_ctrl.move_with_time(0,0.2)
    """
    for i in range(2):
        flag = (-1) ** i
        ep_chassis.drive_wheels(
            w1=120 * flag, w2=-120 * flag, w3=180 * flag, w4=-180 * flag
        )
        sleep(1.2)
        ep_chassis.stop()
        ep_chassis.drive_speed(x=3.5)
        sleep(0.2)
        ep_chassis.stop()
    ep_chassis.stop()
    exit()


if __name__ == '__main__':
    task_2_main()
    ep_robot.close()
