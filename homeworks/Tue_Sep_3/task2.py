from time import sleep
from multi_robomaster import multi_robot

multi_robots = multi_robot.MultiEP()
multi_robots.initialize()
multi_robots.number_id_by_sn([0, '3JKDH7G001V5T2'], [1, '3JKCHC600103E3'])
robot_group = multi_robots.build_group([0, 1])
sleep(3)  # 初始化机器人
while True:
    robot_group.chassis.move(x=1, xy_speed=2).wait_for_completed()
    robot_group.chassis.move(x=-1, xy_speed=2).wait_for_completed()
    sleep(1)
