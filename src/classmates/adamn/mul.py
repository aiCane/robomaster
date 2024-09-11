from multi_robomaster import multi_robot


def group_task():
    x = 1
    robot_group.chassis.move(x, 0, 0, 1, 180).wait_for_completed()
    robot_group.chassis.move(-x, 0, 0, 1, 180).wait_for_completed()


robots_sn_list = ['SN1', 'SN2']
multi_robots = multi_robot.MultiEP()
multi_robots.initialize()

robot_group = multi_robots.build_group([0, 1])
multi_robots.run([robot_group, group_task])

multi_robots.close()
