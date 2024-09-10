from Thu_Aug_29.__init__ import ep_robot, ep_gimbal


def test_usr() -> bool:
    return int(input("answer test: 1 + 1 = ")) == 2


def task_3_main(times: int) -> None:
    """第三项任务

    :param times: 转头的次数
    :return: None

    PS: 代码写得真丑
    """
    ep_robot.set_robot_mode(mode='free')
    _pitch_1: int = 0
    _pitch_2: int = 0
    _yaw_1: int = 0
    _yaw_2: int = 0
    if test_usr():
        _pitch_1 = -25
        _pitch_2 = 30
    else:
        _yaw_1 = -45
        _yaw_2 = 45

    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=60, yaw_speed=90).wait_for_completed()
    for i in range(times):
        ep_gimbal.moveto(
            pitch=_pitch_1, yaw=_yaw_1, pitch_speed=60, yaw_speed=90
        ).wait_for_completed()
        ep_gimbal.moveto(
            pitch=_pitch_2, yaw=_yaw_2, pitch_speed=60, yaw_speed=90
        ).wait_for_completed()
    ep_gimbal.moveto(pitch=0, yaw=0, pitch_speed=60, yaw_speed=90).wait_for_completed()


if __name__ == '__main__':
    task_3_main(3)
    ep_robot.close()
