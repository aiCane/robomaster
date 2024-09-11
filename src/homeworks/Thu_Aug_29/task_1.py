from typing import List

from Thu_Aug_29.__init__ import ep_robot, ep_chassis, ep_led


def check_quit(_direction: str) -> bool:
    return "exit" in _direction or "quit" in _direction or "close" in _direction


def run_mode(
    _x: float = 0.0,
    _z: int = 0,
    _comp: str = 'bottom_front',
    _b: int = 255,
    _effect: str = 'on',
) -> None:
    """定义一种移动和亮灯杂糅的运动模式

    :param _x: 移动中 x 方向上的距离 (m)
    :param _z: 移动中 z 方向上的旋转角度 (°)
    :param _comp: 亮灯的位置
    :param _b: rgb 中的 b
    :param _effect: 亮灯的模式，常亮、闪烁等
    :return: None
    """
    chassis_action = ep_chassis.move(x=_x, y=0, z=_z, xy_speed=1.5, z_speed=90)
    ep_led.set_led(comp=_comp, r=255, g=255, b=_b, effect=_effect)
    ep_led.set_led(comp="bottom_front", r=255, g=255, b=255, effect='on')
    chassis_action.wait_for_completed()
    ep_led.set_led(comp='bottom_all', effect='off')


def task_1_main() -> None:
    """主函数

    :return: None
    """
    ep_led.set_led(comp='bottom_all', effect='off')

    while True:
        answer_list: List[str] = input("command: ").split()
        val: str = answer_list[0].lower()
        if check_quit(val):
            break
        if val == "fd":
            run_mode(_x=float(answer_list[1]))
        elif val == "back":
            run_mode(_x=-float(answer_list[1]), _comp='bottom_back')
        elif val == "left":
            run_mode(_z=int(answer_list[1]), _comp='bottom_left', _b=0, _effect='flash')
        elif val == "right":
            run_mode(_z=-int(answer_list[1]), _comp='bottom_right', _b=0, _effect='flash')
        else:
            print("wrong command")


if __name__ == '__main__':
    task_1_main()
    ep_robot.close()
