from typing import List

from Thu_Aug_29.__init__ import ep_chassis, ep_led


def move_robot(_x: float, _z: int) -> None:
    """移动机器人

    :param _x: 前后移动的距离(m)
    :param _z: 左右旋转的角度(°)
    :return: None

    前灯是一直亮的，白灯
    向后后灯亮白灯，左右旋转时对应灯闪烁黄灯
    最后将所有灯熄灭
    """
    chassis_action = ep_chassis.move(x=_x, y=0, z=_z, xy_speed=1.5, z_speed=90)
    light_led(_x, _z)
    chassis_action.wait_for_completed()
    ep_led.set_led(comp='bottom_all', effect='off')
    print("done")


def light_led(_x: float, _z: int) -> None:
    """控制装甲灯

    :param _x:
    :param _z:
    :return: None

    根据前进方向、旋转方向控制后、左、右的装甲灯
    """
    _comp: str = ''
    _b: int = 0
    _effect: str = 'on'
    if _x < 0 and _z == 0:
        _comp = 'bottom_back'
        _b = 255
    elif _x == 0 and _z < 0:
        _comp = 'bottom_right'
        _effect = 'flash'
    elif _x == 0 and _z > 0:
        _comp = 'bottom_left'
        _effect = 'flash'

    ep_led.set_led(comp='bottom_front', r=255, g=255, b=255, effect='on')
    ep_led.set_led(comp=_comp, r=255, g=255, b=_b, effect=_effect)


def check_negative(_direction: str) -> bool:
    return _direction == "back" or _direction == "right"


def check_quit(_direction: str) -> bool:
    return "exit" in _direction or "quit" in _direction or "close" in _direction


def check_x(_direction: str) -> bool:
    return _direction == "fd" or _direction == "back"


def check_z(_direction: str) -> bool:
    return _direction == "left" or _direction == "right"


def main() -> None:
    """第一项任务

    :return: None

    dir 如果是 fd 或 back 中的一个的话，执行 x 方向上 move 的函数；如果是back的话，dis为负
    如果是 left 或 right 中的一个的话，z 方向上的；left 中的 ang 为负
    """
    while True:
        answer_list: List[str] = input("command: ").split()
        direction: str = answer_list[0].lower()
        val: float = 0.0
        if check_quit(direction):
            break
        try:
            val = (
                -float(answer_list[1])
                if check_negative(direction)
                else float(answer_list[1])
            )
        except IndexError:
            print("Out of index")
        if check_x(direction):
            move_robot(_x=val, _z=0)
        elif check_z(direction):
            move_robot(_x=0, _z=int(val))
        else:
            print("Wrong command.")
