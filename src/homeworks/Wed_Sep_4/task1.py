from robomaster import robot
import cv2
from robomaster import led

global_signal: bool = False


def pid(k, set_c, get_c):
    return k * (set_c - get_c)


def handle_mouse(event, x, y, flags, param):
    global global_signal, ep_chassis, ep_led
    if event == cv2.EVENT_LBUTTONDOWN:
        global_signal = True
    if event == cv2.EVENT_LBUTTONUP:
        global_signal = False
    if global_signal:
        target_x = 320
        if abs(target_x - x) < 11:
            speed_z = 0
        else:
            speed_z = -pid(0.1, target_x, x)
            if (target_x - x) > 0:
                ep_led.set_led(comp=led.COMP_BOTTOM_LEFT, r=255, g=255, b=0, effect=led.EFFECT_ON)
                ep_led.set_led(comp=led.COMP_ALL, r=255, g=255, b=255, effect=led.EFFECT_OFF)
            elif (target_x - x) < 0:
                ep_led.set_led(comp=led.COMP_BOTTOM_RIGHT, r=255, g=255, b=0, effect=led.EFFECT_ON)
                ep_led.set_led(comp=led.COMP_ALL, r=255, g=255, b=255, effect=led.EFFECT_OFF)

        target_y = 180
        if abs(target_y - y) < 11:
            speed_x = 0
        else:
            speed_x = pid(0.005, target_y, y)
            if speed_x < 0:
                ep_led.set_led(comp=led.COMP_BOTTOM_BACK, r=255, g=0, b=0, effect=led.EFFECT_ON)
                ep_led.set_led(comp=led.COMP_ALL, r=255, g=255, b=255, effect=led.EFFECT_OFF)

        ep_chassis.drive_speed(x=speed_x, y=0, z=speed_z)
    else:
        ep_chassis.drive_speed().wait_for_completed()


if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type='ap')
    ep_robot.gimbal.recenter().wait_for_completed()
    ep_robot.set_robot_mode(robot.CHASSIS_LEAD)
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_led = ep_robot.led
    ep_led.set_led(comp=led.COMP_ALL, r=255, g=255, b=255, effect=led.EFFECT_OFF)

    # 视频流
    ep_camera.start_video_stream(display=False, resolution="360p")
    cv2.namedWindow("EP")
    cv2.setMouseCallback("EP", handle_mouse)

    while True:
        frame = ep_camera.read_cv2_image(strategy="newest")
        frame = cv2.flip(frame, 1)
        cv2.imshow("EP", frame)
        c = cv2.waitKey(10)
        # ESC结束视频
        if c == 27:
            print("视频结束")
            break

    ep_camera.stop_video_stream()
    cv2.destroyAllWindows()
