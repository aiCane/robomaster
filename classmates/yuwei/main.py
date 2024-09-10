import time
import numpy as np
from robomaster import robot
import cv2

global_distance = None
previous_error = 0
integral = 0


def sub_data_handler(sub_info):
    global global_distance
    global_distance = sub_info[0] / 1000


def process_frame(img, ep_gimbal, center_tolerance, adjusting):
    if img is not None:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red, upper_red)

        lower_red = np.array([160, 100, 100])
        upper_red = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red, upper_red)

        mask = mask1 | mask2

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)
            cx = x + w // 2
            cy = y + h // 2

            img_center_x = img.shape[1] // 2
            img_center_y = img.shape[0] // 2

            x_offset = cx - img_center_x
            y_offset = cy - img_center_y

            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(img, (cx, cy), 5, (255, 0, 0), -1)
            cv2.line(
                img, (img_center_x, 0), (img_center_x, img.shape[0]), (0, 255, 255), 1
            )
            cv2.line(
                img, (0, img_center_y), (img.shape[1], img_center_y), (0, 255, 255), 1
            )

            if abs(x_offset) > center_tolerance or abs(y_offset) > center_tolerance:
                if adjusting:
                    ep_gimbal.move(
                        pitch=-y_offset / 100,
                        yaw=x_offset / 100,
                        pitch_speed=120,
                        yaw_speed=120,
                    ).wait_for_completed()
            else:
                adjusting = False
                ep_gimbal.stop()

        else:
            adjusting = True

    return adjusting


def distance_maintain(ep_chassis):
    global global_distance, previous_error, integral
    distance_defined = 0.5

    if global_distance is not None:
        Kp = 1.0
        Ki = 0.1
        Kd = 0.05
        error = global_distance - distance_defined
        integral += error
        derivative = error - previous_error

        distance_difference = Kp * error + Ki * integral + Kd * derivative
        previous_error = error

        if abs(distance_difference) > 0.1:
            print('moving')
            ep_chassis.move(
                x=distance_difference, y=0, z=0, xy_speed=1
            ).wait_for_completed()


def main():
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_camera = ep_robot.camera
    ep_gimbal = ep_robot.gimbal
    ep_sensor = ep_robot.sensor
    ep_chassis = ep_robot.chassis
    center_tolerance = 0.3

    ep_sensor.sub_distance(callback=sub_data_handler)
    ep_camera.start_video_stream(display=False)

    adjusting = True

    while True:
        ep_robot.set_robot_mode(mode=robot.GIMBAL_LEAD)

        img = ep_camera.read_cv2_image(strategy="newest")
        adjusting = process_frame(img, ep_gimbal, center_tolerance, adjusting)
        distance_maintain(ep_chassis)

        cv2.imshow("Robot", img)
        cv2.waitKey(1)

        time.sleep(0.01)

    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()
    ep_robot.close()


if __name__ == '__main__':
    main()
