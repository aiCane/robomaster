from typing import List
from time import sleep, time

from robomaster.robot import Robot
from pid_control import PID

import cv2 as cv
import numpy as np

# global vals
status:     int = 0
count_line: int = 0

# current_marker_info: str = ''

lines:       List = []
markers:     List = []
aim_markers: List = ['1', '2', '3', '4', '5']

ep_robot: Robot = Robot()

chassis_speed_z_pid    = PID(kp=150, ki=0,    kd=0)
marker_yaw_speed_pid   = PID(kp=50,  ki=1e-3, kd=0)
marker_pitch_speed_pid = PID(kp=25,  ki=1e-3, kd=0)

def on_detect_line(line_info: List) -> None:
    global lines
    lines = [] if line_info == [0] else line_info
    # print(f"lines: {lines}")

def on_detect_marker(marker_info: List) -> None:
    global markers # , current_marker_info
    if marker_info:
        # print(f"marker_info: {marker_info}")
        if marker_info[0] in [0, 1, 2]:
            markers = []
        else:
            markers = marker_info
    if markers:
        if markers[0] in [0, 1, 2, 3, 4]:
            markers = []

    # if markers:
    #     print(f"markers[0]: {markers[0]}")
    #     current_marker_info = markers[0][4]
    #
    #     for marker in markers:
    #         if marker[4] != current_marker_info:
    #             markers.remove(marker)
    #             # print(marker)

def go(precision=3, max_count=10):
    global status, count_line, ep_robot
    if status == 0:
        # simply go
        ep_robot.vision.sub_detect_info(name='line', color='green', callback=on_detect_line)
        y = 0.0
        if lines and lines != [0]: # lines = [1, [], [], ...]; not lines = [0]
            number = lines[0] - 1
            for line in lines[ 1+10*number : 10*number+precision+1 ]:
                y += line[0] / precision
            ep_robot.chassis.drive_speed(x=0.25, z=chassis_speed_z_pid.update(y, 0.5))
            count_line = 0
        else: # no lines at all
            count_line += 1
        if count_line >= max_count:
            status = 1
            deal_line()
            count_line = 0
        ep_robot.vision.unsub_detect_info(name='line')

def find_light():
    global status
    if status == 0:
        # finding red light
        if detect_traffic_light():
            print("red light stop!")
            status = 2
            deal_light()

def find_marker():
    global status
    if status == 0:
        # finding marker
        ep_robot.vision.sub_detect_info(name='marker', callback=on_detect_marker)
        sleep(0.05) # change if (TypeError: 'int' object is not subscriptable) or (Index out of range)
        if is_aim_marker():
            print("marker!")
            status = 3
            deal_marker()
        ep_robot.vision.unsub_detect_info(name='marker')

def is_aim_marker() -> bool:
    # global current_marker_info
    for marker in markers:
        return marker[4] in aim_markers and ( 0.1 < marker[2] or 0.2 < marker[3] )

def deal_line():
    global status, lines, ep_robot
    ep_robot.chassis.drive_speed() # stop 先
    print("finding line...")
    while True:
        ep_robot.gimbal.moveto(pitch=-10, yaw=90).wait_for_completed()
        if lines:
            ep_robot.gimbal.move(pitch=10).wait_for_completed()
            ep_robot.gimbal.moveto(pitch=-15, yaw=0, pitch_speed=90, yaw_speed=60).wait_for_completed()
            ep_robot.chassis.move(z=-60).wait_for_completed()
            break
        ep_robot.gimbal.moveto(pitch=-10, yaw=-90).wait_for_completed()
        if lines:
            ep_robot.gimbal.move(pitch=10).wait_for_completed()
            ep_robot.gimbal.moveto(pitch=-15, yaw=0, pitch_speed=90, yaw_speed=60).wait_for_completed()
            ep_robot.chassis.move(z=60).wait_for_completed()
            break
        ep_robot.gimbal.moveto(pitch=-15, yaw=0, pitch_speed=90, yaw_speed=60).wait_for_completed()
        ep_robot.chassis.move(x=-0.1).wait_for_completed()
        print("...")
    ep_robot.gimbal.moveto(pitch=-15, yaw=0, pitch_speed=90, yaw_speed=60).wait_for_completed()
    print("founded")
    status = 0 # when found line

def deal_light():
    global status
    ep_robot.chassis.drive_speed()
    counter = 0
    while True:
        if counter >= 15:
            break
        if detect_traffic_light():
            counter = 0
        else:
            counter += 1
    cv.imwrite(
        filename=f"../../../build/traffic_lights/detected_red_light_{time()}.jpg",
        img=ep_robot.camera.read_cv2_image(strategy="newest")
    )
    status = 0 # when red light disappear

def deal_marker():
    global status, aim_markers
    ep_robot.chassis.drive_speed()
    ep_robot.set_robot_mode(mode='free')
    print("tracking...")
    # marker_x:  float = 0.5
    # marker_y:  float = 0.5
    # marker_info: str = ''
    count = 0
    dealing_markers = []
    if markers:
        dealing_markers: List[str] = [marker[4] for marker in markers]
    while True:
        if markers:
            if markers[0][4] == dealing_markers[0]:
            # if markers[0][4] == current_marker_info:
                current_marker = markers[0]
            else:
                current_marker = markers[1]
            marker_x    = current_marker[0]  # 标签中心点x的比例
            marker_y    = current_marker[1]  # 标签中心点y的比例
            marker_info = current_marker[4]
            print( [marker_x, marker_y, marker_info] )
            if 0.499 < marker_x < 0.501 and 0.535 < marker_y < 0.545:
                print("got you!")

                sleep(0.5)
                image = ep_robot.camera.read_cv2_image(strategy="newest")
                texted_image = draw_text_at_center(
                    image=image,
                    text=f"Team 04 detected a marker with ID of {marker_info}"
                )
                squared_image = square(texted_image)
                cut_image = crop_center(squared_image)
                cv.imwrite(
                        filename=f"../../../build/markers/marker_{marker_info}.jpg",
                        img=cut_image
                )

                ep_robot.blaster.fire(fire_type='ir')
                aim_markers.remove(markers[0][4])
                dealing_markers.remove(marker_info)
            else:
                ep_robot.gimbal.drive_speed(
                    pitch_speed=-marker_pitch_speed_pid.update(marker_y, 0.54),
                    yaw_speed=marker_yaw_speed_pid.update(marker_x, 0.5)
                )
                sleep(0.1)
            if not current_marker:
                break
        else:
            count += 1
        if count >= 10:
            break
    ep_robot.gimbal.moveto(pitch=-15, yaw=0).wait_for_completed()
    ep_robot.set_robot_mode(mode='chassis_lead')
    sleep(0.1)
    status = 0 # when detected and took a photo

def square(image):
    # 转换到HSV色彩空间
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # 定义红色的HSV范围
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([15, 255, 255])

    # 创建红色掩码
    mask = cv.inRange(hsv, lower_red, upper_red)

    # 形态学操作，去除噪点
    kernel = np.ones((5, 5), np.uint8)
    mask = cv.dilate(mask, kernel, iterations=1)
    mask = cv.erode(mask, kernel, iterations=1)

    # 查找轮廓
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)

    # 绘制矩形框
    for contour in contours:
        if cv.contourArea(contour) > 2500:  # 过滤小轮廓
            x, y, w, h = cv.boundingRect(contour)
            cv.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            break
    return image

def draw_text_at_center(
        image,
        text,
        font=cv.FONT_HERSHEY_SIMPLEX,
        font_scale=0.8,
        color=(255, 255, 255),
        thickness=2
):
    """
    在图像中心绘制文本。

    参数:
    - image: 要绘制的图像。
    - text: 要绘制的文本。
    - font: 字体类型。
    - font_scale: 字体缩放比例。
    - color: 文本的颜色，默认为白色 (255, 255, 255)。
    - thickness: 文本的厚度。
    """
    # 获取图像的尺寸
    height, width = image.shape[:2]

    # 计算文本的宽度和高度
    text_size = cv.getTextSize(text, font, font_scale, thickness)[0]

    # 计算文本在图像中的位置（中心）
    text_x = (width - text_size[0]) // 2
    text_y = (height + text_size[1]) // 2

    # 在图像上绘制文本
    cv.putText(image, text, (text_x, text_y), font, font_scale, color, thickness, lineType=cv.LINE_AA)

    return image

# traffic_light_function <- provided by
def detect_traffic_light() -> bool:
    # 读取图片
    image = ep_robot.camera.read_cv2_image(strategy="newest")

    if image is None:
        print("Error: Unable to load image.")
        return False

    # 转换到HSV色彩空间
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)

    # 定义HSV范围
    # 注意：这些值可能需要根据你的具体图片进行调整
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([15, 255, 255])

    # 红色掩码
    mask_red = cv.inRange(hsv, lower_red, upper_red)

    # 形态学操作，去除噪点
    kernel = np.ones((5, 5), np.uint8)
    mask_red = cv.dilate(mask_red, kernel, iterations=1)

    # 检测红色区域
    contours, _ = cv.findContours(mask_red, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if cv.contourArea(contour) > 2500:  # 假设交通灯区域的最小面积

            # 轮廓近似
            epsilon = 0.02 * cv.arcLength(contour, True)
            approx = cv.approxPolyDP(contour, epsilon, True)

            # 检测近似轮廓的顶点数是否为圆形（近似为大于4个顶点）
            if 6 <= len(approx) <= 8:
                # print(f"{len(approx)}, almost there")
                # 计算轮廓的周长
                perimeter = cv.arcLength(contour, True)
                # 计算轮廓的面积
                area = cv.contourArea(contour)
                # 计算形状的圆度
                circularity = 4 * np.pi * (area / (perimeter * perimeter))

                # 如果圆度足够高，则认为是圆形
                if circularity > 0.8:  # 这个阈值可能需要根据实际情况调整
                    return True
    return False

def crop_center(image, fraction_width=0.8, fraction_height=0.3):
    # 获取图像的尺寸
    height, width = image.shape[:2]

    # 计算裁剪区域的宽度和高度
    crop_width = int(width * fraction_width)
    crop_height = int(height * fraction_height)

    # 计算裁剪区域的起始点（中心点）
    start_x = (width - crop_width) // 2
    start_y = (height - crop_height) // 2

    # 裁剪图像
    cropped_image = image[start_y:start_y + crop_height, start_x:start_x + crop_width]

    return cropped_image

def init_robot():
    ep_robot.initialize(conn_type='ap')
    ep_robot.set_robot_mode(mode='chassis_lead')
    ep_robot.gimbal.moveto(pitch=-15, yaw=0, pitch_speed=90, yaw_speed=90).wait_for_completed()
    ep_robot.camera.start_video_stream(display=False)

def main():
    while True:
        go()
        find_light()
        find_marker()

if __name__ == "__main__":
    init_robot()
    main()
