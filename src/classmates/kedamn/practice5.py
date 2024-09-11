# import robomaster
from robomaster import robot

# from robomaster import vision
# from robomaster import sensor
import time


# 初始化PID参数
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数
        self.prev_error = 0  # 上一次误差
        self.integral = 0  # 误差积分

    def calculate(self, error):
        self.integral += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


# 全局变量用于存储当前跟踪的标记和当前距离
current_marker = []
current_distance = []


def track_marker(marker_info):
    global current_marker
    print(f"检测信息：{marker_info}")

    if len(marker_info) == 0:
        print("没有检测到标记")
        current_marker = None
        return

    try:
        # 处理第一个检测到的标记
        current_marker = marker_info[0]
        x = current_marker[0]
        y = current_marker[1]
        # width = current_marker[2]
        # height = current_marker[3]
        marker_id = current_marker[4]

        print(f"检测到标记，ID: {marker_id}，坐标: x={x}, y={y}")

    except IndexError as e:
        print(f"索引错误: {e}, marker_info 内容: {marker_info}")
    except TypeError as e:
        print(f"类型错误: {e}, marker_info 内容: {marker_info}")
    except Exception as e:
        print(f"其他错误: {e}, marker_info 内容: {marker_info}")


def distance_callback(distance_info):
    global current_distance
    try:
        # 假设distance_info是一个列表，第一个元素是距离（毫米）
        current_distance = distance_info[0] / 10.0  # 转换为厘米
        print(f"订阅到的距离: {current_distance} cm")
    except IndexError as e:
        print(f"索引错误: {e}, distance_info 内容: {distance_info}")
    except Exception as e:
        print(f"处理距离信息时出错: {e}, distance_info 内容: {distance_info}")


def main():
    global current_marker, current_distance

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_gimbal = ep_robot.gimbal
    ep_chassis = ep_robot.chassis
    ep_vision = ep_robot.vision
    ep_distance_sensor = ep_robot.sensor

    # 初始化PID控制器
    pid_x = PID(kp=1.0, ki=0.1, kd=0.05)
    pid_y = PID(kp=1.0, ki=0.1, kd=0.05)

    # 启动视觉标记检测
    ep_vision.sub_detect_info(name="marker", callback=track_marker)

    # 启动距离传感器订阅
    ep_distance_sensor.sub_distance(freq=50, callback=distance_callback)  # 频率根据需求调整

    # 运行60秒以检测和跟踪标记
    start_time = time.time()

    try:
        while time.time() - start_time < 60:
            if current_marker:
                try:
                    x = current_marker[0]
                    y = current_marker[1]

                    # 使用PID控制云台，使标记保持在视图中心
                    error_x = x - 0.5  # 假设x在0到1之间，中心为0.5
                    error_y = 0.5 - y  # 假设y在0到1之间，中心为0.5

                    gimbal_x_speed = pid_x.calculate(error_x)
                    gimbal_y_speed = pid_y.calculate(error_y)

                    # 调整云台位置
                    ep_gimbal.drive_speed(
                        pitch_speed=-gimbal_y_speed, yaw_speed=gimbal_x_speed
                    )

                    # 使用订阅的距离数据调整底盘位置
                    if current_distance:
                        print(f"当前距离: {current_distance} cm")
                        if current_distance[0] > 50:
                            ep_chassis.drive_speed(x=0.2, y=0, z=0)
                        elif current_distance[0] < 50:
                            ep_chassis.drive_speed(x=-0.2, y=0, z=0)

                except Exception as e:
                    print(f"处理标记时出错: {e}")

            else:
                print("当前没有标记跟踪")

            time.sleep(0.1)  # 添加短暂的延时以避免过于频繁地发起命令

    finally:
        # 停止视觉标记检测和距离传感器订阅
        ep_vision.unsub_detect_info(name="marker")
        ep_distance_sensor.unsub_distance()

        # 关闭机器人连接
        ep_robot.close()


if __name__ == '__main__':
    main()
