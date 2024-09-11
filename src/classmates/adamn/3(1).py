# import time
# import robomaster
# from asyncio import wait_for
# from distutils.file_util import move_file
#
# from PIL.ImImagePlugin import number
# from multi_robomaster.multi_robot import MultiRobotBase
# from robomaster import conn
# from MyQR import myqr
# from PIL import Image
# from multi_robomaster import multi_robot
#
# # multi_robot1 = multi_robot.MultiEP()
# # multi_robot2 = multi_robot.MultiEP()
# # multi_robot1.initialize()
# # multi_robot2.initialize()
#
# #multi_robots.number_id_by_sn([0,JKCHC600103QJ],[1,SN2])
# #robot_group1 = multi_robots.build_group([0, 1])
#
# # class PID:
# #     def __init__(self,kp,ki,kd,setpoint=0.0):
# #         self.kp=kp
# #         self.ki=ki
# #         self.kd=kd
# #         self.setpoint=setpoint
# #         self.P=0.0
# #         self.I=0.0
# #         self.D=0.0
# #         self.last_error=0.0
# #         self.integral=0.0
# #
# #     def update(self, measured_value):
# #         # """
# #         # 计算PID控制器的输出值
# #         # :param measured_value: 当前测量值
# #         # :return: 控制器的输出
# #         # """
# #         error = self.setpoint - measured_value  # 计算误差
# #         self.integral += error  # 积分项
# #         derivative = error - self.last_error  # 微分项
# #
# #         # 计算P, I, D项
# #         self.P = self.kp * error
# #         self.I = self.ki * self.integral
# #         self.D = self.kd * derivative
# #
# #         # 更新上一次的误差
# #         self.last_error = error
# #
# #         # 计算PID输出
# #         output = self.P + self.I + self.D
# #         return output
# #
# #     # 使用PID控制器
# #
# # pid = PID(0.1, 0.01, 0.05, setpoint=100.0)  # 实例化PID控制器
# # measured_values = [0, 20, 40, 60, 80, 90, 95, 98, 99, 100]
#
# for value in measured_values:
#     control = pid.update(value)
#     print(f"Measured: {value}, Control: {control}")
#
#
# def move_forward_task(self,robot_group):
#     robot_group.chassis.move(x=1, y=0, z=0, xy_speed=1).wait_for_completed()
# def move_backward_task(self,robot_group):
#     robot_group.chassis.move(x=-1, y=0, z=0, xy_speed=1).wait_for_completed()
#
# QRCODE_NAME = "qrcode.png" #wifi
#
# if __name__ == '__main__':
#     robots_sn_list = ['SN1',"SN2"]
#     multi_robots = multi_robot.MultiEP()
#     multi_robots.initialize()
#
#     number = multi_robots.number_id_by_sn([0,SN1],[1,SN2])
#     robot_group = multi_robots.build_group([0,1])
#     robot_group1 = multi_robots.build_group([0])
#     multi_robots.run([robot_group,group_task])
#     multi_robots.run([robot_group1,group_task1])
#
#
#
#     helper = conn.ConnectionHelper()
#     info = helper.build_qrcode_string(ssid="OUC_WIFI_bubu", password="woyaoqian")
#     myqr.run(words=info)
#     time.sleep(1)
#     img = Image.open(QRCODE_NAME)
#     img.show()
#     if helper.wait_for_connection():
#         print("Connected!")
#     else:
#         print("Connect failed!")
#
#
#
#
#     multi_robots.run([robot_group1, move_forward_task])
#     time.sleep(5)
#
#     multi_robots.run([robot_group1, move_backward_task])
#     time.sleep(5)
#     multi_robots.close()
