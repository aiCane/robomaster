# import time
# from distutils.file_util import move_file
#
# import robomaster
# from robomaster import conn
# from MyQR import myqr
# from PIL import Image
# from multi_robomaster import multi_robot
#
# multi_robot1 = multi_robot.MultiEP()
# multi_robot2 = multi_robot.MultiEP()
# multi_robot1.initialize()
# multi_robot2.initialize()
#
# multi_robot.number_id_by_sn([0,JKCHC600103QJ],[1,SN2])
# robot_group1 = multi_robots.build_group([0, 1])
#
# class Move_Robot:
#     def move_forward_task(self,robot_group):
#         robot_group.chassis.move(x=1, y=0, z=0, xy_speed=1).wait_for_completed()
#     def move_backward_task(self,robot_group):
#         robot_group.chassis.move(x=-1, y=0, z=0, xy_speed=1).wait_for_completed()
#     def move_left_task(self,robot_group):
#         robot_group.chassis.move(x=0, y=-1, z=0, xy_speed=1).wait_for_completed()
#     def move_right_task(self,robot_group):
#         robot_group.chassis.move(x=0, y=1, z=0, xy_speed=1).wait_for_completed()
#
#
# QRCODE_NAME = "qrcode.png"#wifi
#
# if __name__ == '__main__':
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
#     multi_robots.run([robot_group1, move_forward_task])
#     multi_robots.run([robot_group1, move_backward_task])
