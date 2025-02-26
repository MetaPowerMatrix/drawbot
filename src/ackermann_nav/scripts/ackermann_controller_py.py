#!/usr/bin/env python

import sys
# 如果你用的是ROS Noetic，确保安装了python3-rospy
try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError:
    print("Error: ROS Python modules not found. Please source your ROS environment.")
    print("Run: source /opt/ros/noetic/setup.bash (adjust path for your ROS version)")
    sys.exit(1)

import serial
import time

class AckermannController:
    def __init__(self):
        # 初始化ROS节点（不需要依赖rosrun）
        try:
            rospy.init_node('ackermann_controller', anonymous=True)
        except rospy.ROSException as e:
            print(f"Failed to initialize ROS node: {e}")
            sys.exit(1)

        # 串口配置（根据你的控制板调整）
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            sys.exit(1)

        # 订阅/cmd_vel话题
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        # 参数：最大速度和转向角度限制
        self.max_speed = 1.0  # m/s
        self.max_steer = 0.5  # 弧度

        print("Ackermann Controller Initialized")

    def cmd_vel_callback(self, msg):
        # 获取ROS的Twist消息
        linear_speed = msg.linear.x
        angular_vel = msg.angular.z

        # 限制速度和角度
        linear_speed = max(min(linear_speed, self.max_speed), -self.max_speed)
        steer_angle = angular_vel * 0.5  # 简化为比例系数，可调
        steer_angle = max(min(steer_angle, self.max_steer), -self.max_steer)

        # 构造控制板指令
        command = f"{linear_speed:.2f},{steer_angle:.2f}\n"

        # 发送到控制板
        try:
            self.serial_port.write(command.encode('utf-8'))
            print(f"Sent to controller: {command.strip()}")
        except serial.SerialException as e:
            print(f"Serial write error: {e}")

    def run(self):
        # 保持节点运行
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        # 关闭时停止小车
        try:
            self.serial_port.write("0.0,0.0\n".encode('utf-8'))
            self.serial_port.close()
            print("Controller stopped and serial port closed.")
        except serial.SerialException as e:
            print(f"Error during shutdown: {e}")

if __name__ == '__main__':
    # 检查ROS环境是否已加载
    if "ROS_DISTRO" not in os.environ:
        print("ROS environment not detected. Please source your ROS setup:")
        print("e.g., source /opt/ros/noetic/setup.bash")
        sys.exit(1)

    controller = AckermannController()
    controller.run()