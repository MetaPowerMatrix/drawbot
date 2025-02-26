#!/usr/bin/env python

import sys
import os
try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError:
    print("Error: ROS Python modules not found. Please source your ROS environment.")
    sys.exit(1)

import serial
import time

class AckermannController:
    def __init__(self):
        # 检查ROS Master是否可用
        if not self.check_ros_master():
            print("ROS Master not running. Please start 'roscore' first.")
            sys.exit(1)

        # 初始化ROS节点，设置超时
        try:
            print("Initializing ROS node...")
            rospy.init_node('ackermann_controller', anonymous=True)
            print("ROS node initialized.")
        except rospy.ROSException as e:
            print(f"Failed to initialize ROS node: {e}")
            sys.exit(1)

        # 串口配置
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
            sys.exit(1)

        # 订阅/cmd_vel话题
        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

        self.max_speed = 1.0
        self.max_steer = 0.5

        print("Ackermann Controller Initialized")

    def check_ros_master(self):
        # 检查ROS Master是否可用
        import socket
        master_uri = os.getenv('ROS_MASTER_URI', 'http://localhost:11311')
        host, port = master_uri.split('//')[1].split(':')
        port = int(port)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)  # 1秒超时
            sock.connect((host, port))
            sock.close()
            return True
        except (socket.timeout, ConnectionRefusedError):
            return False

    def cmd_vel_callback(self, msg):
        linear_speed = msg.linear.x
        angular_vel = msg.angular.z

        linear_speed = max(min(linear_speed, self.max_speed), -self.max_speed)
        steer_angle = angular_vel * 0.5
        steer_angle = max(min(steer_angle, self.max_steer), -self.max_steer)

        command = f"{linear_speed:.2f},{steer_angle:.2f}\n"
        try:
            self.serial_port.write(command.encode('utf-8'))
            print(f"Sent to controller: {command.strip()}")
        except serial.SerialException as e:
            print(f"Serial write error: {e}")

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        try:
            self.serial_port.write("0.0,0.0\n".encode('utf-8'))
            self.serial_port.close()
            print("Controller stopped and serial port closed.")
        except serial.SerialException as e:
            print(f"Error during shutdown: {e}")

if __name__ == '__main__':
    if "ROS_DISTRO" not in os.environ:
        print("ROS environment not detected. Please source your ROS setup:")
        print("e.g., source /opt/ros/noetic/setup.bash")
        sys.exit(1)

    controller = AckermannController()
    controller.run()