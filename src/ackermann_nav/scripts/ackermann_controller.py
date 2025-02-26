#!/usr/bin/env python

import sys
import os
try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError:
    print "Error: ROS Python modules not found. Please source your ROS environment."
    sys.exit(1)

import serial
import time
import threading

class AckermannController:
    def __init__(self):
        print "Checking ROS Master..."
        if not self.check_ros_master():
            print "ROS Master not running or unreachable. Please start 'roscore'."
            sys.exit(1)
        print "ROS Master is reachable."

        print "Initializing ROS node..."
        def init_node_with_timeout():
            try:
                rospy.init_node('ackermann_controller', anonymous=True, log_level=rospy.DEBUG)
                rospy.logdebug("Debug: Node initialized")
                print "ROS node initialized."
            except rospy.ROSException as e:
                print "ROS init failed: %s" % e

        init_thread = threading.Thread(target=init_node_with_timeout)
        init_thread.start()
        init_thread.join(timeout=5)
        if init_thread.is_alive():
            print "ROS node initialization timed out."
            sys.exit(1)

        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        except serial.SerialException as e:
            print "Failed to open serial port: %s" % e
            sys.exit(1)

        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.max_speed = 1.0
        self.max_steer = 0.5
        print "Ackermann Controller Initialized"

    def check_ros_master(self):
        import socket
        master_uri = os.getenv('ROS_MASTER_URI', 'http://localhost:11311')
        host, port = master_uri.split('//')[1].split(':')
        port = int(port)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(1)
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
        command = "{:.2f},{:.2f}\n".format(linear_speed, steer_angle)
        try:
            self.serial_port.write(command.encode('utf-8'))
            print "Sent to controller: %s" % command.strip()
        except serial.SerialException as e:
            print "Serial write error: %s" % e

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        try:
            self.serial_port.write("0.0,0.0\n".encode('utf-8'))
            self.serial_port.close()
            print "Controller stopped and serial port closed."
        except serial.SerialException as e:
            print "Error during shutdown: %s" % e

if __name__ == '__main__':
    if "ROS_DISTRO" not in os.environ:
        print "ROS environment not detected. Please source your ROS setup:"
        print "e.g., source /opt/ros/melodic/setup.bash"
        sys.exit(1)

    controller = AckermannController()
    controller.run()