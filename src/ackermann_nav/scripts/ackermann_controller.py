#!/usr/bin/env python

import sys
import os
import struct
try:
    import rospy
    from geometry_msgs.msg import Twist
except ImportError:
    print "Error: ROS Python modules not found. Please source your ROS environment."
    sys.exit(1)

import serial
import time

class AckermannController:
    def __init__(self):
        print "Checking ROS Master..."
        if not self.check_ros_master():
            print "ROS Master not running or unreachable. Please start 'roscore'."
            sys.exit(1)
        print "ROS Master is reachable."

        print "Initializing ROS node..."
        try:
            rospy.init_node('ackermann_controller', anonymous=True, log_level=rospy.DEBUG)
            print "ROS node initialized."
        except rospy.ROSException as e:
            print "ROS init failed: %s" % e
            sys.exit(1)

        # Set baudrate to 115200 as confirmed
        baudrate = 115200
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', baudrate, timeout=0.1)
            print "Serial port opened: /dev/ttyACM0, baudrate %d" % baudrate
        except serial.SerialException as e:
            print "Failed to open serial port: %s" % e
            sys.exit(1)

        self.subscriber = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.max_speed = 1.0  # m/s
        self.max_steer = 0.5  # rad/s
        print "Ackermann Controller Initialized (Model C30D)"

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

    def create_frame(self, linear_x, angular_z):
        # Scale linear and angular values to short range (-32768 to 32767)
        scale = 10000.0  # Scaling factor, adjust if needed
        linear_x_scaled = int(linear_x * scale)
        angular_z_scaled = int(angular_z * scale)

        # Ackermann vehicle: X linear velocity, Y and Z linear velocity as 0
        # Angular velocity mapped to Z axis
        x_linear = linear_x_scaled  # X linear velocity
        y_linear = 0  # Y linear velocity
        z_linear = 0  # Z linear velocity
        x_angular = 0  # X angular velocity
        y_angular = 0  # Y angular velocity
        z_angular = angular_z_scaled  # Z angular velocity

        # Construct 24-byte frame
        frame = bytearray(24)
        frame[0] = 0x7B  # Frame header
        frame[1] = 0  # flag_stop = 0 (normal operation)

        # Pack short data (big-endian)
        struct.pack_into('>h', frame, 2, x_linear)  # X linear velocity
        struct.pack_into('>h', frame, 4, y_linear)  # Y linear velocity
        struct.pack_into('>h', frame, 6, z_linear)  # Z linear velocity
        struct.pack_into('>h', frame, 8, x_angular)  # X angular velocity
        struct.pack_into('>h', frame, 10, y_angular)  # Y angular velocity
        struct.pack_into('>h', frame, 12, z_angular)  # Z angular velocity

        # Frame tail
        struct.pack_into('>h', frame, 14, z_angular)  # Z angular velocity (repeated)
        struct.pack_into('>h', frame, 16, 0)  # Odometer status (assume 0)
        struct.pack_into('>h', frame, 18, 0)  # Odometer status (assume 0)
        frame[21] = self.calculate_checksum(frame)  # Checksum
        frame[23] = 0x7D  # Frame tail

        return frame

    def calculate_checksum(self, data):
        # Simple sum checksum (sum of all bytes modulo 256)
        checksum = 0
        for byte in data[:-3]:  # Exclude checksum and frame tail
            checksum += byte
        return checksum & 0xFF

    def cmd_vel_callback(self, msg):
        # Option 1: Send the specific frame you provided
        specific_frame = bytearray([
            0x7B, 0x00, 0x00, 0x9B, 0x00, 0x00, 0xFF, 0xDF,
            0x00, 0x60, 0x00, 0x0C, 0x40, 0xA8, 0xFF, 0xFD,
            0x00, 0x06, 0x00, 0x1E, 0x5B, 0x87, 0x82, 0x7D
        ])
        
        # Option 2: Generate dynamic frame from /cmd_vel
        linear_speed = msg.linear.x
        angular_vel = msg.angular.z
        linear_speed = max(min(linear_speed, self.max_speed), -self.max_speed)
        angular_vel = max(min(angular_vel, self.max_steer), -self.max_steer)
        dynamic_frame = self.create_frame(linear_speed, angular_vel)

        try:
            # Uncomment one of the following lines to test:
            # Test specific frame
            self.serial_port.write(specific_frame)
            print "Sent specific frame to controller (binary):", ' '.join(['%02x' % b for b in specific_frame])

            # Test dynamic frame (comment out specific frame if testing this)
            # self.serial_port.write(dynamic_frame)
            # print "Sent dynamic frame to controller (binary):", ' '.join(['%02x' % b for b in dynamic_frame])
        except serial.SerialException as e:
            print "Serial write error: %s" % e

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        try:
            if self.serial_port and self.serial_port.is_open:
                stop_frame = self.create_frame(0.0, 0.0)
                self.serial_port.write(stop_frame)
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