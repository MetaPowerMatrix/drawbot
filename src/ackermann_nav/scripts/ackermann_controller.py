#!/usr/bin/env python

import sys
import os
import struct
import signal
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
        self.is_shutting_down = False  # Initialize here to ensure availability
        self.start_monitoring()

        # Register signal handler for Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

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

    def start_monitoring(self):
        def monitor_thread():
            while not self.is_shutting_down:  # Use shutdown flag
                try:
                    response = self.serial_port.read(24)  # Read 24 bytes for uplink frame
                    if response and len(response) == 24 and not self.is_shutting_down:
                        print "Controller async response (raw):", ' '.join(['%02x' % b for b in response])
                        self.parse_uplink_frame(response)
                    else:
                        print "Incomplete or no async response from controller."
                except serial.SerialException as e:
                    if not self.is_shutting_down:  # Only print if not shutting down
                        print "Serial read error (async): %s" % e
                time.sleep(0.1)

        import threading
        self.monitor_thread = threading.Thread(target=monitor_thread)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

    def parse_uplink_frame(self, data):
        # Parse 24-byte uplink frame (0x7B header, 0x7D tail)
        if len(data) != 24 or data[0] != 0x7B or data[23] != 0x7D:
            print "Invalid uplink frame format."
            return

        # Extract flag_stop (byte 1, though you mentioned no stop bit)
        flag_stop = data[1]
        print "Flag (byte 1): %d" % flag_stop

        # Extract velocities (short, big-endian)
        x_linear, = struct.unpack('>h', data[2:4])  # X linear velocity
        y_linear, = struct.unpack('>h', data[4:6])  # Y linear velocity
        z_linear, = struct.unpack('>h', data[6:8])  # Z linear velocity
        x_angular, = struct.unpack('>h', data[8:10])  # X angular velocity
        y_angular, = struct.unpack('>h', data[10:12])  # Y angular velocity
        z_angular, = struct.unpack('>h', data[12:14])  # Z angular velocity (repeated in tail)

        # Scale back to physical units (reverse of scale = 10000.0)
        scale = 10000.0
        print "X Linear Velocity: %.4f m/s" % (x_linear / scale)
        print "Y Linear Velocity: %.4f m/s" % (y_linear / scale)
        print "Z Linear Velocity: %.4f m/s" % (z_linear / scale)
        print "X Angular Velocity: %.4f rad/s" % (x_angular / scale)
        print "Y Angular Velocity: %.4f rad/s" % (y_angular / scale)
        print "Z Angular Velocity: %.4f rad/s" % (z_angular / scale)

        # Extract odometry (bytes 14-19, shorts)
        odom1, = struct.unpack('>h', data[14:16])
        odom2, = struct.unpack('>h', data[16:18])
        print "Odometer 1: %d" % odom1
        print "Odometer 2: %d" % odom2

        # Checksum (byte 21, optional verification)
        checksum = data[21]
        calculated_checksum = self.calculate_checksum(data[:-3])  # Exclude checksum and tail
        print "Received Checksum: %02x, Calculated Checksum: %02x" % (checksum, calculated_checksum)

    def create_frame(self, linear_x, angular_z):
        # Scale linear and angular values to short range (-32768 to 32767)
        scale = 10000.0  # Scaling factor, adjust if needed
        linear_x_scaled = int(linear_x * scale)
        angular_z_scaled = int(angular_z * scale)

        # Ackermann vehicle: X linear velocity, Y linear velocity as 0, Z for angular
        x_linear = linear_x_scaled  # X linear velocity (forward speed)
        y_linear = 0  # Y linear velocity (assume 0 for Ackermann)
        z_linear = angular_z_scaled  # Z linear velocity (map angular velocity)

        # Construct 11-byte frame
        frame = bytearray(11)
        frame[0] = 0x7B  # Frame header
        frame[1] = 0  # Flag byte 1 (always 0, no stop bit)
        frame[2] = 0  # Flag byte 2 (assume 0)
        frame[3] = 0  # Flag byte 3 (assume 0)

        # Pack short data (big-endian, MSB/LSB)
        struct.pack_into('>h', frame, 4, x_linear)  # X linear velocity
        struct.pack_into('>h', frame, 6, y_linear)  # Y linear velocity
        struct.pack_into('>h', frame, 8, z_linear)  # Z linear velocity (angular)

        # Calculate BCC (XOR of first 9 bytes)
        bcc = 0
        for i in range(9):
            bcc ^= frame[i]
        frame[9] = bcc  # Set BCC checksum

        frame[10] = 0x7D  # Frame tail

        return frame

    def calculate_checksum(self, data):
        # Simple sum checksum (sum of all bytes modulo 256) - for uplink verification
        checksum = 0
        for byte in data:
            checksum += byte
        return checksum & 0xFF

    def cmd_vel_callback(self, msg):
        # Option 1: Send the specific frame you provided
        specific_frame = bytearray([
            0x7B, 0x00, 0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x1F, 0x7D
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

            # Check and parse response after sending (synchronous)
            time.sleep(0.1)  # Small delay to allow response
            response = self.serial_port.read(24)  # Read 24 bytes for uplink frame
            if response and len(response) == 24:
                print "Controller sync response (raw):", ' '.join(['%02x' % b for b in response])
                self.parse_uplink_frame(response)
            else:
                print "No sync response or incomplete response from controller."
        except serial.SerialException as e:
            print "Serial write or read error: %s" % e

    def signal_handler(self, signal, frame):
        print "\nReceived Ctrl+C, shutting down..."
        self.shutdown()
        sys.exit(0)

    def run(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            self.shutdown()
        except Exception as e:
            print "Unexpected error: %s" % e
            self.shutdown()

    def shutdown(self):
        if self.is_shutting_down:
            return  # Prevent re-entry
        self.is_shutting_down = True
        try:
            # Unsubscribe from /cmd_vel to stop receiving new commands
            if hasattr(self, 'subscriber') and self.subscriber:
                self.subscriber.unregister()
                print "Unsubscribed from /cmd_vel topic."

            if self.serial_port and self.serial_port.is_open:
                # Send stop command with all velocities = 0, frame[1] = 0, and correct BCC
                stop_frame = self.create_frame(0.0, 0.0)
                for attempt in range(3):  # Retry up to 3 times
                    try:
                        self.serial_port.write(stop_frame)
                        print "Sent stop command to controller (binary):", ' '.join(['%02x' % b for b in stop_frame])
                        time.sleep(0.1)  # Small delay to ensure write completes
                        break
                    except serial.SerialException as e:
                        print "Serial write error (attempt %d): %s" % (attempt + 1, e)
                        if attempt == 2:
                            print "Failed to send stop command after 3 attempts."

                # Check and parse response after sending stop command (synchronous)
                time.sleep(0.1)  # Small delay to allow response
                response = self.serial_port.read(24)  # Read 24 bytes for uplink frame
                if response and len(response) == 24:
                    print "Controller sync response after stop (raw):", ' '.join(['%02x' % b for b in response])
                    self.parse_uplink_frame(response)
                else:
                    print "No sync response or incomplete response from controller after stop."

                self.serial_port.close()
                print "Controller stopped and serial port closed."
        except serial.SerialException as e:
            print "Error during shutdown: %s" % e
        finally:
            self.is_shutting_down = False  # Reset flag

if __name__ == '__main__':
    if "ROS_DISTRO" not in os.environ:
        print "ROS environment not detected. Please source your ROS setup:"
        print "e.g., source /opt/ros/melodic/setup.bash"
        sys.exit(1)

    controller = AckermannController()
    controller.run()