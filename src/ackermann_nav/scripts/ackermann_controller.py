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
        self.frame_id = 21  # Initialize frame ID starting at 21
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
                        # Convert response bytes to integers for %x formatting
                        byte_values = [ord(b) for b in response]  # Python 2: use ord() for str
                        print "Controller async response (raw):", ' '.join(['%02x' % b for b in byte_values])
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

        # Extract flag_stop (byte 1)
        flag_stop = data[1]
        print "Flag stop: %d" % flag_stop

        # Extract velocities (short, big-endian)
        x_linear, = struct.unpack('>h', data[2:4])  # X linear velocity (mm/s)
        y_linear, = struct.unpack('>h', data[4:6])  # Y linear velocity (mm/s)
        z_linear, = struct.unpack('>h', data[6:8])  # Z linear velocity (mm/s)
        x_angular, = struct.unpack('>h', data[8:10])  # X angular velocity (rad/s, scaled by 1000)
        y_angular, = struct.unpack('>h', data[10:12])  # Y angular velocity (rad/s, scaled by 1000)
        z_angular, = struct.unpack('>h', data[12:14])  # Z angular velocity (rad/s, scaled by 1000)

        # Extract accelerations (short, big-endian)
        x_acceleration, = struct.unpack('>h', data[14:16])  # X acceleration (mm/s², scaled by 1000)
        y_acceleration, = struct.unpack('>h', data[16:18])  # Y acceleration (mm/s², scaled by 1000)
        z_acceleration, = struct.unpack('>h', data[18:20])  # Z acceleration (mm/s², scaled by 1000)

        # Scale to physical units
        print "X Linear Velocity: %.4f m/s" % (x_linear / 1000.0)  # Convert mm/s to m/s
        print "Y Linear Velocity: %.4f m/s" % (y_linear / 1000.0)  # Convert mm/s to m/s
        print "Z Linear Velocity: %.4f m/s" % (z_linear / 1000.0)  # Convert mm/s to m/s
        print "X Angular Velocity: %.4f rad/s" % (x_angular / 1000.0)  # Scale down by 1000
        print "Y Angular Velocity: %.4f rad/s" % (y_angular / 1000.0)  # Scale down by 1000
        print "Z Angular Velocity: %.4f rad/s" % (z_angular / 1000.0)  # Scale down by 1000
        print "X Acceleration: %.4f m/s²" % (x_acceleration / 1000.0)  # Convert mm/s² to m/s²
        print "Y Acceleration: %.4f m/s²" % (y_acceleration / 1000.0)  # Convert mm/s² to m/s²
        print "Z Acceleration: %.4f m/s²" % (z_acceleration / 1000.0)  # Convert mm/s² to m/s²

        # Checksum (byte 21, BCC verification)
        checksum = data[21]
        calculated_checksum = self.calculate_bcc(data[:-2])  # Exclude checksum and tail
        print "Received Checksum (BCC): %02x, Calculated Checksum (BCC): %02x" % (checksum, calculated_checksum)

    def calculate_bcc(self, data):
        # BCC (XOR of all bytes in data)
        bcc = 0
        for byte in data:
            bcc ^= byte
        return bcc

    def calculate_checksum(self, data):
        # Simple sum checksum (sum of all bytes modulo 256) - legacy for uplink verification
        checksum = 0
        for byte in data:
            checksum += byte
        return checksum & 0xFF

    def create_frame(self, linear_x, angular_z):
        # Scale linear and angular values to short range (-32768 to 32767)
        # Convert m/s to mm/s and rad/s to scaled rad/s (x1000)
        scale_linear = 1000.0  # Convert m/s to mm/s
        scale_angular = 1000.0  # Scale rad/s by 1000
        linear_x_scaled = int(linear_x * scale_linear)  # mm/s
        angular_z_scaled = int(angular_z * scale_angular)  # rad/s * 1000

        # Ackermann vehicle: X linear velocity, Y linear velocity as 0, Z for angular
        x_linear = linear_x_scaled  # X linear velocity (forward speed, mm/s)
        y_linear = 0  # Y linear velocity (assume 0 for Ackermann, mm/s)
        z_linear = angular_z_scaled  # Z linear velocity (map angular velocity, rad/s * 1000)

        # Construct 9-byte frame
        frame = bytearray(9)
        frame[0] = 0x7B  # Frame header
        # Set frame ID (start at 21, increment to 50, then reset to 0)
        self.frame_id = (self.frame_id + 1) % 50  # Cycle between 0 and 49
        if self.frame_id < 21:
            self.frame_id = 21  # Ensure starts at 21
        frame_id = self.frame_id
        struct.pack_into('>H', frame, 1, frame_id)  # Frame ID (2 bytes, big-endian)

        # Pack short data (big-endian, MSB/LSB for X and Y, MSB for Z)
        struct.pack_into('>h', frame, 3, x_linear)  # X linear velocity (mm/s)
        struct.pack_into('>h', frame, 5, y_linear)  # Y linear velocity (mm/s)
        frame[7] = (z_linear >> 8) & 0xFF  # Z linear velocity (MSB, rad/s * 1000)

        # Calculate BCC (XOR of first 8 bytes)
        bcc = 0
        for i in range(8):
            bcc ^= frame[i]
        frame[8] = bcc  # Set BCC checksum

        return frame

    def cmd_vel_callback(self, msg):
        # Generate dynamic frame from /cmd_vel
        linear_speed = msg.linear.x
        angular_vel = msg.angular.z
        linear_speed = max(min(linear_speed, self.max_speed), -self.max_speed)
        angular_vel = max(min(angular_vel, self.max_steer), -self.max_steer)
        dynamic_frame = self.create_frame(linear_speed, angular_vel)

        try:
            self.serial_port.write(dynamic_frame)
            print "Sent dynamic frame to controller (binary):", ' '.join(['%02x' % b for b in dynamic_frame])  # Use dynamic_frame directly

            # Check and parse response after sending (synchronous)
            time.sleep(0.1)  # Small delay to allow response
            response = self.serial_port.read(24)  # Read 24 bytes for uplink frame
            if response and len(response) == 24:
                print "Controller sync response (raw):", ' '.join(['%02x' % b for b in [ord(c) for c in response]])
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
                        print "Sent stop command to controller (binary):", ' '.join(['%02x' % b for b in stop_frame])  # Use stop_frame directly
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
                    print "Controller sync response after stop (raw):", ' '.join(['%02x' % b for b in [ord(c) for c in response]])
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