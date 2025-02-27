#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <XmlRpcValue.h>
#include <string>
#include <cstdio>
#include <thread>
#include <mutex>
#include <atomic>
#include <signal.h>
#include <serial/serial.h>
#include <sstream>
#include <iomanip>
#include <vector>

class AckermannController {
private:
    ros::NodeHandle nh_;
    ros::Publisher ackermann_cmd_pub_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    tf::TransformBroadcaster odom_broadcaster_;

    double wheelbase_;
    double max_steering_angle_;
    double max_speed_;
    double x_, y_, th_;
    ros::Time last_time_;
    ros::Timer update_timer_;
    double last_v_;
    double last_steering_angle_;

    // Log control variables
    int cmd_log_counter_ = 0;
    int odom_log_counter_ = 0;
    const int CMD_LOG_INTERVAL = 50;   // Print command logs every 50 iterations
    const int ODOM_LOG_INTERVAL = 100; // Print odometry logs every 100 iterations

    // Serial communication
    serial::Serial serial_port_;
    std::string port_name_;
    int baud_rate_;
    std::atomic<bool> is_shutting_down_;
    std::thread monitor_thread_;
    std::mutex serial_mutex_;

public:
    AckermannController() : x_(0.0), y_(0.0), th_(0.0), last_v_(0.0), last_steering_angle_(0.0), is_shutting_down_(false) {
        nh_.param<double>("wheelbase", wheelbase_, 0.25);
        nh_.param<double>("max_steering_angle", max_steering_angle_, 0.6);
        nh_.param<double>("max_speed", max_speed_, 1.0);
        nh_.param<std::string>("port", port_name_, "/dev/ttyACM0");
        nh_.param<int>("baud_rate", baud_rate_, 115200);

        ROS_INFO("Ackermann controller initialized with parameters: wheelbase=%.3f, max_steering_angle=%.3f, max_speed=%.3f",
                 wheelbase_, max_steering_angle_, max_speed_);

        // Initialize serial port
        try {
            serial_port_.setPort(port_name_);
            serial_port_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
            
            ROS_INFO("Serial port opened: %s, baudrate %d", port_name_.c_str(), baud_rate_);
        } catch (serial::IOException& e) {
            ROS_ERROR("Failed to open serial port: %s", e.what());
            throw;
        }

        // Start monitoring thread
        startMonitoring();

        ackermann_cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
            "ackermann_cmd", 10);
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10,
            &AckermannController::cmdVelCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);

        last_time_ = ros::Time::now();
        
        // Add timer to update odometry every 50ms
        update_timer_ = nh_.createTimer(ros::Duration(0.05),
            &AckermannController::timerCallback, this);
        ROS_INFO("Ackermann controller initialization completed (Model C30D)");
    }

    ~AckermannController() {
        shutdown();
    }

    void startMonitoring() {
        monitor_thread_ = std::thread([this]() {
            while (!is_shutting_down_) {
                try {
                    std::lock_guard<std::mutex> lock(serial_mutex_);
                    if (serial_port_.isOpen()) {
                        size_t bytes_available = serial_port_.available();
                        if (bytes_available >= 24) {
                            std::vector<uint8_t> response(24);
                            size_t bytes_read = serial_port_.read(response.data(), 24);
                            
                            if (bytes_read == 24 && !is_shutting_down_) {
                                std::stringstream ss;
                                for (size_t i = 0; i < bytes_read; ++i) {
                                    ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(response[i]) << " ";
                                }
                                ROS_INFO("Controller async response (raw): %s", ss.str().c_str());
                                parseUplinkFrame(response);
                            }
                        } else {
                            ROS_DEBUG_THROTTLE(5.0, "Incomplete or no async response from controller.");
                        }
                    }
                } catch (const std::exception& e) {
                    if (!is_shutting_down_) {
                        ROS_ERROR_THROTTLE(1.0, "Serial read error (async): %s", e.what());
                    }
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });
    }

    void parseUplinkFrame(const std::vector<uint8_t>& data) {
        // Parse 24-byte uplink frame (0x7B header, 0x7D tail)
        if (data.size() != 24 || data[0] != 0x7B || data[23] != 0x7D) {
            ROS_ERROR("Invalid uplink frame format.");
            return;
        }

        // Extract flag_stop (byte 1)
        uint8_t flag_stop = data[1];
        ROS_INFO("Flag (byte 1): %d", flag_stop);

        // Extract velocities (short, big-endian)
        int16_t x_linear = (data[2] << 8) | data[3];
        int16_t y_linear = (data[4] << 8) | data[5];
        int16_t z_linear = (data[6] << 8) | data[7];
        int16_t x_angular = (data[8] << 8) | data[9];
        int16_t y_angular = (data[10] << 8) | data[11];
        int16_t z_angular = (data[12] << 8) | data[13];

        // Scale back to physical units (reverse of scale = 10000.0)
        double scale = 10000.0;
        ROS_INFO("X Linear Velocity: %.4f m/s", x_linear / scale);
        ROS_INFO("Y Linear Velocity: %.4f m/s", y_linear / scale);
        ROS_INFO("Z Linear Velocity: %.4f m/s", z_linear / scale);
        ROS_INFO("X Angular Velocity: %.4f rad/s", x_angular / scale);
        ROS_INFO("Y Angular Velocity: %.4f rad/s", y_angular / scale);
        ROS_INFO("Z Angular Velocity: %.4f rad/s", z_angular / scale);

        // Extract odometry (bytes 14-19, shorts)
        int16_t odom1 = (data[14] << 8) | data[15];
        int16_t odom2 = (data[16] << 8) | data[17];
        ROS_INFO("Odometer 1: %d", odom1);
        ROS_INFO("Odometer 2: %d", odom2);

        // Checksum (byte 21, optional verification)
        uint8_t checksum = data[21];
        uint8_t calculated_checksum = calculateChecksum(data, 0, data.size() - 3);  // Exclude checksum and tail
        ROS_INFO("Received Checksum: %02x, Calculated Checksum: %02x", checksum, calculated_checksum);
    }

    std::vector<uint8_t> createFrame(double linear_x, double angular_z) {
        // Scale linear and angular values to short range (-32768 to 32767)
        double scale = 10000.0;  // Scaling factor
        int16_t linear_x_scaled = static_cast<int16_t>(linear_x * scale);
        int16_t angular_z_scaled = static_cast<int16_t>(angular_z * scale);

        // Ackermann vehicle: X linear velocity, Y linear velocity as 0, Z for angular
        int16_t x_linear = linear_x_scaled;  // X linear velocity (forward speed)
        int16_t y_linear = 0;                // Y linear velocity (assume 0 for Ackermann)
        int16_t z_linear = angular_z_scaled; // Z linear velocity (map angular velocity)

        // Construct 12-byte frame (changed from 11 to 12 to include tail)
        std::vector<uint8_t> frame(12);
        frame[0] = 0x7B;  // Frame header
        frame[1] = 0;     // Flag byte 1 (always 0, no stop bit)
        frame[2] = 0;     // Flag byte 2 (assume 0)
        frame[3] = 0;     // Flag byte 3 (assume 0)

        // Pack short data (big-endian, MSB/LSB)
        frame[4] = (x_linear >> 8) & 0xFF;  // X linear velocity MSB
        frame[5] = x_linear & 0xFF;         // X linear velocity LSB
        frame[6] = (y_linear >> 8) & 0xFF;  // Y linear velocity MSB
        frame[7] = y_linear & 0xFF;         // Y linear velocity LSB
        frame[8] = (z_linear >> 8) & 0xFF;  // Z linear velocity MSB
        frame[9] = z_linear & 0xFF;         // Z linear velocity LSB

        // Calculate BCC (XOR of first 10 bytes)
        uint8_t bcc = 0;
        for (int i = 0; i < 10; ++i) {
            bcc ^= frame[i];
        }
        frame[10] = bcc;  // Set BCC checksum

        frame[11] = 0x7D;  // Frame tail

        return frame;
    }

    uint8_t calculateChecksum(const std::vector<uint8_t>& data, size_t start, size_t end) {
        // Simple sum checksum (sum of all bytes modulo 256) - for uplink verification
        uint8_t checksum = 0;
        for (size_t i = start; i < end && i < data.size(); ++i) {
            checksum += data[i];
        }
        return checksum;
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // Increment counter
        cmd_log_counter_++;
        
        // Only print detailed logs at specified intervals
        bool should_log = (cmd_log_counter_ % CMD_LOG_INTERVAL == 0);
        
        if (should_log) {
        ROS_INFO("Received velocity command: linear=%.3f, angular=%.3f", msg->linear.x, msg->angular.z);
        }

        static ros::Time last_cmd_time = ros::Time::now();
        ros::Time current_cmd_time = ros::Time::now();
        double cmd_interval = (current_cmd_time - last_cmd_time).toSec();
        if (cmd_interval > 0.001 && should_log) {
            ROS_DEBUG("Command processing interval: %.3f seconds (%.2f Hz)", 
                     cmd_interval, 1.0/cmd_interval);
        }
        last_cmd_time = current_cmd_time;

        // Convert Twist to Ackermann steering
        double v = msg->linear.x;
        double steering_angle = msg->angular.z;

        // Apply limits
        v = std::max(std::min(v, max_speed_), -max_speed_);
        steering_angle = std::max(std::min(steering_angle, max_steering_angle_), -max_steering_angle_);

        // Store for odometry updates
        last_v_ = v;
        last_steering_angle_ = steering_angle;

        // Create Ackermann message
        ackermann_msgs::AckermannDriveStamped ackermann_cmd;
        ackermann_cmd.header.stamp = ros::Time::now();
        ackermann_cmd.header.frame_id = "base_link";
        ackermann_cmd.drive.speed = v;
        ackermann_cmd.drive.steering_angle = steering_angle;
        ackermann_cmd.drive.steering_angle_velocity = 0.0;
        ackermann_cmd.drive.acceleration = 0.0;
        ackermann_cmd.drive.jerk = 0.0;

        // Publish Ackermann message
        ackermann_cmd_pub_.publish(ackermann_cmd);
        
        // Prevent counter overflow
        if (cmd_log_counter_ > 10000) {
            cmd_log_counter_ = 0;
        }

        ROS_INFO_THROTTLE(1.0, "Publishing Ackermann command: speed=%.3f, steering_angle=%.3f", 
                         ackermann_cmd.drive.speed, ackermann_cmd.drive.steering_angle);

        try {
            std::lock_guard<std::mutex> lock(serial_mutex_);
            
            // Generate dynamic frame from velocity command
            double linear_speed = v;
            double angular_vel = steering_angle;
            std::vector<uint8_t> dynamic_frame = createFrame(linear_speed, angular_vel);

            // Send the frame
            serial_port_.write(dynamic_frame.data(), dynamic_frame.size());
            
            // Format frame for logging
            std::stringstream ss;
            for (size_t i = 0; i < dynamic_frame.size(); ++i) {
                ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(dynamic_frame[i]) << " ";
            }
            ROS_INFO("Sent frame to controller (binary): %s", ss.str().c_str());

            // Wait for response
            ros::Duration(0.1).sleep();
            
            // Read response
            if (serial_port_.available() >= 24) {
                std::vector<uint8_t> response(24);
                size_t bytes_read = serial_port_.read(response.data(), 24);
                
                if (bytes_read == 24) {
                    std::stringstream ss_resp;
                    for (size_t i = 0; i < bytes_read; ++i) {
                        ss_resp << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(response[i]) << " ";
                    }
                    ROS_INFO("Controller sync response (raw): %s", ss_resp.str().c_str());
                    parseUplinkFrame(response);
                } else {
                    ROS_WARN("No sync response or incomplete response from controller.");
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Serial write or read error: %s", e.what());
        }
    }

    void timerCallback(const ros::TimerEvent&) {
        updateOdometry(last_v_, last_steering_angle_);
    }

    void updateOdometry(double v, double steering_angle) {
        // Increment counter
        odom_log_counter_++;
        
        // Only print detailed logs at specified intervals
        bool should_log = (odom_log_counter_ % ODOM_LOG_INTERVAL == 0);
        
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();

        // Update robot pose
        double delta_x = v * cos(th_) * dt;
        double delta_y = v * sin(th_) * dt;
        double delta_th = v * tan(steering_angle) / wheelbase_ * dt;

        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;

        if (should_log) {
        ROS_INFO("Odometry update: dt=%.3f, dx=%.3f, dy=%.3f, dth=%.3f",
                 dt, delta_x, delta_y, delta_th);
        ROS_INFO("Current pose: x=%.3f, y=%.3f, th=%.3f",
                 x_, y_, th_);
        }

        // Publish tf transform
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th_);
        odom_broadcaster_.sendTransform(odom_trans);

        // Publish odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th_);
        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = v * tan(steering_angle) / wheelbase_;
        odom_pub_.publish(odom);

        last_time_ = current_time;
        
        // Prevent counter overflow
        if (odom_log_counter_ > 10000) {
            odom_log_counter_ = 0;
        }
    }

    void shutdown() {
        if (is_shutting_down_) {
            return;  // Prevent re-entry
        }
        is_shutting_down_ = true;
        
        ROS_INFO("Shutting down Ackermann controller...");
        
        try {
            // Unsubscribe from cmd_vel
            cmd_vel_sub_.shutdown();
            ROS_INFO("Unsubscribed from /cmd_vel topic.");
            
            // Send stop command
            if (serial_port_.isOpen()) {
                std::vector<uint8_t> stop_frame = createFrame(0.0, 0.0);
                
                for (int attempt = 0; attempt < 3; ++attempt) {
                    try {
                        std::lock_guard<std::mutex> lock(serial_mutex_);
                        serial_port_.write(stop_frame.data(), stop_frame.size());
                        
                        std::stringstream ss;
                        for (size_t i = 0; i < stop_frame.size(); ++i) {
                            ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(stop_frame[i]) << " ";
                        }
                        ROS_INFO("Sent stop command to controller (binary): %s", ss.str().c_str());
                        
                        ros::Duration(0.1).sleep();
                        break;
                    } catch (const std::exception& e) {
                        ROS_ERROR("Serial write error (attempt %d): %s", attempt + 1, e.what());
                        if (attempt == 2) {
                            ROS_ERROR("Failed to send stop command after 3 attempts.");
                        }
                    }
                }
                
                // Read final response
                ros::Duration(0.1).sleep();
                
                try {
                    std::lock_guard<std::mutex> lock(serial_mutex_);
                    if (serial_port_.available() >= 24) {
                        std::vector<uint8_t> response(24);
                        size_t bytes_read = serial_port_.read(response.data(), 24);
                        
                        if (bytes_read == 24) {
                            std::stringstream ss;
                            for (size_t i = 0; i < bytes_read; ++i) {
                                ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(response[i]) << " ";
                            }
                            ROS_INFO("Controller sync response after stop (raw): %s", ss.str().c_str());
                            parseUplinkFrame(response);
                        } else {
                            ROS_WARN("No sync response or incomplete response from controller after stop.");
                        }
                    }
                    
                    serial_port_.close();
                    ROS_INFO("Controller stopped and serial port closed.");
                } catch (const std::exception& e) {
                    ROS_ERROR("Error during final read: %s", e.what());
                }
            }
        } catch (const std::exception& e) {
            ROS_ERROR("Error during shutdown: %s", e.what());
        }
        
        // Wait for monitoring thread to finish
        if (monitor_thread_.joinable()) {
            monitor_thread_.join();
        }
        
        is_shutting_down_ = false;
    }
};

// Signal handler for Ctrl+C
AckermannController* controller_ptr = nullptr;

void signalHandler(int sig) {
    ROS_INFO("Received signal %d, shutting down...", sig);
    if (controller_ptr) {
        controller_ptr->shutdown();
    }
    ros::shutdown();
}

int main(int argc, char** argv) {
    ROS_INFO("Starting Ackermann controller node...");
    ros::init(argc, argv, "ackermann_controller", ros::init_options::NoSigintHandler);
    ROS_INFO("ROS node initialization completed, node name: %s", ros::this_node::getName().c_str());

    // Set up custom signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    try {
        ROS_INFO("Creating Ackermann controller instance...");
        AckermannController controller;
        controller_ptr = &controller;
        ROS_INFO("Ackermann controller instance created successfully, starting main loop");
        ros::spin();
        ROS_INFO("Ackermann controller node exited normally");
    } catch (const std::exception& e) {
        ROS_ERROR("Ackermann controller error: %s", e.what());
        return 1;
    }

    controller_ptr = nullptr;
    return 0;
}