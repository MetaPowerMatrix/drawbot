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
#include "odom_updater.h"

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

    // 里程计更新器
    OdomUpdater odom_updater_;

public:
    AckermannController() : x_(0.0), y_(0.0), th_(0.0), last_v_(0.0), last_steering_angle_(0.0), is_shutting_down_(false), odom_updater_(nh_) {
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
        ROS_INFO("启动数据流监控线程");
        monitor_thread_ = std::thread([this]() {
            std::vector<uint8_t> buffer;
            const size_t READ_SIZE = 128; // 每次尝试读取的字节数
            buffer.reserve(READ_SIZE);
            
            while (!is_shutting_down_) {
                try {
                    if (serial_port_.isOpen()) {
                        size_t bytes_available = serial_port_.available();
                        
                        // 只有当有数据可读时才进行读取
                        if (bytes_available > 0) {
                            // 每次读取最多READ_SIZE字节的数据
                            size_t bytes_to_read = std::min(bytes_available, READ_SIZE);
                            buffer.resize(bytes_to_read);
                            
                            {
                                std::lock_guard<std::mutex> lock(serial_mutex_);
                                size_t bytes_read = serial_port_.read(buffer.data(), bytes_to_read);
                                buffer.resize(bytes_read); // 调整为实际读取的大小
                            }
                            
                            if (!buffer.empty() && !is_shutting_down_) {
                                // 使用OdomUpdater处理数据流
                                bool processed = odom_updater_.processControllerDataStream(buffer);
                                
                                // 调试日志（每5秒最多输出一次）
                                if (processed) {
                                    ROS_DEBUG_THROTTLE(5.0, "处理了%zu字节的控制器数据流", buffer.size());
                                }
                            }
                        }
                    }
                } catch (const std::exception& e) {
                    if (!is_shutting_down_) {
                        ROS_ERROR_THROTTLE(1.0, "数据流监控线程出错: %s", e.what());
                        
                        // 尝试重新打开串口
                        try {
                            std::lock_guard<std::mutex> lock(serial_mutex_);
                            if (!serial_port_.isOpen()) {
                                ROS_WARN("尝试重新打开串口...");
                                serial_port_.open();
                                if (serial_port_.isOpen()) {
                                    ROS_INFO("串口重新打开成功");
                                }
                            }
                        } catch (const std::exception& e2) {
                            ROS_ERROR("重新打开串口失败: %s", e2.what());
                        }
                    }
                }
                
                // 短暂睡眠以避免CPU占用过高
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            ROS_INFO("数据流监控线程已终止");
        });
    }

    std::vector<uint8_t> createFrame(double linear_x, double angular_z) {
        // Scale linear and angular values to short range (-32768 to 32767)
        double scale = 1000.0;  // Scaling factor
        double linear_scale = 1000.0;  // Scaling factor
        int16_t linear_x_scaled = static_cast<int16_t>(linear_x * linear_scale);
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

        // Pack short data (big-endian, MSB/LSB)
        frame[3] = (x_linear >> 8) & 0xFF;  // X linear velocity MSB
        frame[4] = x_linear & 0xFF;         // X linear velocity LSB
        frame[5] = (y_linear >> 8) & 0xFF;  // Y linear velocity MSB
        frame[6] = y_linear & 0xFF;         // Y linear velocity LSB
        frame[7] = (z_linear >> 8) & 0xFF;  // Z linear velocity MSB
        frame[8] = z_linear & 0xFF;         // Z linear velocity LSB

        // Calculate BCC (XOR of first 10 bytes)
        uint8_t bcc = 0;
        for (int i = 0; i < 10; ++i) {
            bcc ^= frame[i];
        }
        frame[9] = bcc;  // Set BCC checksum

        frame[10] = 0x7D;  // Frame tail

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
        } catch (const std::exception& e) {
            ROS_ERROR("Serial write or read error: %s", e.what());
        }
    }

    void timerCallback(const ros::TimerEvent&) {
        // 使用OdomUpdater更新里程计
        odom_updater_.update(last_v_, last_steering_angle_);
    }

    // 新增方法：获取当前位姿
    void getCurrentPose(double& x, double& y, double& th) {
        odom_updater_.getPose(x, y, th);
    }
    
    // 新增方法：获取当前速度
    void getCurrentVelocity(double& vx, double& vy, double& omega) {
        odom_updater_.getVelocity(vx, vy, omega);
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
                            // 处理最后一帧数据
                            odom_updater_.processControllerDataStream(response);
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