#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/UInt16.h>
#include <serial/serial.h>
#include <vector>
#include <string>
#include <cmath>
#include <mutex>
#include <thread>

// RPLidar A1通信协议常量
#define RPLIDAR_CMD_SYNC_BYTE        0xA5
#define RPLIDAR_CMD_SCAN             0x20
#define RPLIDAR_CMD_FORCE_SCAN       0x21
#define RPLIDAR_CMD_STOP             0x25
#define RPLIDAR_CMD_RESET            0x40
#define RPLIDAR_CMD_GET_INFO         0x50
#define RPLIDAR_CMD_GET_HEALTH       0x52

#define RPLIDAR_ANS_SYNC_BYTE1       0xA5
#define RPLIDAR_ANS_SYNC_BYTE2       0x5A

#define RPLIDAR_RESP_MEAS_SYNCBIT        (0x1<<0)
#define RPLIDAR_RESP_MEAS_QUALITY_SHIFT  2

class RPLidarDriver {
private:
    ros::NodeHandle nh_;
    ros::Publisher scan_pub_;
    ros::Timer timer_;
    
    serial::Serial serial_port_;
    std::string port_name_;
    int baud_rate_;
    std::string frame_id_;
    
    double min_angle_;
    double max_angle_;
    double min_range_;
    double max_range_;
    
    bool inverted_;
    bool angle_compensate_;
    
    std::mutex mutex_;
    std::thread *scan_thread_;
    bool running_;
    
    struct RPLidarMeasurement {
        float angle;
        float distance;
        uint8_t quality;
        bool startBit;
    };

public:
    RPLidarDriver() : 
        port_name_("/dev/ttyUSB0"), 
        baud_rate_(115200),
        frame_id_("laser"),
        min_angle_(-M_PI),
        max_angle_(M_PI),
        min_range_(0.15),
        max_range_(6.0),
        inverted_(false),
        angle_compensate_(true),
        scan_thread_(NULL),
        running_(false) {
        
        // 获取参数
        ros::NodeHandle private_nh("~");
        private_nh.param<std::string>("serial_port", port_name_, "/dev/ttyUSB0");
        private_nh.param<int>("serial_baudrate", baud_rate_, 115200);
        private_nh.param<std::string>("frame_id", frame_id_, "laser");
        private_nh.param<bool>("inverted", inverted_, false);
        private_nh.param<bool>("angle_compensate", angle_compensate_, true);
        private_nh.param<double>("min_angle", min_angle_, -M_PI);
        private_nh.param<double>("max_angle", max_angle_, M_PI);
        private_nh.param<double>("min_range", min_range_, 0.15);
        private_nh.param<double>("max_range", max_range_, 6.0);
        
        // 创建激光雷达数据发布者
        scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 1000);
        
        // 连接至激光雷达
        try {
            ROS_INFO("RPLidar opening serial port: %s", port_name_.c_str());
            serial_port_.setPort(port_name_);
            serial_port_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
            
            if (serial_port_.isOpen()) {
                ROS_INFO("RPLidar serial port opened successfully");
            } else {
                ROS_ERROR("Failed to open serial port");
                ros::shutdown();
                return;
            }
        } catch (serial::IOException& e) {
            ROS_ERROR("Failed to open serial port: %s", e.what());
            ros::shutdown();
            return;
        }
        
        // 重置激光雷达
        resetLidar();
        
        // 检查激光雷达健康状态
        // if (!checkLidarHealth()) {
        //     ROS_ERROR("RPLidar health check failed");
        //     ros::shutdown();
        //     return;
        // }
        
        // 启动扫描线程
        startScan();
    }
    
    ~RPLidarDriver() {
        stop();
        
        if (scan_thread_ != NULL) {
            scan_thread_->join();
            delete scan_thread_;
            scan_thread_ = NULL;
        }
        
        if (serial_port_.isOpen()) {
            serial_port_.close();
            ROS_INFO("RPLidar serial port closed");
        }
    }
    
    bool resetLidar() {
        if (!sendCommand(RPLIDAR_CMD_RESET)) {
            return false;
        }
        
        ros::Duration(0.1).sleep();
        
        // 清空缓冲区
        serial_port_.flush();
        
        return true;
    }
    
    bool checkLidarHealth() {
        if (!sendCommand(RPLIDAR_CMD_GET_HEALTH)) {
            return false;
        }
        
        uint8_t response[7];
        size_t bytes_read = serial_port_.read(response, 7);
        
        if (bytes_read != 7) {
            ROS_ERROR("Invalid health response");
            return false;
        }
        
        if (response[0] != RPLIDAR_ANS_SYNC_BYTE1 || response[1] != RPLIDAR_ANS_SYNC_BYTE2) {
            ROS_ERROR("Invalid health response header");
            return false;
        }
        
        uint8_t status = response[6];
        
        if (status == 0) {
            ROS_INFO("RPLidar health status: Good");
            return true;
        } else if (status == 1) {
            ROS_WARN("RPLidar health status: Warning");
            return true;
        } else {
            ROS_ERROR("RPLidar health status: Error");
            return false;
        }
    }
    
    bool sendCommand(uint8_t cmd) {
        uint8_t command_packet[2] = {RPLIDAR_CMD_SYNC_BYTE, cmd};
        
        try {
            size_t bytes_written = serial_port_.write(command_packet, 2);
            return bytes_written == 2;
        } catch (serial::IOException& e) {
            ROS_ERROR("Failed to send command: %s", e.what());
            return false;
        }
    }
    
    void startScan() {
        if (running_) return;
        
        // 发送扫描命令
        if (!sendCommand(RPLIDAR_CMD_SCAN)) {
            ROS_ERROR("Failed to start scan");
            return;
        }
        
        // 清除扫描响应头部
        uint8_t response[7];
        size_t bytes_read = serial_port_.read(response, 7);
        
        if (bytes_read != 7) {
            ROS_ERROR("Invalid scan response");
            return;
        }
        
        if (response[0] != RPLIDAR_ANS_SYNC_BYTE1 || response[1] != RPLIDAR_ANS_SYNC_BYTE2) {
            ROS_ERROR("Invalid scan response header");
            return;
        }
        
        // 启动扫描线程
        running_ = true;
        scan_thread_ = new std::thread(&RPLidarDriver::scanThread, this);
        
        ROS_INFO("RPLidar scan started");
    }
    
    void stop() {
        running_ = false;
        
        if (scan_thread_ != NULL) {
            scan_thread_->join();
            delete scan_thread_;
            scan_thread_ = NULL;
        }
        
        // 发送停止命令
        sendCommand(RPLIDAR_CMD_STOP);
        
        ROS_INFO("RPLidar scan stopped");
    }
    
    void scanThread() {
        std::vector<RPLidarMeasurement> measurements;
        measurements.reserve(360);
        
        const int SCAN_PACKET_SIZE = 5;
        uint8_t packet_buf[SCAN_PACKET_SIZE];
        
        while (running_ && ros::ok()) {
            measurements.clear();
            
            // 持续读取一圈的扫描数据
            bool got_complete_scan = false;
            
            while (!got_complete_scan && running_ && ros::ok()) {
                try {
                    size_t bytes_read = serial_port_.read(packet_buf, SCAN_PACKET_SIZE);
                    
                    if (bytes_read != SCAN_PACKET_SIZE) {
                        ROS_WARN_THROTTLE(1, "Incomplete scan packet");
                        continue;
                    }
                    
                    RPLidarMeasurement measurement;
                    
                    // 解析测量数据
                    measurement.startBit = (packet_buf[0] & RPLIDAR_RESP_MEAS_SYNCBIT) != 0;
                    measurement.quality = packet_buf[0] >> RPLIDAR_RESP_MEAS_QUALITY_SHIFT;
                    
                    uint16_t angle_q6 = ((packet_buf[1] << 8) | packet_buf[2]) >> 1;
                    uint16_t distance_q2 = ((packet_buf[3] << 8) | packet_buf[4]);
                    
                    measurement.angle = static_cast<float>(angle_q6) / 64.0f * (180.0f / 360.0f) * M_PI;
                    measurement.distance = static_cast<float>(distance_q2) / 4.0f / 1000.0f;  // 转换为米
                    
                    if (measurements.empty() && !measurement.startBit) {
                        // 等待一圈的开始
                        continue;
                    }
                    
                    measurements.push_back(measurement);
                    
                    if (measurement.startBit && measurements.size() > 1) {
                        got_complete_scan = true;
                    }
                    
                } catch (serial::IOException& e) {
                    ROS_ERROR("Serial read error: %s", e.what());
                    break;
                }
            }
            
            // 处理并发布扫描数据
            if (got_complete_scan && !measurements.empty()) {
                publishScan(measurements);
            }
        }
    }
    
    void publishScan(const std::vector<RPLidarMeasurement>& measurements) {
        sensor_msgs::LaserScan scan_msg;
        
        scan_msg.header.stamp = ros::Time::now();
        scan_msg.header.frame_id = frame_id_;
        
        // 扫描参数
        scan_msg.angle_min = min_angle_;
        scan_msg.angle_max = max_angle_;
        scan_msg.angle_increment = 2.0 * M_PI / measurements.size();
        scan_msg.time_increment = 1.0 / 7200;  // RPLidar A1旋转频率10Hz，7200个点/秒
        scan_msg.scan_time = 0.1;  // 每次扫描10Hz = 0.1秒
        scan_msg.range_min = min_range_;
        scan_msg.range_max = max_range_;
        
        // 准备数据
        int ranges_size = std::ceil((max_angle_ - min_angle_) / scan_msg.angle_increment);
        
        scan_msg.ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
        scan_msg.intensities.assign(ranges_size, 0.0);
        
        // 填充数据
        for (const auto& measurement : measurements) {
            float angle = measurement.angle;
            float distance = measurement.distance;
            
            if (inverted_) {
                angle = 2.0 * M_PI - angle;
            }
            
            if (angle > max_angle_ || angle < min_angle_) {
                continue;
            }
            
            int index = std::round((angle - min_angle_) / scan_msg.angle_increment);
            
            if (index >= 0 && index < ranges_size) {
                // 过滤无效的测量值
                if (distance >= min_range_ && distance <= max_range_) {
                    scan_msg.ranges[index] = distance;
                    scan_msg.intensities[index] = measurement.quality;
                }
            }
        }
        
        // 发布扫描数据
        scan_pub_.publish(scan_msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rplidar_node");
    
    try {
        RPLidarDriver driver;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_ERROR("Exception: %s", e.what());
        return 1;
    }
    
    return 0;
} 