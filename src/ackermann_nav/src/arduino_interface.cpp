#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <serial/serial.h>
#include <string>
#include <sstream>

class ArduinoInterface {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_;
    serial::Serial serial_port_;
    std::string port_;
    int baud_rate_;

public:
    ArduinoInterface() {
        // Get parameters
        nh_.param<std::string>("port", port_, "/dev/ttyUSB0");
        nh_.param<int>("baud_rate", baud_rate_, 115200);
        
        // Subscribe to ackermann_cmd topic
        cmd_sub_ = nh_.subscribe("ackermann_cmd", 10, &ArduinoInterface::cmdCallback, this);
        
        // Initialize serial port
        try {
            serial_port_.setPort(port_);
            serial_port_.setBaudrate(baud_rate_);
            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_port_.setTimeout(timeout);
            serial_port_.open();
            
            ROS_INFO("Connected to Arduino on port %s at %d baud", port_.c_str(), baud_rate_);
        } catch (serial::IOException& e) {
            ROS_ERROR("Failed to connect to Arduino: %s", e.what());
        }
    }
    
    void cmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
        if (!serial_port_.isOpen()) {
            ROS_ERROR("Serial port not open");
            return;
        }
        
        // Format command for Arduino
        std::stringstream cmd;
        cmd << "S" << msg->drive.speed << "A" << msg->drive.steering_angle << "\n";
        
        // Send command to Arduino
        serial_port_.write(cmd.str());
        ROS_INFO("Sent to Arduino: %s", cmd.str().c_str());
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "arduino_interface");
    ArduinoInterface interface;
    ros::spin();
    return 0;
} 