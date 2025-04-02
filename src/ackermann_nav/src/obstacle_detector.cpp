#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <vector>
#include <algorithm>
#include <cmath>

class ObstacleDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    ros::Publisher collision_pub_;
    ros::Publisher collision_info_pub_;
    
    double safety_distance_;    // 安全距离阈值 (米)
    double min_valid_range_;    // 有效范围最小值 (米)
    double max_valid_range_;    // 有效范围最大值 (米)
    int consecutive_readings_;  // 连续检测次数阈值
    bool ignore_rear_;          // 是否忽略后方障碍物
    double front_angle_range_;  // 前方扇区角度范围 (弧度)
    
    int collision_count_;       // 连续检测碰撞计数
    
public:
    ObstacleDetector() : collision_count_(0) {
        // 获取参数
        ros::NodeHandle private_nh("~");
        private_nh.param("safety_distance", safety_distance_, 0.2);
        private_nh.param("min_valid_range", min_valid_range_, 0.05);
        private_nh.param("max_valid_range", max_valid_range_, 5.0);
        private_nh.param("consecutive_readings", consecutive_readings_, 3);
        private_nh.param("ignore_rear", ignore_rear_, true);
        private_nh.param("front_angle_range", front_angle_range_, 3.0);
        
        // 创建发布者和订阅者
        laser_sub_ = nh_.subscribe("scan", 1, &ObstacleDetector::laserCallback, this);
        collision_pub_ = nh_.advertise<std_msgs::Bool>("collision_warning", 10);
        collision_info_pub_ = nh_.advertise<std_msgs::String>("collision_info", 10);
        
        ROS_INFO("Obstacle detector initialized with safety distance: %.2f meters", safety_distance_);
        ROS_INFO("Valid range: %.2f to %.2f meters", min_valid_range_, max_valid_range_);
        ROS_INFO("Consecutive readings threshold: %d", consecutive_readings_);
        ROS_INFO("Ignore rear obstacles: %s", ignore_rear_ ? "true" : "false");
        ROS_INFO("Front sector angle range: %.2f radians", front_angle_range_);
    }
    
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        bool collision_detected = false;
        double min_distance = std::numeric_limits<double>::max();
        int min_distance_index = -1;
        
        // 计算前方扇区的角度范围
        double half_front_range = front_angle_range_ / 2.0;
        
        // 遍历所有激光雷达数据点
        for (size_t i = 0; i < scan->ranges.size(); ++i) {
            double range = scan->ranges[i];
            
            // 检查测量值是否有效
            if (std::isnan(range) || std::isinf(range) || 
                range < min_valid_range_ || range > max_valid_range_) {
                continue;
            }
            
            // 计算当前角度
            double angle = scan->angle_min + i * scan->angle_increment;
            
            // 如果忽略后方，跳过后方的数据点
            if (ignore_rear_) {
                // 将角度标准化到[-PI, PI]
                while (angle > M_PI) angle -= 2.0 * M_PI;
                while (angle < -M_PI) angle += 2.0 * M_PI;
                
                // 检查是否在前方扇区
                if (std::abs(angle) > half_front_range) {
                    continue;
                }
            }
            
            // 更新最小距离
            if (range < min_distance) {
                min_distance = range;
                min_distance_index = i;
            }
            
            // 检查是否小于安全距离
            if (range < safety_distance_) {
                collision_detected = true;
            }
        }
        
        // 处理检测结果
        std_msgs::Bool collision_msg;
        std_msgs::String info_msg;
        
        if (collision_detected) {
            collision_count_++;
            
            // 只有连续多次检测到才发布碰撞消息
            if (collision_count_ >= consecutive_readings_) {
                collision_msg.data = true;
                collision_pub_.publish(collision_msg);
                
                // 计算碰撞点的角度
                double collision_angle = 0.0;
                if (min_distance_index >= 0) {
                    collision_angle = scan->angle_min + min_distance_index * scan->angle_increment;
                }
                
                // 发布详细信息
                std::stringstream ss;
                ss << "Collision warning! Obstacle at " << min_distance << " meters, "
                   << "angle: " << collision_angle << " rad";
                info_msg.data = ss.str();
                collision_info_pub_.publish(info_msg);
                
                ROS_WARN_THROTTLE(1.0, "%s", info_msg.data.c_str());
            }
        } else {
            // 重置连续计数
            collision_count_ = 0;
            
            // 发布安全信息
            collision_msg.data = false;
            collision_pub_.publish(collision_msg);
            
            // 定期发布最近障碍物信息
            ROS_INFO_THROTTLE(3.0, "Nearest obstacle: %.2f meters", min_distance);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "obstacle_detector");
    ObstacleDetector detector;
    ros::spin();
    return 0;
} 