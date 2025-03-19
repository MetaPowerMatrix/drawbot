#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>

class MoveDistance {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    
    double target_distance_;
    double target_angle_;
    double start_x_;
    double start_y_;
    double start_th_;
    bool movement_started_;
    bool goal_reached_;
    
    double linear_speed_;
    double angular_speed_;
    
    bool angle_first_;
    bool angle_finished_;
    
public:
    MoveDistance() : movement_started_(false), goal_reached_(false), angle_first_(true), angle_finished_(false) {
        // 默认参数设置
        target_distance_ = 0.0;
        target_angle_ = 0.0;
        linear_speed_ = 0.2;  // 默认线速度
        angular_speed_ = 0.5; // 默认角速度
        
        // 创建发布者和订阅者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        odom_sub_ = nh_.subscribe("odom", 10, &MoveDistance::odomCallback, this);
    }
    
    void setTargets(double target_x, double target_y) {
        // 计算绝对距离和方向角度
        target_distance_ = std::sqrt(target_x * target_x + target_y * target_y);
        target_angle_ = std::atan2(target_y, target_x);
        
        // 根据x坐标正负设置运动方向
        linear_speed_ = (target_x >= 0) ? 0.2 : -0.2;  // 后退时速度为负
        
        // 当需要后退时关闭先转角度逻辑
        angle_first_ = (target_x >= 0);  // 只有前进时需要先转向
        
        ROS_INFO("Set targets: x = %.2f meters, y = %.2f meters", target_x, target_y);
        ROS_INFO("Calculated: distance = %.2f meters, angle = %.2f radians", target_distance_, target_angle_);
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 检查里程计消息的有效性
        if (!msg) {
            ROS_WARN("Received empty odom message");
            return;
        }

        if (!movement_started_) {
            // 记录起始位置和朝向
            start_x_ = msg->pose.pose.position.x;
            start_y_ = msg->pose.pose.position.y;
            start_th_ = tf::getYaw(msg->pose.pose.orientation);
            movement_started_ = true;
            ROS_INFO("Starting movement, initial position: (%.2f, %.2f), heading: %.2f", start_x_, start_y_, start_th_);
        }
        
        // 获取当前绝对朝向
        double current_th = tf::getYaw(msg->pose.pose.orientation);
        // 计算与目标角度的差值（考虑初始朝向）
        double target_relative_angle = std::atan2(target_y, target_x);
        double target_absolute_angle = start_th_ + target_relative_angle;
        double angle_diff = current_th - target_absolute_angle;

        // 标准化角度差到[-pi, pi]
        angle_diff = fmod(angle_diff + M_PI, 2*M_PI) - M_PI;

        if (angle_first_ && !angle_finished_) {
            ROS_DEBUG("Current angle error: %.2f rad", angle_diff);
            
            // 增加最小角速度控制
            if (fabs(angle_diff) < 0.05) {
                angle_finished_ = true;
                ROS_INFO("Rotation completed");
            } else {
                // 根据角度差动态调整角速度
                double speed_factor = std::min(fabs(angle_diff)/0.5, 1.0);
                move(true, angle_diff, speed_factor);
                return;
            }
        }
        
        // 计算已移动的距离
        double dx = msg->pose.pose.position.x - start_x_;
        double dy = msg->pose.pose.position.y - start_y_;
        double distance_moved = std::sqrt(dx*dx + dy*dy);
        
        // 输出调试信息
        ROS_DEBUG("Current position: (%.2f, %.2f), distance moved: %.2f meters",
                 msg->pose.pose.position.x,
                 msg->pose.pose.position.y,
                 distance_moved);
        
        // 检查是否达到目标距离
        if (distance_moved >= target_distance_) {
            stop();
            goal_reached_ = true;
            ROS_INFO("Movement completed: %.2f meters", distance_moved);
        } else if (!goal_reached_) {
            // 继续移动
            move(false, angle_diff);
            ROS_DEBUG("Continuing movement, target: %.2f meters, current: %.2f meters",
                     target_distance_, distance_moved);
        }
    }
    
    void move(bool rotating, double angle_error, double speed_factor=1.0) {
        geometry_msgs::Twist cmd_vel;
        if (rotating) {
            // 优化转向控制：更平滑的角速度控制
            cmd_vel.angular.z = (angle_error < 0 ? 1 : -1) * angular_speed_ * speed_factor;
            cmd_vel.linear.x = 0.0;
            ROS_DEBUG("Rotating with angular speed: %.2f", cmd_vel.angular.z);
        } else {
            cmd_vel.linear.x = linear_speed_;
            cmd_vel.angular.z = 0.0;
            ROS_DEBUG("Moving forward with speed: %.2f", cmd_vel.linear.x);
        }
        cmd_vel_pub_.publish(cmd_vel);
    }
    
    void stop() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
    }
    
    bool isGoalReached() {
        return goal_reached_;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_distance");
    
    if (argc != 3) {
        ROS_ERROR("Usage: %s <target_x(meters)> <target_y(meters)>", argv[0]);
        return 1;
    }
    
    // 解析命令行参数
    double target_x = std::atof(argv[1]);
    double target_y = std::atof(argv[2]);
    
    MoveDistance move_controller;
    move_controller.setTargets(target_x, target_y);
    
    // 等待1秒钟让ROS系统完全初始化
    ros::Duration(1.0).sleep();
    
    ros::Rate rate(10);
    while (ros::ok() && !move_controller.isGoalReached()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}