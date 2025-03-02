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
    
    void setTargets(double distance, double angle, double speed) {
        target_distance_ = distance;
        target_angle_ = angle;
        linear_speed_ = speed;
        ROS_INFO("Set targets: distance = %.2f meters, angle = %.2f radians, speed = %.2f m/s", distance, angle, speed);
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
        
        // 获取当前朝向
        double current_th = tf::getYaw(msg->pose.pose.orientation);
        double angle_diff = current_th - start_th_;
        
        // 标准化角度差到[-pi, pi]
        while (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2 * M_PI;
        
        if (angle_first_ && !angle_finished_) {
            // 先完成转向
            if (fabs(angle_diff - target_angle_) < 0.05) {
                angle_finished_ = true;
                ROS_INFO("Rotation completed: %.2f radians", angle_diff);
            } else {
                // 继续转向
                move(true, angle_diff);
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
    
    void move(bool rotating, double current_angle_diff) {
        geometry_msgs::Twist cmd_vel;
        if (rotating) {
            // 计算剩余需要转向的角度
            double remaining_angle = target_angle_ - current_angle_diff;
            // 根据剩余角度确定转向方向和速度
            cmd_vel.angular.z = (remaining_angle > 0 ? 1 : -1) * angular_speed_;
            cmd_vel.linear.x = 0.0;
        } else {
            cmd_vel.linear.x = linear_speed_;
            cmd_vel.angular.z = 0.0;
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
    
    if (argc != 4) {
        ROS_ERROR("Usage: %s <distance(meters)> <angle(radians)> <speed(m/s)>", argv[0]);
        return 1;
    }
    
    // 解析命令行参数
    double distance = std::atof(argv[1]);
    double angle = std::atof(argv[2]);
    double speed = std::atof(argv[3]);
    
    // 验证速度参数
    if (speed <= 0) {
        ROS_ERROR("Speed must be greater than 0");
        return 1;
    }
    
    MoveDistance move_controller;
    move_controller.setTargets(distance, angle, speed);
    
    // 等待1秒钟让ROS系统完全初始化
    ros::Duration(1.0).sleep();
    
    ros::Rate rate(10);
    while (ros::ok() && !move_controller.isGoalReached()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}