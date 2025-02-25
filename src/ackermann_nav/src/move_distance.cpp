#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <cmath>

class MoveDistance {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    
    double target_distance_;
    double start_x_;
    double start_y_;
    bool movement_started_;
    bool goal_reached_;
    
    double linear_speed_;
    
public:
    MoveDistance() : movement_started_(false), goal_reached_(false) {
        // 设置目标距离为20厘米
        target_distance_ = 0.20;
        // 设置线速度为0.2米/秒
        linear_speed_ = 0.2;
        
        // 创建发布者和订阅者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        odom_sub_ = nh_.subscribe("odom", 10, &MoveDistance::odomCallback, this);
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        if (!movement_started_) {
            // 记录起始位置
            start_x_ = msg->pose.pose.position.x;
            start_y_ = msg->pose.pose.position.y;
            movement_started_ = true;
        }
        
        // 计算已移动的距离
        double dx = msg->pose.pose.position.x - start_x_;
        double dy = msg->pose.pose.position.y - start_y_;
        double distance_moved = std::sqrt(dx*dx + dy*dy);
        
        // 检查是否达到目标距离
        if (distance_moved >= target_distance_) {
            stop();
            goal_reached_ = true;
            ROS_INFO("目标距离已达到: %.2f 米", distance_moved);
        } else if (!goal_reached_) {
            // 继续移动
            move();
        }
    }
    
    void move() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = linear_speed_;
        cmd_vel.angular.z = 0.0;
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
    MoveDistance move_controller;
    
    // 等待1秒钟让ROS系统完全初始化
    ros::Duration(1.0).sleep();
    
    ros::Rate rate(10);
    while (ros::ok() && !move_controller.isGoalReached()) {
        ros::spinOnce();
        rate.sleep();
    }
    
    return 0;
}