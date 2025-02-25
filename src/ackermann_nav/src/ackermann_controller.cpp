#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

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

public:
    AckermannController() : x_(0.0), y_(0.0), th_(0.0), last_v_(0.0), last_steering_angle_(0.0) {
        nh_.param<double>("wheelbase", wheelbase_, 0.25);
        nh_.param<double>("max_steering_angle", max_steering_angle_, 0.6);
        nh_.param<double>("max_speed", max_speed_, 1.0);

        ackermann_cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
            "ackermann_cmd", 10);
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10,
            &AckermannController::cmdVelCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);

        last_time_ = ros::Time::now();
        
        // 添加定时器，每50ms更新一次里程计
        update_timer_ = nh_.createTimer(ros::Duration(0.05),
            &AckermannController::timerCallback, this);
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        // 将Twist消息转换为Ackermann转向命令
        ackermann_msgs::AckermannDriveStamped ackermann_cmd;
        ackermann_cmd.header.stamp = ros::Time::now();
        ackermann_cmd.header.frame_id = "base_link";

        // 限制线速度在最大速度范围内
        double v = std::min(std::max(msg->linear.x, -max_speed_), max_speed_);
        
        // 计算转向角度
        double steering_angle = 0.0;
        if (fabs(v) > 0.001 && fabs(msg->angular.z) > 0.001) {  // 避免除以零
            double radius = v / msg->angular.z;
            steering_angle = atan(wheelbase_ / radius);
            ROS_DEBUG("计算转向角度：速度=%.2f, 角速度=%.2f, 转向角=%.2f", v, msg->angular.z, steering_angle);
        } else {
            ROS_DEBUG("速度或角速度太小，保持直线行驶");
        }
        
        // 限制转向角度在最大范围内
        steering_angle = std::min(std::max(steering_angle, -max_steering_angle_),
                                max_steering_angle_);

        ackermann_cmd.drive.steering_angle = steering_angle;
        ackermann_cmd.drive.speed = v;

        ackermann_cmd_pub_.publish(ackermann_cmd);
        
        // 保存当前速度和转向角度
        last_v_ = v;
        last_steering_angle_ = steering_angle;
    }

    void timerCallback(const ros::TimerEvent&) {
        updateOdometry(last_v_, last_steering_angle_);
    }

    void updateOdometry(double v, double steering_angle) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();

        // 更新机器人位姿
        double delta_x = v * cos(th_) * dt;
        double delta_y = v * sin(th_) * dt;
        double delta_th = v * tan(steering_angle) / wheelbase_ * dt;

        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;

        // 发布tf变换
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th_);
        odom_broadcaster_.sendTransform(odom_trans);

        // 发布里程计消息
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
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "ackermann_controller2");
    AckermannController controller;
    ros::spin();
    return 0;
}