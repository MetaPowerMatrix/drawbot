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

        ROS_INFO("Ackermann controller initialized with parameters: wheelbase=%.3f, max_steering_angle=%.3f, max_speed=%.3f",
                 wheelbase_, max_steering_angle_, max_speed_);

        ackermann_cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(
            "ackermann_cmd", 10);
        cmd_vel_sub_ = nh_.subscribe("cmd_vel", 10,
            &AckermannController::cmdVelCallback, this);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);

        last_time_ = ros::Time::now();
        
        // Add timer to update odometry every 50ms
        update_timer_ = nh_.createTimer(ros::Duration(0.05),
            &AckermannController::timerCallback, this);
        ROS_INFO("Ackermann controller initialization completed");
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        ROS_INFO("Received velocity command: linear=%.3f, angular=%.3f", msg->linear.x, msg->angular.z);

        static ros::Time last_cmd_time = ros::Time::now();
        ros::Time current_cmd_time = ros::Time::now();
        double cmd_interval = (current_cmd_time - last_cmd_time).toSec();
        if (cmd_interval > 0.001) {
            ROS_DEBUG("Command processing interval: %.3f seconds (%.2f Hz)", 
                     cmd_interval, 1.0/cmd_interval);
        }
        last_cmd_time = current_cmd_time;

        // Convert Twist message to Ackermann steering command
        ackermann_msgs::AckermannDriveStamped ackermann_cmd;
        ackermann_cmd.header.stamp = ros::Time::now();
        ackermann_cmd.header.frame_id = "base_link";

        // Limit linear velocity within maximum speed range
        double v = std::min(std::max(msg->linear.x, -max_speed_), max_speed_);
        ROS_DEBUG("Limited linear velocity: %.3f", v);
        
        // Calculate steering angle
        double steering_angle = 0.0;
        if (fabs(v) > 0.001 && fabs(msg->angular.z) > 0.001) {  // Avoid division by zero
            double radius = v / msg->angular.z;
            steering_angle = atan(wheelbase_ / radius);
            ROS_INFO("Steering calculation: velocity=%.3f, angular=%.3f, turn_radius=%.3f, steering_angle=%.3f",
                     v, msg->angular.z, radius, steering_angle);
        } else {
            ROS_INFO("Velocity or angular velocity too small, maintaining straight path: velocity=%.3f, angular=%.3f",
                     v, msg->angular.z);
        }
        
        // Limit steering angle within maximum range
        double original_angle = steering_angle;
        steering_angle = std::min(std::max(steering_angle, -max_steering_angle_),
                                max_steering_angle_);
        if (fabs(original_angle - steering_angle) > 0.001) {
            ROS_WARN("Steering angle exceeds limit: original=%.3f, limited=%.3f",
                     original_angle, steering_angle);
        }

        ackermann_cmd.drive.steering_angle = steering_angle;
        ackermann_cmd.drive.speed = v;

        ackermann_cmd_pub_.publish(ackermann_cmd);
        
        // Save current velocity and steering angle
        last_v_ = v;
        last_steering_angle_ = steering_angle;
    }

    void timerCallback(const ros::TimerEvent&) {
        updateOdometry(last_v_, last_steering_angle_);
    }

    void updateOdometry(double v, double steering_angle) {
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time_).toSec();

        // Update robot pose
        double delta_x = v * cos(th_) * dt;
        double delta_y = v * sin(th_) * dt;
        double delta_th = v * tan(steering_angle) / wheelbase_ * dt;

        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;

        ROS_INFO("Odometry update: dt=%.3f, dx=%.3f, dy=%.3f, dth=%.3f",
                 dt, delta_x, delta_y, delta_th);
        ROS_INFO("Current pose: x=%.3f, y=%.3f, th=%.3f",
                 x_, y_, th_);

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
    }
};

int main(int argc, char** argv) {
    ROS_INFO("Starting Ackermann controller node...");
    ros::init(argc, argv, "ackermann_controller");
    ROS_INFO("ROS node initialization completed, node name: %s", ros::this_node::getName().c_str());

    try {
        ROS_INFO("Creating Ackermann controller instance...");
        AckermannController controller;
        ROS_INFO("Ackermann controller instance created successfully, starting main loop");
        ros::spin();
        ROS_INFO("Ackermann controller node exited normally");
    } catch (const std::exception& e) {
        ROS_ERROR("Ackermann controller error: %s", e.what());
        return 1;
    }

    return 0;
}