#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_test");
    ros::NodeHandle nh;
    
    // Create publisher to the same topic as the motor controller
    ros::Publisher cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 10);
    
    // Wait for connections to establish
    ros::Duration(1.0).sleep();
    
    // Create a simple forward command
    ackermann_msgs::AckermannDriveStamped cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "base_link";
    cmd.drive.speed = 0.5;  // Set a low speed for testing
    cmd.drive.steering_angle = 0.0;  // Drive straight
    
    ROS_INFO("Starting motor test, sending forward command...");
    
    // Send commands for 5 seconds
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10);  // 10Hz
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.0) {
        cmd.header.stamp = ros::Time::now();
        cmd_pub.publish(cmd);
        ROS_INFO("Sending command: speed=%.2f, steering_angle=%.2f", cmd.drive.speed, cmd.drive.steering_angle);
        rate.sleep();
    }
    
    // Send stop command
    cmd.drive.speed = 0.0;
    cmd_pub.publish(cmd);
    ROS_INFO("Test completed, sending stop command");
    
    return 0;
} 