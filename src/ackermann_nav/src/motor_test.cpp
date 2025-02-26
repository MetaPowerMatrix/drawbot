#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <iostream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_test");
    ros::NodeHandle nh;
    
    // 创建发布器，发布到与底层电机控制器相同的话题
    ros::Publisher cmd_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>("ackermann_cmd", 10);
    
    // 等待连接建立
    ros::Duration(1.0).sleep();
    
    // 创建一个简单的前进命令
    ackermann_msgs::AckermannDriveStamped cmd;
    cmd.header.stamp = ros::Time::now();
    cmd.header.frame_id = "base_link";
    cmd.drive.speed = 0.5;  // 设置较低的速度进行测试
    cmd.drive.steering_angle = 0.0;  // 直线行驶
    
    ROS_INFO("开始电机测试，发送前进命令...");
    
    // 持续发送命令5秒钟
    ros::Time start_time = ros::Time::now();
    ros::Rate rate(10);  // 10Hz
    
    while (ros::ok() && (ros::Time::now() - start_time).toSec() < 5.0) {
        cmd.header.stamp = ros::Time::now();
        cmd_pub.publish(cmd);
        ROS_INFO("发送命令: 速度=%.2f, 转向角=%.2f", cmd.drive.speed, cmd.drive.steering_angle);
        rate.sleep();
    }
    
    // 发送停止命令
    cmd.drive.speed = 0.0;
    cmd_pub.publish(cmd);
    ROS_INFO("测试完成，发送停止命令");
    
    return 0;
} 