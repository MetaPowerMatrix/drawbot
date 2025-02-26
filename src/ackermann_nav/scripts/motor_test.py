#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
import time

def main():
    rospy.init_node('motor_test_py')
    
    # 创建发布器
    cmd_pub = rospy.Publisher('ackermann_cmd', AckermannDriveStamped, queue_size=10)
    
    # 等待连接建立
    rospy.sleep(1.0)
    
    # 创建命令
    cmd = AckermannDriveStamped()
    cmd.header.frame_id = "base_link"
    cmd.drive.speed = 0.5
    cmd.drive.steering_angle = 0.0
    
    rospy.loginfo("开始电机测试，发送前进命令...")
    
    # 持续发送命令5秒钟
    start_time = rospy.Time.now()
    rate = rospy.Rate(10)  # 10Hz
    
    while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < 5.0:
        cmd.header.stamp = rospy.Time.now()
        cmd_pub.publish(cmd)
        rospy.loginfo("发送命令: 速度=%.2f, 转向角=%.2f", cmd.drive.speed, cmd.drive.steering_angle)
        rate.sleep()
    
    # 发送停止命令
    cmd.drive.speed = 0.0
    cmd_pub.publish(cmd)
    rospy.loginfo("测试完成，发送停止命令")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 