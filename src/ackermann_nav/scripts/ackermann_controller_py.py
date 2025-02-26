#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import serial
import time

class AckermannController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('ackermann_controller', anonymous=True)
        
        # 串口配置（根据你的控制板调整端口和波特率）
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        
        # 订阅/cmd_vel话题
        rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # 参数：最大速度和转向角度限制（根据你的小车调整）
        self.max_speed = 1.0  # m/s
        self.max_steer = 0.5  # 弧度
        
        rospy.loginfo("Ackermann Controller Initialized")
        
    def cmd_vel_callback(self, msg):
        # 获取ROS的Twist消息
        linear_speed = msg.linear.x  # 前进速度
        angular_vel = msg.angular.z  # 角速度
        
        # 限制速度和角度
        linear_speed = max(min(linear_speed, self.max_speed), -self.max_speed)
        
        # 把角速度转为转向角度（简化为比例关系，实际需阿克曼模型计算）
        steer_angle = angular_vel * 0.5  # 假设比例系数0.5，可调
        steer_angle = max(min(steer_angle, self.max_steer), -self.max_steer)
        
        # 构造控制板指令
        command = f"{linear_speed:.2f},{steer_angle:.2f}\n"
        
        # 发送到控制板
        self.serial_port.write(command.encode('utf-8'))
        rospy.loginfo(f"Sent to controller: {command.strip()}")
        
    def run(self):
        # 保持节点运行
        rospy.spin()
        
    def shutdown(self):
        # 关闭时停止小车
        self.serial_port.write("0.0,0.0\n".encode('utf-8'))
        self.serial_port.close()

if __name__ == '__main__':
    try:
        controller = AckermannController()
        controller.run()
    except rospy.ROSInterruptException:
        controller.shutdown()