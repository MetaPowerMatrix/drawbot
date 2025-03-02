#!/usr/bin/env python
import os
import time
import rospy
from std_msgs.msg import Int32, Float32

class FanController:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('fan_controller', anonymous=True)
        
        # 创建发布器，发布风扇速度和温度信息
        self.fan_speed_pub = rospy.Publisher('/fan_speed', Int32, queue_size=10)
        self.cpu_temp_pub = rospy.Publisher('/cpu_temperature', Float32, queue_size=10)
        
        # 风扇PWM控制文件路径
        self.fan_pwm_path = '/sys/devices/pwm-fan/target_pwm'
        
        # 温度阈值和对应的风扇速度（PWM值范围0-255）
        self.temp_thresholds = {
            50: 100,    # 温度低于50℃，风扇速度40%
            60: 150,    # 温度在50-60℃，风扇速度60%
            70: 200,    # 温度在60-70℃，风扇速度80%
            80: 255     # 温度高于70℃，风扇速度100%
        }

    def get_cpu_temp(self):
        """获取CPU温度"""
        try:
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp = float(f.read().strip()) / 1000.0
            return temp
        except Exception as e:
            rospy.logerr(f"Error reading CPU temperature: {e}")
            return 0

    def set_fan_speed(self, speed):
        """设置风扇速度"""
        try:
            with open(self.fan_pwm_path, 'w') as f:
                f.write(str(speed))
            self.fan_speed_pub.publish(speed)
            rospy.loginfo(f"风扇速度设置为: {speed}")
        except Exception as e:
            rospy.logerr(f"设置风扇速度出错: {e}")

    def control_fan(self):
        """根据温度控制风扇速度"""
        rate = rospy.Rate(0.2)  # 0.2Hz更新频率，即每5秒一次
        
        while not rospy.is_shutdown():
            temp = self.get_cpu_temp()
            
            # 发布CPU温度
            self.cpu_temp_pub.publish(temp)
            
            # 根据温度设置风扇速度
            if temp < 50:
                self.set_fan_speed(100)
            elif temp < 60:
                self.set_fan_speed(150)
            elif temp < 70:
                self.set_fan_speed(200)
            else:
                self.set_fan_speed(255)
            
            # 打印CPU温度
            rospy.loginfo(f"CPU温度: {temp}°C")
            
            # 等待5秒
            rate.sleep()

if __name__ == '__main__':
    try:
        # 确保脚本有权限控制风扇
        os.system('sudo chmod 666 /sys/devices/pwm-fan/target_pwm')
        
        controller = FanController()
        rospy.loginfo(f"start fan control")
        controller.control_fan()
    except rospy.ROSInterruptException:
        pass