#!/usr/bin/env python
import os
import time

def get_cpu_temp():
    """获取CPU温度"""
    try:
        with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
            temp = float(f.read().strip()) / 1000.0
        return temp
    except Exception as e:
        print(f"读取CPU温度出错: {e}")
        return 0

def set_fan_speed(speed):
    """设置风扇速度"""
    try:
        with open('/sys/devices/pwm-fan/target_pwm', 'w') as f:
            f.write(str(speed))
        print(f"风扇速度设置为: {speed}")
    except Exception as e:
        print(f"设置风扇速度出错: {e}")

def control_fan():
    """根据温度控制风扇速度"""
    print("开始风扇控制")
    
    # 确保脚本有权限控制风扇
    os.system('sudo chmod 666 /sys/devices/pwm-fan/target_pwm')
    
    try:
        while True:
            temp = get_cpu_temp()
            
            # 根据温度设置风扇速度
            if temp < 50:
                set_fan_speed(100)
            elif temp < 60:
                set_fan_speed(150)
            elif temp < 70:
                set_fan_speed(200)
            else:
                set_fan_speed(255)
            
            # 打印CPU温度
            print(f"CPU温度: {temp}°C")
            
            # 等待5秒
            time.sleep(5)
    except KeyboardInterrupt:
        print("风扇控制已停止")

if __name__ == '__main__':
    control_fan() 