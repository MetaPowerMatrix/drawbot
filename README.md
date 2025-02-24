# 自动导航阿克曼小车项目

## 项目简介
本项目基于Jetson Nano和RPLidar实现了一个自动导航的阿克曼小车系统。使用ROS作为开发框架，实现了SLAM建图、路径规划和运动控制等功能。

## 硬件要求
- Jetson Nano B01
- RPLidar
- 阿克曼结构小车底盘

## 软件依赖
- ROS Melodic/Noetic
- rplidar_ros
- navigation stack
- ackermann_msgs

## 详细安装步骤

### 1. 安装ROS Melodic
```bash
# 设置软件源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 添加ROS密钥
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 更新软件包列表
sudo apt update

# 安装ROS完整版
sudo apt install ros-melodic-desktop-full

# 初始化rosdep
sudo rosdep init
rosdep update

# 设置环境变量
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 安装常用工具
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### 2. 创建工作空间
```bash
mkdir -p ~/ackermann_ws/src
cd ~/ackermann_ws/src
catkin_init_workspace
```

3. 克隆必要的ROS包：
```bash
git clone https://github.com/robopeak/rplidar_ros.git
```

4. 编译工作空间：
```bash
cd ~/ackermann_ws
catkin_make
```

## 使用说明
1. 启动RPLidar节点：
```bash
roslaunch rplidar_ros rplidar.launch
```

2. 启动导航节点：
```bash
roslaunch ackermann_nav navigation.launch
```

## 文件结构
```
.
├── src/
│   ├── ackermann_nav/       # 导航功能包
│   ├── ackermann_msgs/      # 阿克曼消息类型
│   └── rplidar_ros/         # RPLidar驱动包
```

## 许可证
MIT License

## 贡献
欢迎提交Issue和Pull Request