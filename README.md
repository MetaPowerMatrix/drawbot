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

# 安装rosdep工具
sudo apt install python-rosdep

# 初始化rosdep
sudo rosdep init
rosdep update

# 设置环境变量
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 安装其他常用工具
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
```

### 2. 创建工作空间
```bash
# 创建工作空间目录
mkdir -p ~/ackermann_ws/src
cd ~/ackermann_ws/src
catkin_init_workspace

# 返回工作空间根目录
cd ~/ackermann_ws
# 首次初始化工作空间
catkin_make

# 设置工作空间环境变量
echo "source ~/ackermann_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. 安装依赖包
```bash
# 安装Python依赖
sudo apt-get install python-pip
sudo pip install pyyaml
sudo pip install rospkg empy catkin_pkg

# 安装ROS依赖包
sudo apt-get install ros-melodic-ackermann-msgs
sudo apt-get install ros-melodic-navigation
```

### 4. 克隆必要的ROS包
```bash
# 克隆RPLidar驱动包
cd ~/ackermann_ws/src
git clone https://github.com/robopeak/rplidar_ros.git

# 创建导航功能包
catkin_create_pkg ackermann_nav roscpp rospy std_msgs geometry_msgs nav_msgs ackermann_msgs tf2 tf2_ros
```

### 5. 创建启动文件和配置文件
```bash
# 创建必要的目录
cd ~/ackermann_ws/src/ackermann_nav
mkdir -p launch config urdf

# 创建配置文件
cd config
touch costmap_common_params.yaml
touch local_costmap_params.yaml
touch global_costmap_params.yaml
touch base_local_planner_params.yaml

# 创建URDF目录和文件
cd ../urdf
touch ackermann.urdf.xacro

# 安装额外依赖
sudo apt-get install ros-melodic-robot-state-publisher ros-melodic-amcl ros-melodic-move-base ros-melodic-map-server
sudo apt-get install ros-melodic-gmapping
```

### 6. 编译工作空间
```bash
# 在编译之前更新ROS依赖
cd ~/ackermann_ws
rosdep install --from-paths src --ignore-src -r -y

# 确保在工作空间根目录下编译
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