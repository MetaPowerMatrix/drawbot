#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>
#include <fstream>  // 添加串口操作支持
#include <unistd.h> // 添加usleep支持
#include <tinyxml2.h> // 添加XML解析支持

class ArmController {
private:
    std::ofstream serial_port;
    bool is_connected;

public:
    ArmController() : is_connected(false) {
        // 尝试打开串口
        serial_port.open("/dev/ttyUSB0");
        if (serial_port.is_open()) {
            is_connected = true;
            ROS_INFO("Arm controller connected to /dev/ttyUSB0");
        } else {
            ROS_WARN("Failed to open /dev/ttyUSB0 for arm control");
        }
    }

    ~ArmController() {
        if (is_connected) {
            serial_port.close();
        }
    }

    void sendCommand(const std::string& command) {
        if (!is_connected) return;
        
        serial_port << command << std::endl;
        usleep(100000);  // 等待100ms确保指令发送完成
        ROS_DEBUG("Sent arm command: %s", command.c_str());
    }

    void prepareArm() {
        sendCommand("H");
        sendCommand("x+30");
        sendCommand("y+15");
        sendCommand("z-40");
    }

    void resetArm() {
        sendCommand("H");
    }
};

class MoveDistance {
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber odom_sub_;
    
    double target_distance_;
    double target_angle_;
    double start_x_;
    double start_y_;
    double start_th_;
    bool movement_started_;
    bool goal_reached_;
    
    double linear_speed_;
    double angular_speed_;
    
    bool angle_first_;
    bool angle_finished_;
    
    double target_x_;
    double target_y_;
    
    double current_th_;
    
    // 添加当前位姿信息
    double current_x_;
    double current_y_;
    
    // 添加机械臂控制器
    ArmController arm_controller;
    
    // 添加新的成员变量
    double target_heading_;    // 目标朝向
    double path_curvature_;   // 路径曲率
    bool use_curve_motion_;   // 是否使用曲线运动
    
public:
    MoveDistance() : 
        target_x_(0.0), 
        target_y_(0.0),
        target_heading_(0.0),
        path_curvature_(0.0),
        use_curve_motion_(false),
        movement_started_(false), 
        goal_reached_(false), 
        angle_first_(true), 
        angle_finished_(false),
        current_th_(0.0) {
        // 默认参数设置
        target_distance_ = 0.0;
        target_angle_ = 0.0;
        linear_speed_ = 0.2;  // 默认线速度
        angular_speed_ = 0.5; // 默认角速度
        
        // 创建发布者和订阅者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        odom_sub_ = nh_.subscribe("odom", 10, &MoveDistance::odomCallback, this);
    }
    
    void setTargets(double target_x, double target_y, double heading = 0.0, double curvature = 0.0) {
        // 保存目标参数
        target_x_ = target_x;
        target_y_ = target_y;
        target_heading_ = heading;
        path_curvature_ = curvature;
        
        // 计算目标距离
        target_distance_ = std::sqrt(target_x_ * target_x_ + target_y_ * target_y_);
        
        // 处理特殊情况：直线后退
        if (target_x < 0 && fabs(target_y) < 1e-6) {
            target_angle_ = M_PI;
            angle_first_ = false;
            linear_speed_ = -0.2;
            use_curve_motion_ = false;
        } 
        // 处理曲线运动情况
        else if (fabs(target_y) > 1e-6 || fabs(path_curvature_) > 1e-6) {
            target_angle_ = std::atan2(target_y, target_x);
            use_curve_motion_ = true;
            linear_speed_ = 0.2;  // 曲线运动时使用较小的速度
            
            // 如果未指定曲率，根据目标点计算建议曲率
            if (fabs(path_curvature_) < 1e-6) {
                // 使用简单的圆弧估算
                double chord_length = target_distance_;
                double heading_diff = normalizeAngle(target_heading_ - target_angle_);
                path_curvature_ = 2 * sin(heading_diff / 2) / chord_length;
            }
        }
        // 普通直线前进
        else {
            target_angle_ = 0.0;
            angle_first_ = true;
            linear_speed_ = 0.2;
            use_curve_motion_ = false;
        }
        
        ROS_INFO("设置目标点: x=%.2f, y=%.2f, 朝向=%.2f, 曲率=%.2f", 
                target_x_, target_y_, target_heading_, path_curvature_);
        ROS_INFO("运动模式: %s", use_curve_motion_ ? "曲线运动" : "直线运动");
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 检查里程计消息的有效性
        if (!msg) {
            ROS_WARN("Received empty odom message");
            return;
        }

        // 更新当前位置信息
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_th_ = tf::getYaw(msg->pose.pose.orientation);

        if (!movement_started_) {
            // 记录起始位置和朝向
            start_x_ = current_x_;
            start_y_ = current_y_;
            start_th_ = current_th_;
            movement_started_ = true;
            ROS_INFO("Starting movement, initial position: (%.2f, %.2f), heading: %.2f", start_x_, start_y_, start_th_);
        }
        
        // 计算与目标角度的差值（考虑初始朝向）
        double target_relative_angle = std::atan2(target_y_, target_x_);
        double target_absolute_angle = start_th_ + target_relative_angle;
        double angle_diff = current_th_ - target_absolute_angle;

        // 标准化角度差到[-pi, pi]
        angle_diff = fmod(angle_diff + M_PI, 2*M_PI) - M_PI;

        if (angle_first_ && !angle_finished_) {
            ROS_DEBUG("Current angle error: %.2f rad", angle_diff);
            
            // 带小速度转向阶段
            if (fabs(angle_diff) < 0.05) {
                angle_finished_ = true;
                // 重置起始位置和朝向
                start_x_ = current_x_;
                start_y_ = current_y_;
                start_th_ = current_th_;
                ROS_INFO("Rotation completed, new start position: (%.2f, %.2f)", start_x_, start_y_);
            } else {
                // 带小线速度转向（0.1 m/s）
                move(true, angle_diff);
                return;
            }
        }
        
        // 计算已移动的距离
        double dx = current_x_ - start_x_;
        double dy = current_y_ - start_y_;
        double distance_moved = std::sqrt(dx*dx + dy*dy);
        
        // 输出调试信息
        ROS_DEBUG("Current position: (%.2f, %.2f), distance moved: %.2f meters",
                 current_x_,
                 current_y_,
                 distance_moved);
        
        // 检查是否达到目标距离
        if (distance_moved >= target_distance_) {
            stop();
            goal_reached_ = true;
            ROS_INFO("Movement completed: %.2f meters", distance_moved);
        } else if (!goal_reached_) {
            // 继续移动
            move(false, angle_diff);
            ROS_DEBUG("Continuing movement, target: %.2f meters, current: %.2f meters",
                     target_distance_, distance_moved);
        }
    }
    
    void move(bool rotating, double angle_error, double speed_factor=1.0) {
        geometry_msgs::Twist cmd_vel;
        
        if (rotating) {
            // 转向阶段保持不变
            cmd_vel.linear.x = 0.02;
            cmd_vel.angular.z = (angle_error < 0 ? 1 : -1) * angular_speed_ * speed_factor;
        } 
        else if (use_curve_motion_) {
            // 曲线运动模式
            cmd_vel.linear.x = linear_speed_;
            
            // 计算当前位置到目标点的向量
            double dx = target_x_ - (current_x_ - start_x_);
            double dy = target_y_ - (current_y_ - start_y_);
            double distance_to_goal = std::sqrt(dx*dx + dy*dy);
            
            // 计算期望航向角
            double desired_heading = std::atan2(dy, dx);
            double heading_error = normalizeAngle(desired_heading - current_th_);
            
            // 根据路径曲率和当前误差计算转向角速度
            double base_angular_velocity = linear_speed_ * path_curvature_;
            double correction = 0.5 * heading_error;  // 比例控制修正
            cmd_vel.angular.z = base_angular_velocity + correction;
            
            // 限制角速度
            cmd_vel.angular.z = std::max(-0.5, std::min(0.5, cmd_vel.angular.z));
            
            ROS_DEBUG("曲线运动: 距离=%.2f, 航向误差=%.2f, 角速度=%.2f", 
                    distance_to_goal, heading_error, cmd_vel.angular.z);
        }
        else {
            // 直线运动模式保持不变
            double heading_error;
            if (linear_speed_ < 0) {
                heading_error = 0;
            } else {
                heading_error = target_angle_ - (current_th_ - start_th_);
                heading_error = normalizeAngle(heading_error);
            }
            
            cmd_vel.linear.x = linear_speed_;
            cmd_vel.angular.z = 0.3 * heading_error;
        }
        
        cmd_vel_pub_.publish(cmd_vel);
    }
    
    void stop() {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
        cmd_vel_pub_.publish(cmd_vel);
        
        // 小车停止后重置机械臂
        arm_controller.resetArm();
    }
    
    bool isGoalReached() {
        return goal_reached_;
    }
    
    void reset() {
        // 重置移动状态，为下一个目标点做准备
        movement_started_ = false;
        goal_reached_ = false;
        angle_finished_ = false;
    }
    
    // 添加机械臂控制方法
    void controlArm(bool draw) {
        if (draw) {
            arm_controller.prepareArm();
            ROS_INFO("Preparing arm for drawing");
        } else {
            arm_controller.resetArm();
            ROS_INFO("Resetting arm after drawing");
        }
    }
    
    // 添加角度标准化函数
    double normalizeAngle(double angle) {
        return fmod(angle + M_PI, 2*M_PI) - M_PI;
    }
};

// 定义目标点结构体
struct TargetPoint {
    double x;          // 目标x坐标
    double y;          // 目标y坐标
    double heading;    // 目标朝向角度（弧度）
    double curvature; // 路径曲率（0表示直线，正值表示左转，负值表示右转）
    bool draw;        // 是否绘制
};

// 解析XML文件，获取目标点集合
std::vector<TargetPoint> parseRouteFile(const std::string& file_path) {
    std::vector<TargetPoint> targets;
    tinyxml2::XMLDocument doc;
    
    // 打印文件路径，便于调试
    ROS_INFO("Attempting to load route file: %s", file_path.c_str());
    
    // 打开文件并检查错误
    tinyxml2::XMLError result = doc.LoadFile(file_path.c_str());
    if (result != tinyxml2::XML_SUCCESS) {
        ROS_ERROR("Failed to load route file: %s (Error code: %d)", file_path.c_str(), result);
        const char* errorStr1 = doc.ErrorStr();
        ROS_ERROR("XML Error description: %s", errorStr1);
        return targets;
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("SignDrawInfo");
    if (!root) {
        ROS_ERROR("Invalid route file format: missing SignDrawInfo element");
        return targets;
    }

    // 遍历每个DrawStep
    for (tinyxml2::XMLElement* step = root->FirstChildElement("DrawStep"); step; step = step->NextSiblingElement("DrawStep")) {
        int seqno = step->IntAttribute("seqno", -1);
        ROS_DEBUG("Processing DrawStep with seqno: %d", seqno);
        
        tinyxml2::XMLElement* linepts = step->FirstChildElement("linepts");
        if (!linepts) {
            ROS_WARN("DrawStep without linepts found (seqno: %d), skipping", seqno);
            continue;
        }
        
        // 遍历每个linept
        for (tinyxml2::XMLElement* linept = linepts->FirstChildElement("linept"); linept; linept = linept->NextSiblingElement("linept")) {
            TargetPoint point;
            point.x = linept->DoubleAttribute("x");
            point.y = linept->DoubleAttribute("y");
            
            // 获取目标朝向（如果未指定则根据下一个点计算）
            point.heading = linept->DoubleAttribute("heading", 0.0);
            
            // 获取路径曲率（默认为0，表示直线运动）
            point.curvature = linept->DoubleAttribute("curvature", 0.0);
            
            // 获取draw属性
            const char* draw_attr = linept->Attribute("draw");
            if (draw_attr) {
                std::string draw_str = draw_attr;
                std::transform(draw_str.begin(), draw_str.end(), draw_str.begin(), ::tolower);
                point.draw = (draw_str.find("true") != std::string::npos);
            } else {
                ROS_WARN("Missing 'draw' attribute for point (%.2f, %.2f), defaulting to false", point.x, point.y);
                point.draw = false;
            }
            
            targets.push_back(point);
            ROS_INFO("Added target point: (%.2f, %.2f), heading=%.2f, curvature=%.2f, draw=%s", 
                    point.x, point.y, point.heading, point.curvature, point.draw ? "true" : "false");
        }
    }

    // 如果没有指定heading，根据下一个点计算默认朝向
    for (size_t i = 0; i < targets.size() - 1; ++i) {
        if (targets[i].heading == 0.0) {
            double dx = targets[i + 1].x - targets[i].x;
            double dy = targets[i + 1].y - targets[i].y;
            targets[i].heading = std::atan2(dy, dx);
        }
    }
    // 最后一个点如果未指定朝向，保持与前一个点相同
    if (!targets.empty() && targets.back().heading == 0.0 && targets.size() > 1) {
        targets.back().heading = targets[targets.size() - 2].heading;
    }

    ROS_INFO("Parsed %lu target points from route file", targets.size());
    return targets;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "move_distance");
    
    if (argc != 2) {
        ROS_ERROR("Usage: %s <route_file_path>", argv[0]);
        return 1;
    }
    
    // 解析路由文件
    std::string route_file = argv[1];
    std::vector<TargetPoint> targets = parseRouteFile(route_file);
    if (targets.empty()) {
        ROS_ERROR("No valid target points found in route file");
        return 1;
    }
    
    MoveDistance move_controller;
    
    // 等待1秒钟让ROS系统完全初始化
    ros::Duration(1.0).sleep();
    
    // 按顺序处理每个目标点
    for (size_t i = 0; i < targets.size(); ++i) {
        const auto& target = targets[i];
        ROS_INFO("移动到目标点 %lu: (%.2f, %.2f), 朝向=%.2f°, 曲率=%.2f", 
                i+1, target.x, target.y, target.heading * 180.0 / M_PI, target.curvature);
        
        // 根据draw属性控制机械臂
        move_controller.controlArm(target.draw);
        
        // 设置目标点并开始移动
        move_controller.setTargets(target.x, target.y, target.heading, target.curvature);
        
        // 等待小车到达目标点
        ros::Rate rate(10);
        while (ros::ok() && !move_controller.isGoalReached()) {
            ros::spinOnce();
            rate.sleep();
        }
        
        // 重置移动状态，为下一个目标点做准备
        if (i < targets.size() - 1) {
            move_controller.reset();
        }
    }
    
    ROS_INFO("所有目标点已到达");
    return 0;
}