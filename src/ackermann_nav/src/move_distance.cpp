#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <cmath>
#include <fstream>  // 添加串口操作支持
#include <unistd.h> // 添加usleep支持
#include <tinyxml2.h> // 添加XML解析支持
#include <std_msgs/Bool.h> // 添加碰撞警告消息支持

class ArmController {
private:
    std::ofstream serial_port;
    bool is_connected;
    std::string port_name;

public:
    ArmController(const std::string& port = "/dev/ttyUSB1") : is_connected(false), port_name(port) {
        // 尝试打开串口
        serial_port.open(port_name);
        if (serial_port.is_open()) {
            is_connected = true;
            ROS_INFO("Arm controller connected to %s", port_name.c_str());
        } else {
            ROS_WARN("Failed to open %s for arm control", port_name.c_str());
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
    ros::Subscriber collision_sub_; // 添加碰撞警告订阅者
    
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
    
    // 碰撞避免相关变量
    bool collision_detected_; // 是否检测到碰撞
    bool avoiding_;           // 是否正在执行避障
    ros::Time collision_time_; // 碰撞检测时间
    int avoidance_phase_;     // 避障阶段：0=后退, 1=转向, 2=继续前进
    
    // 添加Ackerman参数
    double wheelbase_;  // 前后轮轴距
    double max_steering_angle_;  // 最大转向角
    
    // 计算Ackerman转向角速度
    double calculateAckermanSteering(double linear_vel, double desired_curvature) {
        if (fabs(linear_vel) < 0.01) return 0.0;
        
        // 计算转向角 (δ = atan(L * κ))
        double steering_angle = atan(wheelbase_ * desired_curvature);
        
        // 限制转向角
        steering_angle = std::max(-max_steering_angle_, 
                                 std::min(max_steering_angle_, steering_angle));
        
        // 计算等效的角速度 (ω = v * κ)
        return linear_vel * tan(steering_angle) / wheelbase_;
    }
public:
    MoveDistance(const std::string& arm_port = "/dev/ttyUSB1") : 
        arm_controller(arm_port),
        target_x_(0.0), 
        target_y_(0.0),
        target_heading_(0.0),
        path_curvature_(0.0),
        use_curve_motion_(false),
        movement_started_(false), 
        goal_reached_(false), 
        angle_first_(true), 
        angle_finished_(false),
        current_th_(0.0),
        collision_detected_(false),
        avoiding_(false),
        avoidance_phase_(0),
        // 默认参数设置
        target_distance_ = 0.0,
        target_angle_ = 0.0,
        linear_speed_ = 0.2,  // 默认线速度
        angular_speed_ = 0.5, // 默认角速度
        wheelbase_(0.15),  // 根据实际小车设置
        max_steering_angle_(0.6) {  // 约35度

        // 创建发布者和订阅者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        odom_sub_ = nh_.subscribe("odom", 10, &MoveDistance::odomCallback, this);
        
        // 订阅碰撞警告消息
        collision_sub_ = nh_.subscribe("collision_warning", 10, &MoveDistance::collisionCallback, this);
        
        ROS_INFO("MoveDistance初始化完成，已订阅collision_warning消息");
    }
    
    // 添加碰撞回调函数
    void collisionCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data && !collision_detected_ && !avoiding_) {
            ROS_WARN("检测到碰撞警告！启动避障程序");
            collision_detected_ = true;
            collision_time_ = ros::Time::now();
            avoiding_ = true;
            avoidance_phase_ = 0; // 开始避障的第一阶段：后退
            
            // 立即停止当前动作
            stop();
        }
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
        
        ROS_INFO("Set target: x=%.2f, y=%.2f, heading=%.2f, curvature=%.2f", 
                target_x_, target_y_, target_heading_, path_curvature_);
        ROS_INFO("Motion mode: %s", use_curve_motion_ ? "Curve motion" : "Straight motion");
    }
    
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        // 更新当前位置和朝向
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_th_ = tf::getYaw(msg->pose.pose.orientation);
        
        if (!movement_started_) {
            start_x_ = current_x_;
            start_y_ = current_y_;
            start_th_ = current_th_;
            movement_started_ = true;
            return;
        }
        
        // 计算在局部坐标系中的移动
        double delta_x = (current_x_ - start_x_) * cos(start_th_) + (current_y_ - start_y_) * sin(start_th_);
        double delta_y = -(current_x_ - start_x_) * sin(start_th_) + (current_y_ - start_y_) * cos(start_th_);
        
        // 计算距离和角度误差
        double distance_error = target_distance_ - sqrt(delta_x*delta_x + delta_y*delta_y);
        double angle_error = atan2(target_y_, target_x_) - atan2(delta_y, delta_x);
        
        // 标准化角度误差
        angle_error = fmod(angle_error + M_PI, 2*M_PI) - M_PI;
        
        // 检查是否到达目标
        if (fabs(distance_error) < 0.05 && fabs(angle_error) < 0.05) {
            stop();
            goal_reached_ = true;
            return;
        }
        
        // 根据误差计算控制命令
        move(distance_error, angle_error);
    }
    
    // 处理避障逻辑
    void handleAvoidance() {
        ros::Duration time_since_collision = ros::Time::now() - collision_time_;
        
        // 根据不同的避障阶段执行不同动作
        if (avoidance_phase_ == 0) {  // 后退阶段
            if (time_since_collision.toSec() < 1.0) {  // 后退1秒
                // 执行后退动作
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = -0.15;  // 低速后退
                cmd_vel.angular.z = 0.0;
                cmd_vel_pub_.publish(cmd_vel);
                ROS_DEBUG("避障-后退阶段 (%.1f秒)", time_since_collision.toSec());
            } else {
                // 进入转向阶段
                avoidance_phase_ = 1;
                collision_time_ = ros::Time::now();
            }
        } else if (avoidance_phase_ == 1) {  // 转向阶段
            if (time_since_collision.toSec() < 1.5) {  // 转向1.5秒
                // 执行转向动作（随机选择左转或右转，这里选择右转）
                geometry_msgs::Twist cmd_vel;
                cmd_vel.linear.x = 0.0;
                cmd_vel.angular.z = -0.5;  // 右转
                cmd_vel_pub_.publish(cmd_vel);
                ROS_DEBUG("避障-转向阶段 (%.1f秒)", time_since_collision.toSec());
            } else {
                // 避障完成，恢复正常导航
                avoiding_ = false;
                collision_detected_ = false;
                ROS_INFO("避障完成，恢复正常导航");
                
                // 重置导航状态
                reset();
            }
        }
    }

    void move(double distance_error, double angle_error) {
        static ros::Time last_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;
        
        // 安全检查：确保时间间隔合理
        if (dt <= 0 || dt > 0.1) {
            dt = 0.05;  // 默认50ms
        }

        // PID参数
        const double kp_distance = 0.8;   // 距离比例系数
        const double ki_distance = 0.01;  // 距离积分系数
        const double kd_distance = 0.1;   // 距离微分系数
        
        const double kp_angle = 1.2;      // 角度比例系数
        const double ki_angle = 0.02;     // 角度积分系数
        const double kd_angle = 0.15;     // 角度微分系数
        
        // 限制最大速度和加速度
        const double max_linear_vel = 0.5;     // m/s
        const double max_angular_vel = 0.8;    // rad/s
        const double max_linear_accel = 0.3;   // m/s^2
        const double max_angular_accel = 1.0;  // rad/s^2

        // ====== 1. 距离控制 ======
        // 更新PID项
        static double last_distance_error = 0;
        static double integral_distance = 0;
        
        // 比例项
        double p_distance = kp_distance * distance_error;
        
        // 积分项（带抗饱和）
        integral_distance += ki_distance * distance_error * dt;
        integral_distance = std::max(-1.0, std::min(1.0, integral_distance));
        
        // 微分项
        double d_distance = kd_distance * (distance_error - last_distance_error) / dt;
        last_distance_error = distance_error;
        
        // 计算期望线速度
        double desired_linear_vel = p_distance + integral_distance + d_distance;
        desired_linear_vel = std::max(-max_linear_vel, std::min(max_linear_vel, desired_linear_vel));

        // ====== 2. 角度控制 ======
        // 更新PID项
        static double last_angle_error = 0;
        static double integral_angle = 0;
        
        // 比例项
        double p_angle = kp_angle * angle_error;
        
        // 积分项（带抗饱和）
        integral_angle += ki_angle * angle_error * dt;
        integral_angle = std::max(-0.5, std::min(0.5, integral_angle));
        
        // 微分项
        double d_angle = kd_angle * (angle_error - last_angle_error) / dt;
        last_angle_error = angle_error;
        
        // 计算期望曲率 (Ackerman转向模型)
        double desired_curvature = (p_angle + integral_angle + d_angle) / (target_distance_ + 0.1);
        
        // ====== 3. Ackerman转向计算 ======
        geometry_msgs::Twist cmd_vel;
        
        // 计算最终线速度（考虑加速度限制）
        static double last_linear_vel = 0;
        double linear_accel = (desired_linear_vel - last_linear_vel) / dt;
        linear_accel = std::max(-max_linear_accel, std::min(max_linear_accel, linear_accel));
        cmd_vel.linear.x = last_linear_vel + linear_accel * dt;
        last_linear_vel = cmd_vel.linear.x;
        
        // 计算角速度 (ω = v * κ / (1 + L*κ^2)) - 考虑转向动力学
        if (fabs(cmd_vel.linear.x) > 0.05) {  // 只有有足够速度时才转向
            double effective_curvature = desired_curvature / (1 + wheelbase_ * desired_curvature * desired_curvature);
            cmd_vel.angular.z = cmd_vel.linear.x * tan(atan(wheelbase_ * effective_curvature)) / wheelbase_;
        } else {
            // 低速时使用简化转向模型
            cmd_vel.angular.z = 0.5 * (p_angle + integral_angle + d_angle);
        }
        
        // 限制角速度和角加速度
        static double last_angular_vel = 0;
        double angular_accel = (cmd_vel.angular.z - last_angular_vel) / dt;
        angular_accel = std::max(-max_angular_accel, std::min(max_angular_accel, angular_accel));
        cmd_vel.angular.z = last_angular_vel + angular_accel * dt;
        cmd_vel.angular.z = std::max(-max_angular_vel, std::min(max_angular_vel, cmd_vel.angular.z));
        last_angular_vel = cmd_vel.angular.z;
        
        // ====== 4. 特殊情况处理 ======
        // 接近目标时的减速处理
        double slow_down_dist = 0.5;  // 开始减速的距离
        if (fabs(distance_error) < slow_down_dist) {
            double speed_factor = fabs(distance_error) / slow_down_dist;
            cmd_vel.linear.x *= std::max(0.1, speed_factor);
            cmd_vel.angular.z *= std::max(0.1, speed_factor);
        }
        
        // ====== 5. 发布控制命令 ======
        cmd_vel_pub_.publish(cmd_vel);
        
        // 调试输出
        ROS_DEBUG_THROTTLE(0.5, 
            "Move Control: "
            "LinVel=%.2f(desired %.2f) "
            "AngVel=%.2f(κ=%.3f) "
            "DistErr=%.2f AngErr=%.1f°",
            cmd_vel.linear.x, desired_linear_vel,
            cmd_vel.angular.z, desired_curvature,
            distance_error, angle_error * 180.0/M_PI);
    }
    // void move(double distance_error, double angle_error) {
    //     geometry_msgs::Twist cmd_vel;
        
    //     // 使用PID控制器计算速度和转向
    //     double kp_distance = 0.5;  // 距离比例系数
    //     double kp_angle = 1.0;     // 角度比例系数
        
    //     // 基本线速度 - 根据距离误差调整
    //     cmd_vel.linear.x = kp_distance * distance_error;
        
    //     // 限制最大速度
    //     cmd_vel.linear.x = std::max(-0.3, std::min(0.3, cmd_vel.linear.x));
        
    //     // Ackerman转向模型 - 根据角度误差和线速度计算转向角速度
    //     if (fabs(cmd_vel.linear.x) > 0.01) {  // 只有有前进速度时才转向
    //         // 计算期望的转向半径: R = v / ω
    //         // 我们希望转向半径与角度误差成反比
    //         double desired_omega = kp_angle * angle_error * cmd_vel.linear.x;
            
    //         // 限制最大转向角速度
    //         cmd_vel.angular.z = std::max(-0.5, std::min(0.5, desired_omega));
    //     } else {
    //         // 如果几乎没有前进速度，原地转向
    //         cmd_vel.angular.z = kp_angle * angle_error;
    //     }
        
    //     cmd_vel_pub_.publish(cmd_vel);
    // }
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
    
    // 解析参数：路线文件和可选的串口设备名
    std::string route_file;
    std::string arm_port = "/dev/ttyUSB1"; // 默认串口设备名
    
    if (argc < 2) {
        ROS_ERROR("用法: %s <route_file_path> [arm_port]", argv[0]);
        return 1;
    }
    
    route_file = argv[1];
    
    // 如果指定了串口设备名，则使用指定的设备名
    if (argc > 2) {
        arm_port = argv[2];
        ROS_INFO("使用指定的机械臂串口设备: %s", arm_port.c_str());
    } else {
        ROS_INFO("使用默认机械臂串口设备: %s", arm_port.c_str());
    }
    
    // 解析路由文件
    std::vector<TargetPoint> targets = parseRouteFile(route_file);
    if (targets.empty()) {
        ROS_ERROR("路由文件中未找到有效的目标点");
        return 1;
    }
    
    // 创建移动控制器，使用指定的串口设备名
    MoveDistance move_controller(arm_port);
    
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
        
        // 等待小车到达目标点，或遇到障碍物避障
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