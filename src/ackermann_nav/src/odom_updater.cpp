#include "odom_updater.h"
#include <algorithm>

OdomUpdater::OdomUpdater(ros::NodeHandle& nh) :
    nh_(nh),
    x_(0.0), y_(0.0), th_(0.0),
    velocity_x_(0.0), velocity_y_(0.0),
    last_v_(0.0), last_steering_angle_(0.0),
    angular_velocity_z_(0.0),
    linear_acceleration_x_(0.0), linear_acceleration_y_(0.0),
    acceleration_x_(0.0), acceleration_y_(0.0),
    controller_linear_x_(0.0), controller_linear_y_(0.0), controller_linear_z_(0.0),
    controller_angular_x_(0.0), controller_angular_y_(0.0), controller_angular_z_(0.0),
    controller_odom1_(0), controller_odom2_(0),
    controller_battery_(0.0), controller_flag_stop_(0),
    frame_in_sync_(false),
    consecutive_valid_frames_(0),
    consecutive_invalid_frames_(0),
    frames_processed_(0),
    valid_frames_(0),
    invalid_frames_(0),
    use_imu_data_(false),
    fusion_alpha_(0.8),
    odom_method_(ENCODER_ONLY),
    odom_log_counter_(0),
    controller_scale_(10000.0) {
    
    // 获取参数
    nh_.param<double>("wheelbase", wheelbase_, 0.25);
    
    std::string method_str;
    nh_.param<std::string>("odom_method", method_str, "encoder_only");
    nh_.param<double>("fusion_alpha", fusion_alpha_, 0.8);
    nh_.param<double>("controller_scale", controller_scale_, 10000.0);
    
    // 创建发布者
    odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 50);
    
    // 根据参数选择里程计方法
    if (method_str == "imu_fusion") {
        odom_method_ = IMU_FUSION;
        use_imu_data_ = true;
        
        // 订阅IMU话题
        imu_sub_ = nh_.subscribe("imu/data", 10, &OdomUpdater::imuCallback, this);
        ROS_INFO("OdomUpdater: Using IMU fusion method (alpha=%.2f)", fusion_alpha_);
    } else if (method_str == "controller_data") {
        odom_method_ = CONTROLLER_DATA;
        ROS_INFO("OdomUpdater: Using controller data method (scale=%.2f)", controller_scale_);
    } else {
        // 默认使用编码器方法
        odom_method_ = ENCODER_ONLY;
        ROS_INFO("OdomUpdater: Using encoder-only method");
    }
    
    last_time_ = ros::Time::now();
    
    ROS_INFO("OdomUpdater initialized with wheelbase=%.3f", wheelbase_);
}

bool OdomUpdater::processControllerDataStream(const std::vector<uint8_t>& data) {
    // 将新数据添加到缓冲区
    for (const auto& byte : data) {
        data_buffer_.push_back(byte);
    }
    
    // 如果缓冲区过大，移除旧数据
    const size_t MAX_BUFFER_SIZE = 1024;
    if (data_buffer_.size() > MAX_BUFFER_SIZE) {
        size_t excess = data_buffer_.size() - MAX_BUFFER_SIZE;
        data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + excess);
        ROS_DEBUG("OdomUpdater: Trimmed data buffer by %zu bytes to prevent overflow", excess);
    }
    
    // 尝试提取并处理完整数据帧
    bool processed_frame = false;
    std::vector<uint8_t> frame;
    
    while (extractFrame(frame)) {
        processed_frame = true;
        frames_processed_++;
        
        // 尝试解析和处理这一帧
        if (updateFromControllerData(frame)) {
            valid_frames_++;
            consecutive_valid_frames_++;
            consecutive_invalid_frames_ = 0;
            
            // 连续成功解析多帧，可以认为同步状态良好
            if (consecutive_valid_frames_ >= 3 && !frame_in_sync_) {
                frame_in_sync_ = true;
                ROS_INFO("OdomUpdater: Data stream synchronized (after %d valid frames)", 
                        consecutive_valid_frames_);
            }
        } else {
            invalid_frames_++;
            consecutive_invalid_frames_++;
            consecutive_valid_frames_ = 0;
            
            // 连续解析失败多帧，可能丢失同步
            if (consecutive_invalid_frames_ >= 5 && frame_in_sync_) {
                frame_in_sync_ = false;
                ROS_WARN("OdomUpdater: Lost data stream synchronization (after %d invalid frames)",
                        consecutive_invalid_frames_);
            }
        }
        
        // 清空当前帧以处理下一帧
        frame.clear();
    }
    
    // 每500帧打印一次统计信息
    if (frames_processed_ % 500 == 0) {
        float valid_rate = frames_processed_ > 0 ? 
                          (float)valid_frames_ / frames_processed_ * 100.0f : 0.0f;
        ROS_INFO("OdomUpdater: Frames processed: %d, valid: %d (%.1f%%), invalid: %d, buffer size: %zu bytes",
                frames_processed_, valid_frames_, valid_rate, invalid_frames_, data_buffer_.size());
    }
    
    return processed_frame;
}

int OdomUpdater::findFrameHeader() {
    // 查找帧头在缓冲区中的位置
    for (size_t i = 0; i < data_buffer_.size(); ++i) {
        if (data_buffer_[i] == FRAME_HEADER) {
            return static_cast<int>(i);
        }
    }
    return -1;  // 未找到帧头
}

bool OdomUpdater::extractFrame(std::vector<uint8_t>& frame) {
    // 缓冲区太小，无法包含完整帧
    if (data_buffer_.size() < FRAME_SIZE) {
        return false;
    }
    
    // 查找帧头
    int header_pos = findFrameHeader();
    if (header_pos < 0) {
        // 未找到帧头，清空缓冲区重新开始
        data_buffer_.clear();
        return false;
    }
    
    // 如果帧头不在缓冲区开始位置，移除帧头之前的数据
    if (header_pos > 0) {
        data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + header_pos);
        header_pos = 0;  // 现在帧头在缓冲区开始位置
    }
    
    // 检查缓冲区是否包含完整帧
    if (data_buffer_.size() < FRAME_SIZE) {
        // 不完整，等待更多数据
        return false;
    }
    
    // 检查帧尾是否正确
    if (data_buffer_[FRAME_SIZE - 1] != FRAME_TAIL) {
        // 帧尾不正确，移除帧头并继续搜索
        data_buffer_.pop_front();
        return extractFrame(frame);  // 递归尝试提取下一帧
    }
    
    // 提取完整帧
    frame.resize(FRAME_SIZE);
    for (size_t i = 0; i < FRAME_SIZE; ++i) {
        frame[i] = data_buffer_[i];
    }
    
    // 从缓冲区移除已处理的帧
    data_buffer_.erase(data_buffer_.begin(), data_buffer_.begin() + FRAME_SIZE);
    
    return true;
}

void OdomUpdater::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    // 获取角速度（绕z轴的旋转速度）
    angular_velocity_z_ = msg->angular_velocity.z;
    
    // 获取线性加速度
    linear_acceleration_x_ = msg->linear_acceleration.x;
    linear_acceleration_y_ = msg->linear_acceleration.y;
    
    // 计算当前加速度（世界坐标系）
    acceleration_x_ = linear_acceleration_x_ * cos(th_) - linear_acceleration_y_ * sin(th_);
    acceleration_y_ = linear_acceleration_x_ * sin(th_) + linear_acceleration_y_ * cos(th_);
    
    ROS_DEBUG("IMU数据：角速度_z=%.3f rad/s, 加速度_x=%.3f m/s², 加速度_y=%.3f m/s²",
             angular_velocity_z_, acceleration_x_, acceleration_y_);
}

void OdomUpdater::update(double v, double steering_angle) {
    // 增加计数器
    odom_log_counter_++;
    
    // 只在指定间隔输出详细日志
    bool should_log = (odom_log_counter_ % ODOM_LOG_INTERVAL == 0);
    
    // 更新速度信息
    last_v_ = v;
    last_steering_angle_ = steering_angle;
    
    // 计算时间增量
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();
    
    // 防止时间增量过小
    if (dt < 0.0001) {
        ROS_WARN_THROTTLE(1.0, "OdomUpdater: Time increment too small (dt=%.6f)", dt);
        last_time_ = current_time;
        return;
    }
    
    // 根据选择的方法更新里程计
    double delta_x = 0.0;
    double delta_y = 0.0;
    double delta_th = 0.0;
    
    switch (odom_method_) {
        case ENCODER_ONLY:
            updateEncoderOnly(v, steering_angle, dt, delta_x, delta_y, delta_th);
            break;
            
        case IMU_FUSION:
            updateWithIMUFusion(v, steering_angle, dt, delta_x, delta_y, delta_th);
            break;
            
        case CONTROLLER_DATA:
            updateWithControllerData(dt, delta_x, delta_y, delta_th);
            break;
            
        case CUSTOM_METHOD:
            // 实现其他自定义方法
            updateEncoderOnly(v, steering_angle, dt, delta_x, delta_y, delta_th);
            break;
            
        default:
            updateEncoderOnly(v, steering_angle, dt, delta_x, delta_y, delta_th);
    }
    
    // 更新位置
    x_ += delta_x;
    y_ += delta_y;
    th_ += delta_th;
    
    // 标准化角度在-π到π之间
    th_ = normalizeAngle(th_);
    
    if (should_log) {
        ROS_INFO("Odometry update: dt=%.3f, dx=%.3f, dy=%.3f, dth=%.3f",
                dt, delta_x, delta_y, delta_th);
        ROS_INFO("Current pose: x=%.3f, y=%.3f, th=%.3f", x_, y_, th_);
    }
    
    // 发布TF变换和里程计消息
    publishOdometry(current_time, v, steering_angle);
    
    // 更新时间戳
    last_time_ = current_time;
    
    // 防止计数器溢出
    if (odom_log_counter_ > 10000) {
        odom_log_counter_ = 0;
    }
}

bool OdomUpdater::updateFromControllerData(const std::vector<uint8_t>& data) {
    // 检查数据长度
    if (data.size() != FRAME_SIZE) {
        ROS_ERROR("OdomUpdater: Invalid data frame length: %zu (expected %d)", data.size(), FRAME_SIZE);
        return false;
    }
    
    // 检查帧头和帧尾
    if (data[0] != FRAME_HEADER || data[FRAME_SIZE - 1] != FRAME_TAIL) {
        ROS_ERROR("OdomUpdater: Invalid frame header/tail: 0x%02X/0x%02X (expected 0x%02X/0x%02X)",
                data[0], data[FRAME_SIZE - 1], FRAME_HEADER, FRAME_TAIL);
        return false;
    }
    
    // 解析上行数据帧
    if (!parseUplinkFrame(data)) {
        ROS_ERROR("OdomUpdater: Frame parse error or checksum verification failed");
        return false;
    }
    
    // 计算时间增量
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_time_).toSec();
    
    // 防止时间增量过小
    if (dt < 0.0001) {
        ROS_WARN_THROTTLE(1.0, "OdomUpdater: Time increment too small (dt=%.6f)", dt);
        last_time_ = current_time;
        return true;
    }
    
    // 只有当我们使用控制器数据作为里程计方法时才更新位置
    if (odom_method_ == CONTROLLER_DATA) {
        // 计算位置增量
        double delta_x = 0.0;
        double delta_y = 0.0;
        double delta_th = 0.0;
        
        // 更新位置和朝向
        updateWithControllerData(dt, delta_x, delta_y, delta_th);
        
        // 更新位置
        x_ += delta_x;
        y_ += delta_y;
        th_ += delta_th;
        
        // 标准化角度在-π到π之间
        th_ = normalizeAngle(th_);
        
        // 发布TF变换和里程计消息
        publishOdometry(current_time, controller_linear_x_, 0.0);
    }
    
    // 更新时间戳
    last_time_ = current_time;
    
    return true;
}

void OdomUpdater::updateEncoderOnly(double v, double steering_angle, double dt, 
                                   double& delta_x, double& delta_y, double& delta_th) {
    // 使用简单的运动学模型计算位置变化
    delta_x = v * cos(th_) * dt;
    delta_y = v * sin(th_) * dt;
    delta_th = v * tan(steering_angle) / wheelbase_ * dt;
    
    // 更新全局速度（用于对外提供速度信息）
    velocity_x_ = v * cos(th_);
    velocity_y_ = v * sin(th_);
    angular_velocity_z_ = v * tan(steering_angle) / wheelbase_;
    
    ROS_DEBUG("Encoder only update: v=%.3f, steering=%.3f, dt=%.3f", 
             v, steering_angle, dt);
}

void OdomUpdater::updateWithIMUFusion(double v, double steering_angle, double dt,
                                     double& delta_x, double& delta_y, double& delta_th) {
    // 使用IMU的角速度更新航向角
    delta_th = angular_velocity_z_ * dt;
    
    // 通过车轮编码器计算位移
    double encoder_delta_x = v * cos(th_) * dt;
    double encoder_delta_y = v * sin(th_) * dt;
    
    // 使用加速度的二次积分更新位置
    double imu_delta_v_x = acceleration_x_ * dt;
    double imu_delta_v_y = acceleration_y_ * dt;
    double imu_delta_x = velocity_x_ * dt + 0.5 * acceleration_x_ * dt * dt;
    double imu_delta_y = velocity_y_ * dt + 0.5 * acceleration_y_ * dt * dt;
    
    // 更新速度
    velocity_x_ += imu_delta_v_x;
    velocity_y_ += imu_delta_v_y;
    
    // 简单低通滤波减少漂移
    velocity_x_ *= 0.95;
    velocity_y_ *= 0.95;
    
    // 使用互补滤波融合位置数据
    delta_x = fusion_alpha_ * encoder_delta_x + (1 - fusion_alpha_) * imu_delta_x;
    delta_y = fusion_alpha_ * encoder_delta_y + (1 - fusion_alpha_) * imu_delta_y;
    
    // 融合角速度数据
    double encoder_delta_th = v * tan(steering_angle) / wheelbase_ * dt;
    delta_th = fusion_alpha_ * encoder_delta_th + (1 - fusion_alpha_) * delta_th;
    
    ROS_DEBUG("IMU fusion update: v=%.3f, accel=(%.3f,%.3f), ang_vel=%.3f",
             v, acceleration_x_, acceleration_y_, angular_velocity_z_);
}

void OdomUpdater::updateWithControllerData(double dt,
                                         double& delta_x, double& delta_y, double& delta_th) {
    // 使用控制器上报的速度直接更新位置
    
    // 计算在世界坐标系下的速度
    velocity_x_ = controller_linear_x_ * cos(th_) - controller_linear_y_ * sin(th_);
    velocity_y_ = controller_linear_x_ * sin(th_) + controller_linear_y_ * cos(th_);
    angular_velocity_z_ = controller_angular_z_;
    
    // 计算位移增量
    delta_x = velocity_x_ * dt;
    delta_y = velocity_y_ * dt;
    delta_th = angular_velocity_z_ * dt;
    
    ROS_DEBUG("Controller data update: v_x=%.3f, v_y=%.3f, omega=%.3f, dt=%.3f",
            controller_linear_x_, controller_linear_y_, controller_angular_z_, dt);
}

bool OdomUpdater::parseUplinkFrame(const std::vector<uint8_t>& data) {
    // 检查帧头和帧尾（虽然在updateFromControllerData中已检查，但为了安全起见这里再检查一次）
    if (data[0] != FRAME_HEADER || data[FRAME_SIZE - 1] != FRAME_TAIL) {
        return false;
    }
    
    // 校验和验证
    if (!verifyChecksum(data)) {
        ROS_ERROR("OdomUpdater: Frame checksum verification failed");
        return false;
    }
    
    // 提取停止标志位
    controller_flag_stop_ = data[1];
    
    // 提取并转换线速度（短整数，大端序）
    controller_linear_x_ = extractShort(data, 2) / controller_scale_;
    controller_linear_y_ = extractShort(data, 4) / controller_scale_;
    controller_linear_z_ = extractShort(data, 6) / controller_scale_;
    
    // 提取并转换角速度（短整数，大端序）
    controller_angular_x_ = extractShort(data, 8) / controller_scale_;
    controller_angular_y_ = extractShort(data, 10) / controller_scale_;
    controller_angular_z_ = extractShort(data, 12) / controller_scale_;
    
    // 提取里程计计数（短整数，大端序）
    controller_odom1_ = extractShort(data, 14);
    controller_odom2_ = extractShort(data, 16);
    
    // 提取电池电压（短整数，大端序，位置是20-21）
    controller_battery_ = extractShort(data, 20) / 100.0;  // 假设单位是0.01V
    
    // 记录日志（throttled，每秒最多一次）
    ROS_DEBUG_THROTTLE(1.0, "控制器数据: 速度=(%.3f,%.3f,%.3f), 角速度=(%.3f,%.3f,%.3f), 里程计=(%d,%d), 电池=%.2fV, 标志位=%d",
                     controller_linear_x_, controller_linear_y_, controller_linear_z_,
                     controller_angular_x_, controller_angular_y_, controller_angular_z_,
                     controller_odom1_, controller_odom2_,
                     controller_battery_, controller_flag_stop_);
    
    return true;
}

int16_t OdomUpdater::extractShort(const std::vector<uint8_t>& data, size_t start_index) const {
    // 检查索引是否有效
    if (start_index + 1 >= data.size()) {
        ROS_ERROR("OdomUpdater: Invalid index for extractShort: %zu (max %zu)",
                start_index, data.size() - 2);
        return 0;
    }
    
    // 从大端序转换为本地字节序
    return (static_cast<int16_t>(data[start_index]) << 8) | static_cast<int16_t>(data[start_index + 1]);
}

bool OdomUpdater::verifyChecksum(const std::vector<uint8_t>& data) const {
    // 检查数据大小
    if (data.size() != FRAME_SIZE) {
        ROS_ERROR("OdomUpdater: Invalid data size for checksum verification: %zu", data.size());
        return false;
    }
    
    // 计算校验和（使用异或XOR，而不是求和）
    uint8_t calculated_checksum = 0;
    for (size_t i = 0; i < 22; ++i) {
        calculated_checksum ^= data[i];  // 使用异或运算
    }
    
    // 获取数据帧中的校验和
    uint8_t frame_checksum = data[22];
    
    // 比较
    if (calculated_checksum != frame_checksum) {
        ROS_ERROR("OdomUpdater: Checksum mismatch: calculated 0x%02X, frame 0x%02X",
                calculated_checksum, frame_checksum);
        return false;
    }
    
    return true;
}

void OdomUpdater::publishOdometry(const ros::Time& current_time, double v, double steering_angle) {
    // 发布TF变换
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = x_;
    odom_trans.transform.translation.y = y_;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(th_);
    odom_broadcaster_.sendTransform(odom_trans);
    
    // 创建并填充里程计消息
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    
    // 设置位置
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th_);
    
    // 设置速度
    if (odom_method_ == IMU_FUSION || odom_method_ == CONTROLLER_DATA) {
        // 使用IMU融合的速度
        odom.twist.twist.linear.x = velocity_x_;
        odom.twist.twist.linear.y = velocity_y_;
        odom.twist.twist.angular.z = angular_velocity_z_;
    } else {
        // 使用编码器计算的速度
        odom.twist.twist.linear.x = v;
        odom.twist.twist.angular.z = v * tan(steering_angle) / wheelbase_;
    }
    
    // 填充协方差矩阵
    for (int i = 0; i < 36; i++) {
        odom.pose.covariance[i] = 0.0;
        odom.twist.covariance[i] = 0.0;
    }
    
    // 位置协方差
    odom.pose.covariance[0] = 0.01;  // x
    odom.pose.covariance[7] = 0.01;  // y
    odom.pose.covariance[35] = 0.01; // theta
    
    // 速度协方差
    odom.twist.covariance[0] = 0.01;  // vx
    odom.twist.covariance[7] = 0.01;  // vy
    odom.twist.covariance[35] = 0.01; // vtheta
    
    // 发布里程计消息
    odom_pub_.publish(odom);
}

double OdomUpdater::normalizeAngle(double angle) {
    // 标准化角度在[-π, π]区间内
    return fmod(angle + M_PI, 2*M_PI) - M_PI;
}

void OdomUpdater::reset() {
    x_ = 0.0;
    y_ = 0.0;
    th_ = 0.0;
    velocity_x_ = 0.0;
    velocity_y_ = 0.0;
    last_v_ = 0.0;
    last_steering_angle_ = 0.0;
    last_time_ = ros::Time::now();
    ROS_INFO("Odometry reset to origin");
}

void OdomUpdater::getPose(double& x, double& y, double& th) const {
    x = x_;
    y = y_;
    th = th_;
}

void OdomUpdater::getVelocity(double& v_x, double& v_y, double& omega) const {
    if (odom_method_ == IMU_FUSION || odom_method_ == CONTROLLER_DATA) {
        v_x = velocity_x_;
        v_y = velocity_y_;
        omega = angular_velocity_z_;
    } else {
        v_x = last_v_ * cos(th_);
        v_y = last_v_ * sin(th_);
        omega = last_v_ * tan(last_steering_angle_) / wheelbase_;
    }
}

void OdomUpdater::getIMUData(double& angular_vel, double& accel_x, double& accel_y) const {
    if (odom_method_ == CONTROLLER_DATA) {
        angular_vel = controller_angular_z_;
        accel_x = 0.0;  // 控制器数据中没有加速度信息，可以考虑求导获得
        accel_y = 0.0;
    } else if (use_imu_data_) {
        angular_vel = angular_velocity_z_;
        accel_x = acceleration_x_;
        accel_y = acceleration_y_;
    } else {
        angular_vel = 0.0;
        accel_x = 0.0;
        accel_y = 0.0;
    }
}

void OdomUpdater::setOdomMethod(const std::string& method) {
    if (method == "encoder_only") {
        odom_method_ = ENCODER_ONLY;
        ROS_INFO("Switched to encoder_only odometry method");
    } else if (method == "imu_fusion") {
        if (!use_imu_data_) {
            use_imu_data_ = true;
            if (!imu_sub_) {
                imu_sub_ = nh_.subscribe("imu/data", 10, &OdomUpdater::imuCallback, this);
            }
        }
        odom_method_ = IMU_FUSION;
        ROS_INFO("Switched to imu_fusion odometry method");
    } else if (method == "controller_data") {
        odom_method_ = CONTROLLER_DATA;
        ROS_INFO("Switched to controller_data odometry method");
    } else {
        ROS_WARN("Unknown odometry method '%s', staying with current method", method.c_str());
    }
}

std::string OdomUpdater::getOdomMethod() const {
    switch (odom_method_) {
        case ENCODER_ONLY: return "encoder_only";
        case IMU_FUSION: return "imu_fusion";
        case CONTROLLER_DATA: return "controller_data";
        case CUSTOM_METHOD: return "custom_method";
        default: return "unknown";
    }
}

double OdomUpdater::getBatteryVoltage() const {
    return controller_battery_;
}

uint8_t OdomUpdater::getStopFlag() const {
    return controller_flag_stop_;
} 