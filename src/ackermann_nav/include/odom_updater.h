#ifndef ODOM_UPDATER_H
#define ODOM_UPDATER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <cstdint>
#include <queue>
#include <deque>
#include <cmath>
#include <mutex>

// 帧头和帧尾标识
#define FRAME_HEADER 0x7B
#define FRAME_TAIL 0x7D
#define FRAME_SIZE 24  // 上行数据帧大小为24字节

/**
 * @brief 里程计数据更新器类
 * 
 * 该类负责计算和发布机器人的里程计信息，支持多种计算方法：
 * 1. 仅使用编码器数据（计算法）
 * 2. 融合IMU和编码器数据
 * 3. 使用控制器上行数据
 */
class OdomUpdater {
public:
    /**
     * @brief 里程计计算方法枚举
     */
    enum OdomMethod {
        ENCODER_ONLY,    // 仅使用编码器(计算)
        IMU_FUSION,      // IMU和编码器融合
        CONTROLLER_DATA, // 使用控制器上行数据
        CUSTOM_METHOD    // 预留的自定义方法
    };

    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    OdomUpdater(ros::NodeHandle& nh);
    
    /**
     * @brief 使用最新的速度和转向角更新里程计
     * @param v 线速度 (m/s)
     * @param steering_angle 转向角 (rad)
     */
    void update(double v, double steering_angle);

    /**
     * @brief 处理控制器上行数据流
     * @param data 接收到的原始数据
     * @return 是否成功解析到至少一帧有效数据
     */
    bool processControllerDataStream(const std::vector<uint8_t>& data);

    /**
     * @brief 使用控制器上行数据更新里程计
     * @param data 上行数据帧，24字节的二进制数据
     * @return 是否成功解析数据
     */
    bool updateFromControllerData(const std::vector<uint8_t>& data);
    
    /**
     * @brief 重置里程计到原点
     */
    void reset();
    
    /**
     * @brief 获取当前位置和朝向
     * @param x 输出参数，当前x坐标
     * @param y 输出参数，当前y坐标
     * @param th 输出参数，当前朝向角(rad)
     */
    void getPose(double& x, double& y, double& th) const;
    
    /**
     * @brief 获取当前速度
     * @param v_x 输出参数，x方向速度
     * @param v_y 输出参数，y方向速度
     * @param omega 输出参数，角速度
     */
    void getVelocity(double& v_x, double& v_y, double& omega) const;
    
    /**
     * @brief 获取IMU数据
     * @param angular_vel 输出参数，角速度
     * @param accel_x 输出参数，x方向加速度
     * @param accel_y 输出参数，y方向加速度
     */
    void getIMUData(double& angular_vel, double& accel_x, double& accel_y) const;
    
    /**
     * @brief 切换里程计计算方法
     * @param method 方法名称："encoder_only"、"imu_fusion"或"controller_data"
     */
    void setOdomMethod(const std::string& method);
    
    /**
     * @brief 获取当前使用的里程计方法
     * @return 方法名称字符串
     */
    std::string getOdomMethod() const;

    /**
     * @brief 获取电池电压信息
     * @return 当前电池电压（伏特）
     */
    double getBatteryVoltage() const;

    /**
     * @brief 获取停止标志位
     * @return 停止标志位值
     */
    uint8_t getStopFlag() const;

private:
    // ROS相关
    ros::NodeHandle& nh_;
    ros::Publisher odom_pub_;
    ros::Subscriber imu_sub_;
    tf::TransformBroadcaster odom_broadcaster_;
    
    // 机器人参数
    double wheelbase_;
    
    // 位置和姿态
    double x_, y_, th_;
    
    // 速度信息
    double velocity_x_;
    double velocity_y_;
    double last_v_;
    double last_steering_angle_;
    
    // IMU数据
    double angular_velocity_z_;     // z轴角速度
    double linear_acceleration_x_;  // x轴加速度
    double linear_acceleration_y_;  // y轴加速度
    double acceleration_x_;         // 世界坐标系下的x轴加速度
    double acceleration_y_;         // 世界坐标系下的y轴加速度

    // 控制器数据
    double controller_linear_x_;    // 控制器上报的X轴线速度 (m/s)
    double controller_linear_y_;    // 控制器上报的Y轴线速度 (m/s)
    double controller_linear_z_;    // 控制器上报的Z轴线速度 (m/s)
    double controller_angular_x_;   // 控制器上报的X轴角速度 (rad/s)
    double controller_angular_y_;   // 控制器上报的Y轴角速度 (rad/s)
    double controller_angular_z_;   // 控制器上报的Z轴角速度 (rad/s)
    int16_t controller_odom1_;      // 控制器上报的里程计值1
    int16_t controller_odom2_;      // 控制器上报的里程计值2
    double controller_battery_;     // 控制器上报的电池电压 (V)
    uint8_t controller_flag_stop_;  // 控制器停止标志位
    
    // 时间相关
    ros::Time last_time_;

    // 数据流处理相关
    std::deque<uint8_t> data_buffer_;  // 数据缓冲区
    bool frame_in_sync_;               // 数据帧同步状态
    int consecutive_valid_frames_;     // 连续有效帧计数
    int consecutive_invalid_frames_;   // 连续无效帧计数
    int frames_processed_;             // 已处理帧计数
    int valid_frames_;                 // 有效帧计数
    int invalid_frames_;               // 无效帧计数
    
    // 配置参数
    bool use_imu_data_;             // 是否使用IMU数据进行融合
    double fusion_alpha_;           // 数据融合参数 (0-1)
    OdomMethod odom_method_;        // 当前使用的里程计方法
    int odom_log_counter_;          // 日志计数器
    double controller_scale_;       // 控制器数据缩放因子
    
    // IMU数据回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    
    // 原始编码器计算方法（最早的计算方法）
    void updateEncoderOnly(double v, double steering_angle, double dt, 
                           double& delta_x, double& delta_y, double& delta_th);
    
    // IMU融合方法
    void updateWithIMUFusion(double v, double steering_angle, double dt,
                             double& delta_x, double& delta_y, double& delta_th);

    // 使用控制器数据更新里程计
    void updateWithControllerData(double dt,
                                 double& delta_x, double& delta_y, double& delta_th);
    
    // 发布TF变换和里程计消息
    void publishOdometry(const ros::Time& current_time, double v, double steering_angle);

    // 查找数据缓冲区中的帧头位置
    int findFrameHeader();

    // 从数据缓冲区中提取完整的数据帧
    bool extractFrame(std::vector<uint8_t>& frame);

    // 解析上行数据帧
    bool parseUplinkFrame(const std::vector<uint8_t>& data);

    // 从原始数据中提取短整数值（大端序）
    int16_t extractShort(const std::vector<uint8_t>& data, size_t start_index) const;

    // 验证上行数据帧校验和
    bool verifyChecksum(const std::vector<uint8_t>& data) const;
    
    // 将角度标准化到[-π,π]范围
    double normalizeAngle(double angle);
};

#endif // ODOM_UPDATER_H 