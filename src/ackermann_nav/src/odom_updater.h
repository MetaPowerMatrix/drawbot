#ifndef ODOM_UPDATER_H
#define ODOM_UPDATER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <vector>
#include <cstdint>
#include <queue>
#include <deque>
#include <cmath>
#include <mutex>

// 添加命名空间前缀避免宏定义冲突
#define ODOM_UPDATER_LOG_INTERVAL 100

// 帧头和帧尾标识
#define FRAME_HEADER 0x7B
#define FRAME_TAIL 0x7D
#define FRAME_SIZE 24  // 上行数据帧大小为24字节

/**
 * @brief 里程计更新器类，负责处理里程计数据的计算和发布
 * 
 * 该类提供了多种里程计计算方法，包括仅使用编码器数据的基本方法和
 * 融合IMU数据的高级方法。可以通过ROS参数或动态方法切换。
 */
class OdomUpdater {
public:
    /**
     * @brief 里程计计算方法枚举
     */
    enum OdomMethod {
        ENCODER_ONLY,  ///< 仅使用编码器数据
        IMU_FUSION,    ///< 融合IMU数据
        CONTROLLER_DATA, ///< 使用控制器上行数据
        CUSTOM_METHOD  ///< 自定义方法
    };

    /**
     * @brief 构造函数
     * @param nh ROS节点句柄
     */
    OdomUpdater(ros::NodeHandle& nh);

    /**
     * @brief 更新里程计数据
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
     * @brief 重置里程计数据到原点
     */
    void reset();

    /**
     * @brief 获取当前位姿
     * @param x 输出参数：x坐标 (m)
     * @param y 输出参数：y坐标 (m)
     * @param th 输出参数：航向角 (rad)
     */
    void getPose(double& x, double& y, double& th) const;

    /**
     * @brief 获取当前速度
     * @param v_x 输出参数：x方向速度 (m/s)
     * @param v_y 输出参数：y方向速度 (m/s)
     * @param omega 输出参数：角速度 (rad/s)
     */
    void getVelocity(double& v_x, double& v_y, double& omega) const;

    /**
     * @brief 获取IMU数据
     * @param angular_vel 输出参数：角速度 (rad/s)
     * @param accel_x 输出参数：x方向加速度 (m/s^2)
     * @param accel_y 输出参数：y方向加速度 (m/s^2)
     */
    void getIMUData(double& angular_vel, double& accel_x, double& accel_y) const;

    /**
     * @brief 设置里程计计算方法
     * @param method 方法名称 ("encoder_only"、"imu_fusion"或"controller_data")
     */
    void setOdomMethod(const std::string& method);

    /**
     * @brief 获取当前里程计计算方法
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
    tf::TransformBroadcaster odom_broadcaster_;
    ros::Publisher odom_pub_;
    ros::Subscriber imu_sub_;
    ros::Time last_time_;

    // 机器人参数
    double wheelbase_;  ///< 轴距

    // 位置和姿态
    double x_;    ///< x坐标 (m)
    double y_;    ///< y坐标 (m)
    double th_;   ///< 航向角 (rad)

    // 速度和加速度
    double velocity_x_;   ///< x方向速度 (m/s)
    double velocity_y_;   ///< y方向速度 (m/s)
    double last_v_;       ///< 上一次线速度 (m/s)
    double last_steering_angle_;  ///< 上一次转向角 (rad)
    
    // IMU数据
    double angular_velocity_z_;     ///< 角速度 (rad/s)
    double linear_acceleration_x_;  ///< x方向加速度，车体坐标系 (m/s^2)
    double linear_acceleration_y_;  ///< y方向加速度，车体坐标系 (m/s^2)
    double acceleration_x_;         ///< x方向加速度，世界坐标系 (m/s^2)
    double acceleration_y_;         ///< y方向加速度，世界坐标系 (m/s^2)

    // 控制器数据
    double controller_linear_x_;    ///< 控制器上报的X轴线速度 (m/s)
    double controller_linear_y_;    ///< 控制器上报的Y轴线速度 (m/s)
    double controller_linear_z_;    ///< 控制器上报的Z轴线速度 (m/s)
    double controller_angular_x_;   ///< 控制器上报的X轴角速度 (rad/s)
    double controller_angular_y_;   ///< 控制器上报的Y轴角速度 (rad/s)
    double controller_angular_z_;   ///< 控制器上报的Z轴角速度 (rad/s)
    int16_t controller_odom1_;      ///< 控制器上报的里程计值1
    int16_t controller_odom2_;      ///< 控制器上报的里程计值2
    double controller_battery_;     ///< 控制器上报的电池电压 (V)
    uint8_t controller_flag_stop_;  ///< 控制器停止标志位

    // 数据流处理相关
    std::deque<uint8_t> data_buffer_;  ///< 数据缓冲区
    bool frame_in_sync_;               ///< 数据帧同步状态
    int consecutive_valid_frames_;     ///< 连续有效帧计数
    int consecutive_invalid_frames_;   ///< 连续无效帧计数
    int frames_processed_;             ///< 已处理帧计数
    int valid_frames_;                 ///< 有效帧计数
    int invalid_frames_;               ///< 无效帧计数

    // 配置参数
    bool use_imu_data_;    ///< 是否使用IMU数据
    double fusion_alpha_;  ///< 融合系数 (0-1)，值越大越信任编码器
    OdomMethod odom_method_;  ///< 当前里程计计算方法
    int odom_log_counter_;    ///< 日志计数器
    double controller_scale_; ///< 控制器数据缩放因子

    /**
     * @brief IMU数据回调函数
     * @param msg IMU消息
     */
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    /**
     * @brief 使用编码器数据更新里程计
     * @param v 线速度 (m/s)
     * @param steering_angle 转向角 (rad)
     * @param dt 时间增量 (s)
     * @param delta_x 输出参数：x方向位移增量 (m)
     * @param delta_y 输出参数：y方向位移增量 (m)
     * @param delta_th 输出参数：航向角增量 (rad)
     */
    void updateEncoderOnly(double v, double steering_angle, double dt,
                         double& delta_x, double& delta_y, double& delta_th);

    /**
     * @brief 使用IMU融合方法更新里程计
     * @param v 线速度 (m/s)
     * @param steering_angle 转向角 (rad)
     * @param dt 时间增量 (s)
     * @param delta_x 输出参数：x方向位移增量 (m)
     * @param delta_y 输出参数：y方向位移增量 (m)
     * @param delta_th 输出参数：航向角增量 (rad)
     */
    void updateWithIMUFusion(double v, double steering_angle, double dt,
                          double& delta_x, double& delta_y, double& delta_th);
                          
    /**
     * @brief 使用控制器数据更新里程计
     * @param dt 时间增量 (s)
     * @param delta_x 输出参数：x方向位移增量 (m)
     * @param delta_y 输出参数：y方向位移增量 (m)
     * @param delta_th 输出参数：航向角增量 (rad)
     */
    void updateWithControllerData(double dt,
                              double& delta_x, double& delta_y, double& delta_th);

    /**
     * @brief 查找数据缓冲区中的帧头位置
     * @return 帧头位置的索引，如果未找到则返回-1
     */
    int findFrameHeader();

    /**
     * @brief 从数据缓冲区中提取完整的数据帧
     * @param frame 输出参数：提取的完整数据帧
     * @return 是否成功提取完整数据帧
     */
    bool extractFrame(std::vector<uint8_t>& frame);

    /**
     * @brief 解析上行数据帧
     * @param data 上行数据帧原始字节数组
     * @return 是否成功解析数据
     */
    bool parseUplinkFrame(const std::vector<uint8_t>& data);

    /**
     * @brief 从原始数据中提取短整数值（大端序）
     * @param data 原始数据数组
     * @param start_index 起始索引
     * @return 解析出的短整数值
     */
    int16_t extractShort(const std::vector<uint8_t>& data, size_t start_index) const;

    /**
     * @brief 验证上行数据帧校验和
     * @param data 原始数据数组
     * @return 校验和是否正确
     */
    bool verifyChecksum(const std::vector<uint8_t>& data) const;

    /**
     * @brief 发布里程计消息和TF变换
     * @param current_time 当前时间
     * @param v 线速度 (m/s)
     * @param steering_angle 转向角 (rad)
     */
    void publishOdometry(const ros::Time& current_time, double v, double steering_angle);

    /**
     * @brief 将角度标准化到[-π, π]区间
     * @param angle 输入角度 (rad)
     * @return 标准化后的角度 (rad)
     */
    double normalizeAngle(double angle);
};

#endif // ODOM_UPDATER_H 