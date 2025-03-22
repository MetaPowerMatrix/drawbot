#ifndef ODOM_UPDATER_H
#define ODOM_UPDATER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>

// 定义里程计日志输出间隔（每多少次更新输出一次详细日志）
#define ODOM_LOG_INTERVAL 100

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
     * @param method 方法名称 ("encoder_only"或"imu_fusion")
     */
    void setOdomMethod(const std::string& method);

    /**
     * @brief 获取当前里程计计算方法
     * @return 方法名称字符串
     */
    std::string getOdomMethod() const;

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

    // 配置参数
    bool use_imu_data_;    ///< 是否使用IMU数据
    double fusion_alpha_;  ///< 融合系数 (0-1)，值越大越信任编码器
    OdomMethod odom_method_;  ///< 当前里程计计算方法
    int odom_log_counter_;    ///< 日志计数器

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