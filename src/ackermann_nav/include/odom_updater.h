#ifndef ODOM_UPDATER_H
#define ODOM_UPDATER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <string>

/**
 * @brief 里程计数据更新器类
 * 
 * 该类负责计算和发布机器人的里程计信息，支持多种计算方法：
 * 1. 仅使用编码器数据（计算法）
 * 2. 融合IMU和编码器数据
 */
class OdomUpdater {
public:
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
     * @param method 方法名称："encoder_only" 或 "imu_fusion"
     */
    void setOdomMethod(const std::string& method);
    
    /**
     * @brief 获取当前使用的里程计方法
     * @return 方法名称字符串
     */
    std::string getOdomMethod() const;

private:
    // 计算方法选择
    enum OdomMethod {
        ENCODER_ONLY,    // 仅使用编码器(计算)
        IMU_FUSION,      // IMU和编码器融合
        CUSTOM_METHOD    // 预留的自定义方法
    };
    
    // ROS相关
    ros::NodeHandle nh_;
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
    
    // 时间相关
    ros::Time last_time_;
    
    // 配置参数
    bool use_imu_data_;             // 是否使用IMU数据进行融合
    double fusion_alpha_;           // 数据融合参数 (0-1)
    OdomMethod odom_method_;        // 当前使用的里程计方法
    
    // 日志控制
    int odom_log_counter_;
    const int ODOM_LOG_INTERVAL = 100;
    
    // IMU数据回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    
    // 原始编码器计算方法（最早的计算方法）
    void updateEncoderOnly(double v, double steering_angle, double dt, 
                           double& delta_x, double& delta_y, double& delta_th);
    
    // IMU融合方法
    void updateWithIMUFusion(double v, double steering_angle, double dt,
                             double& delta_x, double& delta_y, double& delta_th);
    
    // 发布TF变换和里程计消息
    void publishOdometry(const ros::Time& current_time, double v, double steering_angle);
    
    // 将角度标准化到[-π,π]范围
    double normalizeAngle(double angle);
};

#endif // ODOM_UPDATER_H 