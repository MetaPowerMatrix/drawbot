#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <cmath>

class PathFollower {
private:
    ros::NodeHandle nh_;
    ros::Subscriber path_sub_;
    ros::Publisher cmd_vel_pub_;
    tf::TransformListener tf_listener_;

    nav_msgs::Path current_path_;
    bool has_path_;
    double goal_tolerance_;
    double lookahead_distance_;
    double max_linear_speed_;
    double max_angular_speed_;

public:
    PathFollower() : has_path_(false) {
        // 加载参数
        nh_.param<double>("goal_tolerance", goal_tolerance_, 0.1);
        nh_.param<double>("lookahead_distance", lookahead_distance_, 0.5);
        nh_.param<double>("max_linear_speed", max_linear_speed_, 0.5);
        nh_.param<double>("max_angular_speed", max_angular_speed_, 1.0);

        // 订阅路径和发布速度命令
        path_sub_ = nh_.subscribe("path", 1, &PathFollower::pathCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    }

    void pathCallback(const nav_msgs::Path::ConstPtr& path) {
        current_path_ = *path;
        has_path_ = true;
        ROS_INFO("接收到新的路径，包含 %lu 个路点", path->poses.size());
    }

    bool getTransform(tf::StampedTransform& transform) {
        try {
            tf_listener_.lookupTransform("odom", "base_link",
                                        ros::Time(0), transform);
            return true;
        } catch (tf::TransformException& ex) {
            ROS_WARN("%s", ex.what());
            return false;
        }
    }

    int findNearestPoint(const tf::StampedTransform& robot_pose) {
        int nearest_point = 0;
        double min_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < current_path_.poses.size(); i++) {
            double dx = current_path_.poses[i].pose.position.x - robot_pose.getOrigin().x();
            double dy = current_path_.poses[i].pose.position.y - robot_pose.getOrigin().y();
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < min_distance) {
                min_distance = distance;
                nearest_point = i;
            }
        }

        return nearest_point;
    }

    geometry_msgs::Point findLookaheadPoint(const tf::StampedTransform& robot_pose,
                                          int start_index) {
        geometry_msgs::Point target;
        double accumulated_distance = 0.0;
        int i = start_index;

        while (i < current_path_.poses.size() - 1) {
            double dx = current_path_.poses[i+1].pose.position.x -
                        current_path_.poses[i].pose.position.x;
            double dy = current_path_.poses[i+1].pose.position.y -
                        current_path_.poses[i].pose.position.y;
            accumulated_distance += std::sqrt(dx * dx + dy * dy);

            if (accumulated_distance >= lookahead_distance_) {
                target = current_path_.poses[i+1].pose.position;
                break;
            }
            i++;
        }

        if (i == current_path_.poses.size() - 1) {
            target = current_path_.poses.back().pose.position;
        }

        return target;
    }

    void update() {
        if (!has_path_) return;

        tf::StampedTransform robot_pose;
        if (!getTransform(robot_pose)) return;

        // 找到最近的路径点
        int nearest_point = findNearestPoint(robot_pose);

        // 找到前瞻点
        geometry_msgs::Point target = findLookaheadPoint(robot_pose, nearest_point);

        // 计算到目标点的相对位置
        double dx = target.x - robot_pose.getOrigin().x();
        double dy = target.y - robot_pose.getOrigin().y();

        // 计算机器人当前朝向
        double robot_yaw = tf::getYaw(robot_pose.getRotation());

        // 计算到目标点的角度
        double target_angle = std::atan2(dy, dx);

        // 计算转向角度
        double angle_diff = target_angle - robot_yaw;
        if (angle_diff > M_PI) angle_diff -= 2 * M_PI;
        if (angle_diff < -M_PI) angle_diff += 2 * M_PI;

        // 计算距离
        double distance = std::sqrt(dx * dx + dy * dy);

        // 生成速度命令
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = std::min(max_linear_speed_ * (distance / lookahead_distance_),
                                  max_linear_speed_);
        cmd_vel.angular.z = std::min(std::max(angle_diff, -max_angular_speed_),
                                   max_angular_speed_);

        // 检查是否到达终点
        if (distance < goal_tolerance_ &&
            nearest_point >= current_path_.poses.size() - 1) {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            has_path_ = false;
            ROS_INFO("到达目标点");
        }

        cmd_vel_pub_.publish(cmd_vel);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_follower");
    PathFollower follower;
    ros::Rate rate(20);

    while (ros::ok()) {
        follower.update();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}