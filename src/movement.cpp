#include "hi_turtle/movement.hpp"
#include <cmath>

// 计算当前位置到目标位置的速度命令
geometry_msgs::msg::Twist Movement::calculate_pose(const turtlesim::msg::Pose &current_pose, const turtlesim::msg::Pose &target_pose) {
    geometry_msgs::msg::Twist cmd_vel;
    double distance = get_distance(current_pose, target_pose);
    double angle_diff = get_angle_diff(current_pose, target_pose);

    // 根据目标位置和角度差设置线速度和角速度
    if (distance < 0.01) { // 如果与目标非常接近，则停止
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.0;
    } else {
        cmd_vel.linear.x = std::min(1.0, distance); // 设置线速度，最大值为1.0，实际可以根据需求调整
        cmd_vel.angular.z = angle_diff; // 依据角度差进行调整
    }

    return cmd_vel;
}

// 计算当前位置与目标位置之间的距离
double Movement::get_distance(const turtlesim::msg::Pose &current_pose, const turtlesim::msg::Pose &target_pose) {
    return std::hypot(target_pose.x - current_pose.x, target_pose.y - current_pose.y);
}

// 返回指令来停止小车的运动
geometry_msgs::msg::Twist Movement::stopping_pose() {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;  // 线速度设为0
    cmd_vel.angular.z = 0.0; // 角速度设为0
    return cmd_vel;
}

// 计算当前方向与目标方向之间的角度差
double Movement::get_angle_diff(const turtlesim::msg::Pose &current_pose, const turtlesim::msg::Pose &target_pose) {
    double target_angle = std::atan2(target_pose.y - current_pose.y, target_pose.x - current_pose.x);
    double angle_diff = normalize_angle(target_angle - current_pose.theta);
    return angle_diff;
}

// 将角度规范化到 [-π, π] 范围内
double Movement::normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}
