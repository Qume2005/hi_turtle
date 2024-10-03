#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <turtlesim/msg/pose.hpp>

class Movement {
public:
    static geometry_msgs::msg::Twist calculate_pose(const turtlesim::msg::Pose &current_pose, const turtlesim::msg::Pose &target_pose);
    static double get_distance(const turtlesim::msg::Pose &current_pose, const turtlesim::msg::Pose &target_pose);
    static geometry_msgs::msg::Twist stopping_pose();
    static double get_angle_diff(const turtlesim::msg::Pose &current_pose, const turtlesim::msg::Pose &target_pose);
    static double normalize_angle(double angle);
};

#endif // MOVEMENT_HPP
