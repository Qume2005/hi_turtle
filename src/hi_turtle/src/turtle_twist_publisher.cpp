#include "hi_turtle/turtle_twist_publisher.hpp"
#include "hi_turtle/map_to_string.hpp"
#include "hi_turtle/config.hpp"

TurtleTwistPublisher::TurtleTwistPublisher(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
    : RosTopicPubNode<geometry_msgs::msg::Twist>(name, config, params)
{
    RCLCPP_INFO(node_->get_logger(), "%s's Input Port: %s", name.c_str(), mapToString(config.input_ports).c_str());
    if (!getInput<std::string>("topic_name", topic_name_) ||
        !getInput<double>("speed", speed_)
    ) {
        RCLCPP_ERROR(node_->get_logger(), "TurtleTwistPublisher failed to get parameters from input port.");
        throw std::runtime_error("Parameter retrieval failed");
    }
    blackboard_ = config.blackboard;
    if (!blackboard_->get<double>("target_x", target_x_) ||
        !blackboard_->get<double>("target_y", target_y_)
    ) {
        RCLCPP_ERROR(node_->get_logger(), "Pose not found in blackboard for target pose.");
        throw std::runtime_error("Blackboard retrieval failed");
    }
}


bool TurtleTwistPublisher::setMessage(geometry_msgs::msg::Twist& msg) {
    if (!blackboard_->get<turtlesim::msg::Pose>("current_pose", current_pose_)) {
        RCLCPP_ERROR(node_->get_logger(), "Pose not found in blackboard for current pose.");
        return false; // 数据不存在，返回失败
    }
    return calculate_movement(msg);
}

bool TurtleTwistPublisher::calculate_movement(geometry_msgs::msg::Twist& twist) {
    double distance = get_distance();
    if (distance < DISTANCE_TOLERANCE) {
        twist.linear.x = 0;
        twist.angular.z = 0;
        return true;
    }
    double angular = get_angle_diff();
    if (std::abs(angular) > ANGULAR_TOLERANCE) {
        twist.linear.x = 0;
        twist.angular.z = angular * 10;
    } else {
        twist.linear.x = speed_;
        twist.angular.z = 0;
    }
    return true;
}
double TurtleTwistPublisher::get_angle_diff() const {
    double angle_to_target = std::atan2(target_y_ - current_pose_.y, target_x_ - current_pose_.x);
    return normalize_angle(angle_to_target - current_pose_.theta);
}

double TurtleTwistPublisher::get_distance() const {
    return std::hypot(current_pose_.x - target_x_, current_pose_.y - target_y_);
}

double TurtleTwistPublisher::normalize_angle(double angle) const {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}