#include <hi_turtle/stop_once_achieved.hpp>
#include <hi_turtle/map_to_string.hpp>
#include <hi_turtle/config.hpp>

StopOnceAchieved::StopOnceAchieved(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params) : 
    BT::DecoratorNode(name, config)
{
    current_pose_.set__x(MAXFLOAT / 2);
    current_pose_.set__y(MAXFLOAT / 2);
    blackboard_ = config.blackboard;
    auto node = params.nh.lock();
    RCLCPP_INFO(node->get_logger(), "%s's Input Port: %s", name.c_str(), mapToString(config.input_ports).c_str());
    if (!getInput<double>("target_x", target_x_) ||
        !getInput<double>("target_y", target_y_)
    ) {
        RCLCPP_ERROR(node->get_logger(), "TurtleTwistPublisher failed to get parameters from input port.");
        throw std::runtime_error("Parameter retrieval failed");
    }
    RCLCPP_INFO(node->get_logger(), "Target Position: (%f, %f)", target_x_, target_y_);
    blackboard_ = config.blackboard;
    blackboard_->set("target_x", target_x_);
    blackboard_->set("target_y", target_y_);
}

BT::NodeStatus StopOnceAchieved::tick() {
    if (!blackboard_->get<turtlesim::msg::Pose>("current_pose", current_pose_)) {}
    double distance = std::hypot(current_pose_.x - target_x_, current_pose_.y - target_y_);
    if (distance < DISTANCE_TOLERANCE)
    {
        return BT::NodeStatus::SUCCESS; // 达到目标
    }
    if (child_node_->executeTick() == BT::NodeStatus::SUCCESS) {
        
        return BT::NodeStatus::RUNNING;
    }
    return BT::NodeStatus::FAILURE;
}