#include "hi_turtle/turtle_pose_subscriber.hpp"
#include "hi_turtle/map_to_string.hpp"

TurtlePoseSubscriber::TurtlePoseSubscriber(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params) :
    RosTopicSubNode<turtlesim::msg::Pose>(name, config, params), config_(config)
{
    auto node = node_.lock();
    RCLCPP_INFO(node->get_logger(), "%s's Input Port: %s", name.c_str(), mapToString(config.input_ports).c_str());
    if (!getInput<std::string>("topic_name", topic_name_)) {
        RCLCPP_ERROR(node->get_logger(), "TurtlePoseSubscriber failed to get parameters from input port.");
        throw std::runtime_error("Parameter retrieval failed");
    }
    blackboard_ = config.blackboard;
}

BT::NodeStatus TurtlePoseSubscriber::onTick(const std::shared_ptr<turtlesim::msg::Pose>& msg) {
    if (!msg) {
        return BT::NodeStatus::FAILURE;
    }
    blackboard_->set<turtlesim::msg::Pose>("current_pose", *msg);
    return BT::NodeStatus::SUCCESS;
}