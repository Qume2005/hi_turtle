#pragma once

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <turtlesim/msg/pose.hpp>

class TurtlePoseSubscriber : public BT::RosTopicSubNode<turtlesim::msg::Pose>
{
public:
    TurtlePoseSubscriber(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name")
        };
    }

    BT::NodeStatus onTick(const std::shared_ptr<turtlesim::msg::Pose>& msg) override;
private:
    BT::Blackboard::Ptr blackboard_;
    std::string topic_name_;
    const BT::NodeConfig& config_;
};
