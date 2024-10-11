#include <behaviortree_ros2/bt_utils.hpp>
#include "turtlesim/msg/pose.hpp"

class StopOnceAchieved : public BT::DecoratorNode
{
public:
    StopOnceAchieved(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("target_x"),
            BT::InputPort<double>("target_y")
        };
    }

    BT::NodeStatus tick() override;

private:
    turtlesim::msg::Pose current_pose_;
    BT::Blackboard::Ptr blackboard_;
    double target_x_, target_y_;
};