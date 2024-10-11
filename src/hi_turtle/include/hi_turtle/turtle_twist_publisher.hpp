#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>

class TurtleTwistPublisher : public BT::RosTopicPubNode<geometry_msgs::msg::Twist> {
public:
    TurtleTwistPublisher(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params);

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<double>("speed")
        };
    }

    bool setMessage(geometry_msgs::msg::Twist& msg) override;

    bool calculate_movement(geometry_msgs::msg::Twist& twist);

private:
    double get_angle_diff() const;
    double get_distance() const;

    double normalize_angle(double angle) const;

    double target_x_, target_y_, speed_;
    std::string topic_name_;
    BT::Blackboard::Ptr blackboard_;
    turtlesim::msg::Pose current_pose_;
};
