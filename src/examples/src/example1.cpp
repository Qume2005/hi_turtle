#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <turtlesim/msg/pose.hpp>

using namespace BT;

class TurtlePoseSubscriber : public RosTopicSubNode<turtlesim::msg::Pose>
{
public:
    TurtlePoseSubscriber(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
    :   RosTopicSubNode<turtlesim::msg::Pose>(name, config, params)
    {
        auto node_ptr = node_.lock();
        if (!getInput<std::string>("topic_name", topic_name_) || !getInput<std::string>("turtle_name", turtle_name_)) {
            if (node_ptr) {
                RCLCPP_ERROR(node_ptr->get_logger(), "TurtlePoseSubscriber failed to get parameters from input port.");
            }
            throw std::runtime_error("Parameter retrieval failed");
        }
        blackboard_ = BT::Blackboard::create(config.blackboard);
    }

    static PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<std::string>("turtle_name")
        };
    }

    NodeStatus onTick(const std::shared_ptr<turtlesim::msg::Pose>& msg) override
    {
        if (!msg)
        {
            return NodeStatus::FAILURE;
        }
        blackboard_->set<turtlesim::msg::Pose>(turtle_name_, *msg);
        return NodeStatus::SUCCESS;

    }
private:
    BT::Blackboard::Ptr blackboard_;
    std::string topic_name_;
    std::string turtle_name_;
};

class TurtleTwistPublisher : public BT::RosTopicPubNode<geometry_msgs::msg::Twist> {
public:
    TurtleTwistPublisher(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
        : RosTopicPubNode<geometry_msgs::msg::Twist>(name, config, params), tolerance_(0.01)
    {
        if (
            !getInput<std::string>("topic_name", topic_name_)
            || !getInput<std::string>("turtle_name", turtle_name_)
            || !getInput<double>("target_x", target_x_)
            || !getInput<double>("target_y", target_y_)
            || !getInput<double>("speed", speed_)
        ) {
            RCLCPP_ERROR(node_->get_logger(), "TurtleTwistPublisher failed to get parameters from input port.");
            throw std::runtime_error("Parameter retrieval failed");
        }
        RCLCPP_INFO(node_->get_logger(), "Initializing MoveTurtleForward node");
        RCLCPP_INFO(node_->get_logger(), "Topic Name: %s", topic_name_.c_str());
        RCLCPP_INFO(node_->get_logger(), "Target Position: (%f, %f)", target_x_, target_y_);
        RCLCPP_INFO(node_->get_logger(), "Speed: %f", speed_);
        blackboard_ = BT::Blackboard::create(config.blackboard);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<std::string>("turtle_name"),
            BT::InputPort<double>("target_x"),
            BT::InputPort<double>("target_y"),
            BT::InputPort<double>("speed")
        };
    }

    bool setMessage(geometry_msgs::msg::Twist& msg) {
        if (!blackboard_->get<turtlesim::msg::Pose>(turtle_name_, current_pose_)) {
            RCLCPP_ERROR(node_->get_logger(), "Pose not found in blackboard for %s.", turtle_name_.c_str());
            return false; // 数据不存在，返回失败
        }
        if(calculate_movement(msg)) {
            RCLCPP_DEBUG(node_->get_logger(), "Publishing cmd_vel: linear.x = %f, angular.z = %f", msg.linear.x, msg.angular.z);
            return true;
        }
        return false;
    }

    bool calculate_movement(geometry_msgs::msg::Twist& twist) {
        double distance = get_distance();
        if (distance < tolerance_) {
            twist.linear.x = 0;
            twist.angular.z = 0;
            return true;
        }
        double angular = get_angle_diff();
        if (std::abs(angular) > tolerance_) {
            twist.linear.x = 0;
            twist.angular.z = angular;
        } else {
            twist.linear.x = speed_;
            twist.angular.z = 0;
        }
        return true;
    }

private:
    double get_angle_diff() const {
        double angle_to_target = std::atan2(target_y_ - current_pose_.y, target_x_ - current_pose_.x);
        return normalize_angle(angle_to_target - current_pose_.theta);
    }

    double get_distance() const {
        return std::hypot(current_pose_.x - target_x_, current_pose_.y - target_y_);
    }

    double normalize_angle(double angle) const {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    double tolerance_;
    double target_x_, target_y_, speed_;
    std::string topic_name_;
    std::string turtle_name_;
    BT::Blackboard::Ptr blackboard_;
    turtlesim::msg::Pose current_pose_;
};

// 行为树的 XML 定义
static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <TurtlePoseSubscriber topic_name="turtle1/pose" turtle_name="turtle1" />
        <TurtleTwistPublisher topic_name="turtle1/cmd_vel" turtle_name="turtle1" target_x="5.0" target_y="5.0" speed="2.0" />
      </Sequence>
    </BehaviorTree>
  </root>
)";

// 主函数
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("turtle_control_node");

    BehaviorTreeFactory factory;
    RosNodeParams params;
    params.nh = nh;

    factory.registerNodeType<TurtlePoseSubscriber>("TurtlePoseSubscriber", params);
    factory.registerNodeType<TurtleTwistPublisher>("TurtleTwistPublisher", params);
    auto tree = factory.createTreeFromText(xml_text);

    while (rclcpp::ok())
    {
        tree.tickWhileRunning(); // 执行行为树
        rclcpp::spin_some(nh); // 不断处理 ROS 消息
    }

    rclcpp::shutdown();
    return 0;
}
