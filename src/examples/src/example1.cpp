#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include <turtlesim/msg/pose.hpp>

#define DISTANCE_TOLERANCE 0.1
#define ANGULAR_TOLERANCE 0.01

using namespace BT;

class TurtlePoseSubscriber : public RosTopicSubNode<turtlesim::msg::Pose>
{
public:
    TurtlePoseSubscriber(const std::string& name, const NodeConfig& config, const RosNodeParams& params)
    :   RosTopicSubNode<turtlesim::msg::Pose>(name, config, params)
    {
        auto node_ptr = node_.lock();
        if (!getInput<std::string>("topic_name", topic_name_)) {
            if (node_ptr) {
                RCLCPP_ERROR(node_ptr->get_logger(), "TurtlePoseSubscriber failed to get parameters from input port.");
            }
            throw std::runtime_error("Parameter retrieval failed");
        }
        blackboard_ = config.blackboard;
    }

    static PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("topic_name")
        };
    }

    NodeStatus onTick(const std::shared_ptr<turtlesim::msg::Pose>& msg) override
    {
        if (!msg)
        {
            return NodeStatus::FAILURE;
        }
        blackboard_->set<turtlesim::msg::Pose>("current_pose", *msg);
        return NodeStatus::SUCCESS;

    }
private:
    BT::Blackboard::Ptr blackboard_;
    std::string topic_name_;
};

class TurtleTwistPublisher : public BT::RosTopicPubNode<geometry_msgs::msg::Twist> {
public:
    TurtleTwistPublisher(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
        : RosTopicPubNode<geometry_msgs::msg::Twist>(name, config, params)
    {
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
        RCLCPP_INFO(node_->get_logger(), "Topic Name: %s", topic_name_.c_str());
        RCLCPP_INFO(node_->get_logger(), "Target Position: (%f, %f)", target_x_, target_y_);
        RCLCPP_INFO(node_->get_logger(), "Speed: %f", speed_);
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<double>("speed")
        };
    }

    bool setMessage(geometry_msgs::msg::Twist& msg) override {
        if (!blackboard_->get<turtlesim::msg::Pose>("current_pose", current_pose_)) {
            RCLCPP_ERROR(node_->get_logger(), "Pose not found in blackboard for current pose.");
            return false; // 数据不存在，返回失败
        }
        return calculate_movement(msg);
    }

    bool calculate_movement(geometry_msgs::msg::Twist& twist) {
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

    double target_x_, target_y_, speed_;
    std::string topic_name_;
    BT::Blackboard::Ptr blackboard_;
    turtlesim::msg::Pose current_pose_;
};

class StopOnceAchieved : public BT::DecoratorNode
{
public:
    StopOnceAchieved(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
        : BT::DecoratorNode(name, config)
    {
        current_pose_.set__x(MAXFLOAT / 2);
        current_pose_.set__y(MAXFLOAT / 2);
        blackboard_ = config.blackboard;
        auto node = params.nh.lock();
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

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<double>("target_x"),
            BT::InputPort<double>("target_y")
        };
    }

    NodeStatus tick() override
    {
        if (!blackboard_->get<turtlesim::msg::Pose>("current_pose", current_pose_)) {}
        double distance = std::hypot(current_pose_.x - target_x_, current_pose_.y - target_y_);
        if (distance < DISTANCE_TOLERANCE)
        {
            return NodeStatus::SUCCESS; // 达到目标
        }
        if (child_node_->executeTick() == NodeStatus::SUCCESS) {
            return NodeStatus::RUNNING;
        }
        return NodeStatus::FAILURE;
    }

private:
    turtlesim::msg::Pose current_pose_;
    BT::Blackboard::Ptr blackboard_;
    double target_x_, target_y_;
};

static const char* xml_text = R"(
      <root BTCPP_format="4">
        <BehaviorTree>
          <StopOnceAchieved target_x="0.0" target_y="0.0" >
              <Sequence>
                <TurtlePoseSubscriber topic_name="turtle1/pose" />
                <TurtleTwistPublisher topic_name="turtle1/cmd_vel" speed="2.0" />
              </Sequence>
            </StopOnceAchieved>
        </BehaviorTree>
      </root>
    )";

// 主程序
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("turtle_control_node");

    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    params.nh = nh;

    factory.registerNodeType<TurtlePoseSubscriber>("TurtlePoseSubscriber", params);
    factory.registerNodeType<TurtleTwistPublisher>("TurtleTwistPublisher", params);
    factory.registerNodeType<StopOnceAchieved>("StopOnceAchieved", params);

    auto tree = factory.createTreeFromText(xml_text);

    while (rclcpp::ok()) {
        if (tree.tickWhileRunning() == NodeStatus::SUCCESS) {
            break;
        }
    }

    rclcpp::shutdown();
    return 0;
}
