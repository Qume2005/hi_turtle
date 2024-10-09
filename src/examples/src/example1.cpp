#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <behaviortree_ros2/bt_topic_pub_node.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class TurtlePoseSubscriber {
public:
    TurtlePoseSubscriber(rclcpp::Node::SharedPtr node) {
        pose_subscription_ = node->create_subscription<turtlesim::msg::Pose>(
            "turtle1/pose",
            10,
            [this](const turtlesim::msg::Pose::SharedPtr msg) {
                current_pose_ = *msg;
            }
        );
    }

    const turtlesim::msg::Pose& get_current_pose() const {
        return current_pose_;
    }

private:
    turtlesim::msg::Pose current_pose_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
};

class TurtleTwistPublisher : public BT::RosTopicPubNode<geometry_msgs::msg::Twist> {
public:
    TurtleTwistPublisher(const std::string& name, const BT::NodeConfiguration& config, const BT::RosNodeParams& params)
        : RosTopicPubNode<geometry_msgs::msg::Twist>(name, config, params), tolerance_(0.01), pose_subscriber_(TurtlePoseSubscriber(node_))
    {
        std::string topic_name;
        if (
            getInput<std::string>("topic_name", topic_name) &&
            getInput<double>("target_x", target_x_) &&
            getInput<double>("target_y", target_y_) &&
            getInput<double>("speed", speed_)
        ) {
            // Log input parameters
            RCLCPP_INFO(node_->get_logger(), "Initializing MoveTurtleForward node");
            RCLCPP_INFO(node_->get_logger(), "Topic Name: %s", topic_name.c_str());
            RCLCPP_INFO(node_->get_logger(), "Target Position: (%f, %f)", target_x_, target_y_);
            RCLCPP_INFO(node_->get_logger(), "Speed: %f", speed_);
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get parameters from input port.");
            throw std::runtime_error("Parameter retrieval failed");
        }
    }

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<std::string>("topic_name"),
            BT::InputPort<double>("target_x"),
            BT::InputPort<double>("target_y"),
            BT::InputPort<double>("speed")
        };
    }

    bool setMessage(geometry_msgs::msg::Twist& msg) {
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
        double angle_to_target = std::atan2(target_y_ - pose_subscriber_.get_current_pose().y, target_x_ - pose_subscriber_.get_current_pose().x);
        return normalize_angle(angle_to_target - pose_subscriber_.get_current_pose().theta);
    }

    double get_distance() const {
        return std::hypot(pose_subscriber_.get_current_pose().x - target_x_, pose_subscriber_.get_current_pose().y - target_y_);
    }

    double normalize_angle(double angle) const {
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        return angle;
    }

    double tolerance_;
    double target_x_, target_y_, speed_;
    TurtlePoseSubscriber pose_subscriber_;
};

// 改名字在这里改 topic_name 后面的 turtle1 再重新编译即可，topic_name 是个变量，不要把所有 topic_name 替换掉。
static const char* xml_text = R"(
  <root BTCPP_format="4">
    <BehaviorTree>
      <Sequence>
        <TurtleTwistPublisher topic_name="turtle1/cmd_vel" target_x="1.0" target_y="1.0" speed="1.0"/>
      </Sequence>
    </BehaviorTree>
  </root>
)";

// 主函数
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto nh = std::make_shared<rclcpp::Node>("turtle_control_node");

    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;
    params.nh = nh; // 将节点传递给参数
    factory.registerNodeType<TurtleTwistPublisher>("TurtleTwistPublisher", params);
    auto tree = factory.createTreeFromText(xml_text);

    // 主要执行循环
    while (rclcpp::ok()) {
        rclcpp::spin_some(nh);
        tree.tickOnce();
    }

    rclcpp::shutdown();
    return 0;
}
