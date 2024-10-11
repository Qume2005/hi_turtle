#include <rclcpp/rclcpp.hpp>
#include "hi_turtle/turtle_pose_subscriber.hpp"
#include "hi_turtle/turtle_twist_publisher.hpp"
#include "hi_turtle/stop_once_achieved.hpp"

static const char* xml_text = R"(
      <root BTCPP_format="4">
        <BehaviorTree>
          <Sequence>
            <Fallback>
              <StopOnceAchieved target_x="17.0" target_y="17.0" >
                <Sequence>
                  <TurtlePoseSubscriber topic_name="turtle1/pose" />
                  <TurtleTwistPublisher topic_name="turtle1/cmd_vel" speed="2.0" />
                </Sequence>
              </StopOnceAchieved>
            </Fallback>
            <Fallback>
              <StopOnceAchieved target_x="1.0" target_y="17.0" >
                <Sequence>
                  <TurtlePoseSubscriber topic_name="turtle1/pose" />
                  <TurtleTwistPublisher topic_name="turtle1/cmd_vel" speed="2.0" />
                </Sequence>
              </StopOnceAchieved>
            </Fallback>
            <Fallback>
              <StopOnceAchieved target_x="1.0" target_y="1.0" >
                <Sequence>
                  <TurtlePoseSubscriber topic_name="turtle1/pose" />
                  <TurtleTwistPublisher topic_name="turtle1/cmd_vel" speed="2.0" />
                </Sequence>
              </StopOnceAchieved>
            </Fallback>
            <Fallback>
              <StopOnceAchieved target_x="17.0" target_y="1.0" >
                <Sequence>
                  <TurtlePoseSubscriber topic_name="turtle1/pose" />
                  <TurtleTwistPublisher topic_name="turtle1/cmd_vel" speed="2.0" />
                </Sequence>
              </StopOnceAchieved>
            </Fallback>
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
        if (tree.tickWhileRunning() != BT::NodeStatus::SUCCESS) {
            continue;
        }
        break;
    }

    rclcpp::shutdown();
    return 0;
}