#include "hi_turtle/controller.hpp"

Controller::Controller() :
    Node("turtle_controller"),
    tolerance_(0.1)
{

}

std::shared_ptr<Controller> Controller::new_controller(int argc, char *argv[]) {
    auto instance = std::make_shared<Controller>();
    rclcpp::init(argc, argv);
    std::future<void> spin_future = std::async(std::launch::async, [instance]() {
        rclcpp::spin(instance);
        rclcpp::shutdown();
    });
    return instance;
}

std::shared_ptr<TurtleManager> Controller::new_turtle_manager() {
    return std::make_shared<TurtleManager>(this);
}