#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include "hi_turtle/turtle_manager.hpp"
#include <string>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cmath>
#include <turtlesim/srv/kill.hpp>
#include <optional>

class Controller : public rclcpp::Node{
public:
    static std::shared_ptr<Controller> new_controller(int argc, char * argv[]);
    Controller();
    std::shared_ptr<TurtleManager> new_turtle_manager();
private:
    std::shared_ptr<TurtleManager> turtle_manager_;
    double tolerance_;
};

#endif // CONTROLLER_HPP
