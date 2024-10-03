#ifndef TURTLE_MANAGER_HPP
#define TURTLE_MANAGER_HPP

#include "hi_turtle/turtle.hpp"
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>
#include <turtlesim/srv/spawn.hpp>
#include <turtlesim/srv/kill.hpp>
#include <memory>
#include <string>

class TurtleManager : std::enable_shared_from_this<TurtleManager> {
public:
    TurtleManager(rclcpp::Node *controller);
    std::shared_ptr<TurtleManager> kill_all();
    std::shared_ptr<TurtleManager> kill(const std::string &turtle_name);
    std::optional<std::shared_ptr<Turtle>> new_turtle(const std::string &turtle_name, double target_x, double target_y, double target_theta);
    std::optional<std::shared_ptr<Turtle>> get_turtle(const std::string &turtle_name);
private:
    rclcpp::Node *node_;
    std::unordered_map<std::string, std::shared_ptr<Turtle>> turtle_map_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
};

#endif // TURTLE_MANAGER_HPP
