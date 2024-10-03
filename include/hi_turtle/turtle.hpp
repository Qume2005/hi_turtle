#ifndef TURTLE_HPP
#define TURTLE_HPP

#include <memory>
#include <string>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <turtlesim/srv/set_pen.hpp>

// Pen 结构体用于存储当前钢笔状态
struct Pen {
    uint8_t r{255}; // 默认红色值
    uint8_t g{255}; // 默认绿色值
    uint8_t b{255}; // 默认蓝色值
    uint8_t width{1}; // 默认宽度
};

class Turtle : public std::enable_shared_from_this<Turtle> {
public:
    // 构造函数
    Turtle(rclcpp::Node *node, const std::string &turtle_name);

    // 海龟移动和设置目标函数
    std::shared_ptr<Turtle> async_move_turtle(double target_x, double target_y, double speed);
    std::shared_ptr<Turtle> move_turtle(double target_x, double target_y, double speed);
    std::shared_ptr<Turtle> sleep(std::chrono::duration<double> duration);
    std::shared_ptr<Turtle> set_pen_color(double r, double g, double b);
    std::shared_ptr<Turtle> set_pen_enabled(bool enabled);
    std::shared_ptr<Turtle> set_pen_width(double width);

private:
    // 私有成员函数
    void pose_callback(const geometry_msgs::msg::Twist twist);

    // 成员变量
    std::string turtle_name_;
    bool is_sleeped_;
    Pen current_pen_; // 使用 Pen 结构体来管理钢笔状态
    geometry_msgs::msg::Twist target_twist_;
    rclcpp::Node *node_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // TURTLE_HPP
