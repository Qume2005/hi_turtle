#include "hi_turtle/turtle.hpp"

// 构造函数
Turtle::Turtle(rclcpp::Node *node, const std::string &turtle_name)
    : turtle_name_(turtle_name),
      is_sleeped_(false),
      node_(node),
      current_pen_(),
      target_twist_(),
      pose_subscription_(
          node_->create_subscription<turtlesim::msg::Pose>(
              turtle_name_ + "/pose",
              rclcpp::QoS(10),
              [this](const turtlesim::msg::Pose::SharedPtr msg) {
                  geometry_msgs::msg::Twist twist;
                  // 要从 msg 中提取位姿信息并进行处理
                  pose_callback(twist);
              }
          )
      ),
      cmd_vel_publisher_(node_->create_publisher<geometry_msgs::msg::Twist>(turtle_name_ + "/cmd_vel", 10)),
      set_pen_client_(node_->create_client<turtlesim::srv::SetPen>(turtle_name_ + "/set_pen")),
      timer_(nullptr)
{}

// 设置钢笔颜色
std::shared_ptr<Turtle> Turtle::set_pen_color(double r, double g, double b) {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = static_cast<uint8_t>(r);
    request->g = static_cast<uint8_t>(g);
    request->b = static_cast<uint8_t>(b);
    request->width = current_pen_.width; // 使用当前宽度

    if (set_pen_client_->wait_for_service(std::chrono::seconds(1))) {
        auto result_future = set_pen_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_->shared_from_this(), result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Pen color set to (%.2f, %.2f, %.2f)", r, g, b);
            current_pen_.r = request->r; // 更新当前钢笔状态
            current_pen_.g = request->g;
            current_pen_.b = request->b;
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set pen color");
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Set pen service not available");
    }

    return shared_from_this();
}

// 设置钢笔启用状态
std::shared_ptr<Turtle> Turtle::set_pen_enabled(bool enabled) {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->off = !enabled; // 启用或禁用钢笔
    request->r = current_pen_.r; // 使用当前颜色
    request->g = current_pen_.g;
    request->b = current_pen_.b;
    request->width = current_pen_.width; // 使用当前宽度

    if (set_pen_client_->wait_for_service(std::chrono::seconds(1))) {
        auto result_future = set_pen_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_->shared_from_this(), result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Pen status set to %s", enabled ? "enabled" : "disabled");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set pen status");
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Set pen service not available");
    }

    return shared_from_this();
}

// 设置钢笔宽度
std::shared_ptr<Turtle> Turtle::set_pen_width(double width) {
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->width = static_cast<uint8_t>(width); // 设置宽度为无符号整数
    request->r = current_pen_.r; // 使用当前颜色
    request->g = current_pen_.g;
    request->b = current_pen_.b;

    if (set_pen_client_->wait_for_service(std::chrono::seconds(1))) {
        auto result_future = set_pen_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_->shared_from_this(), result_future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(node_->get_logger(), "Pen width set to %.2f", width);
            current_pen_.width = request->width; // 更新当前宽度
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set pen width");
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "Set pen service not available");
    }

    return shared_from_this();
}

// 其他方法
std::shared_ptr<Turtle> Turtle::async_move_turtle(double target_x, double target_y, double speed) {
    target_twist_.linear.x = speed; // 设置线速度
    // 此处应添加逻辑让海龟朝向目标位置
    is_sleeped_ = false; // 唤醒海龟
    cmd_vel_publisher_->publish(target_twist_); // 发布速度
    RCLCPP_INFO(node_->get_logger(), "Turtle will move asynchronously to (%.2f, %.2f) with speed %.2f", target_x, target_y, speed);
    return shared_from_this();
}

// 移动海龟
std::shared_ptr<Turtle> Turtle::move_turtle(double target_x, double target_y, double speed) {
    target_twist_.linear.x = speed; // 设置线速度
    is_sleeped_ = false; // 唤醒海龟
    cmd_vel_publisher_->publish(target_twist_); // 发布速度命令
    RCLCPP_INFO(node_->get_logger(), "Turtle is moving to (%.2f, %.2f) at speed %.2f", target_x, target_y, speed);
    return shared_from_this();
}

// 使海龟进入睡眠
std::shared_ptr<Turtle> Turtle::sleep(std::chrono::duration<double> duration) {
    if (is_sleeped_) {
        RCLCPP_WARN(node_->get_logger(), "Turtle is already sleeping.");
        return shared_from_this();
    }
    is_sleeped_ = true;
    RCLCPP_INFO(node_->get_logger(), "Turtle is sleeping for %.2f seconds.", duration.count());
    timer_ = node_->create_wall_timer(duration, [this]() {
        is_sleeped_ = false;
        RCLCPP_INFO(node_->get_logger(), "Turtle has woken up.");
    });
    return shared_from_this();
}

// 位姿回调处理
void Turtle::pose_callback(const geometry_msgs::msg::Twist twist) {
    if (is_sleeped_) {
        return; // 如果海龟在睡觉，则不处理
    }
    // 在这里可以添加更新位置信息的逻辑
}
