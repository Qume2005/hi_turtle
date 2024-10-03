#include "hi_turtle/turtle_manager.hpp"

// 构造函数
TurtleManager::TurtleManager(rclcpp::Node *controller)
    : node_(controller),
      spawn_client_(node_->create_client<turtlesim::srv::Spawn>("spawn")),
      kill_client_(node_->create_client<turtlesim::srv::Kill>("kill")) {}

// 删除所有海龟
std::shared_ptr<TurtleManager> TurtleManager::kill_all() {
    for (const auto& pair : turtle_map_) {
        kill(pair.first);
    }
    return shared_from_this();
}

// 删除指定名称的海龟
std::shared_ptr<TurtleManager> TurtleManager::kill(const std::string &turtle_name) {
    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = turtle_name;

    // 检查Kill服务是否可用
    if (!kill_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "Kill service not available");
        return shared_from_this(); // 服务不可用，直接返回
    }

    // 异步发送请求
    auto result_future = kill_client_->async_send_request(request);
    
    // 检查请求是否超时
    if (result_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for kill response");
        return shared_from_this(); // 超时，直接返回
    }

    // 获取请求结果
    auto result = result_future.get(); // 获取结果
    // 检查结果是否有效
    if (!result) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to kill turtle: %s", turtle_name.c_str());
        return shared_from_this(); // 结果无效，直接返回
    }

    // 成功杀死海龟，更新海龟地图
    turtle_map_.erase(turtle_name); // 从地图中移除海龟
    RCLCPP_INFO(node_->get_logger(), "Successfully killed turtle: %s", turtle_name.c_str());

    return shared_from_this(); // 返回当前对象的共享指针
}


// 创建新的海龟
std::optional<std::shared_ptr<Turtle>> TurtleManager::new_turtle(const std::string &turtle_name, double target_x, double target_y, double target_theta) {
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->name = turtle_name;
    request->x = target_x;
    request->y = target_y;
    request->theta = target_theta;

    // 检查Spawn服务是否可用
    if (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(node_->get_logger(), "Spawn service not available");
        return std::nullopt; // 服务不可用，返回空
    }

    // 异步发送请求
    auto result_future = spawn_client_->async_send_request(request);
    // 检查请求是否超时
    if (result_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(), "Timeout waiting for spawn response");
        return std::nullopt; // 超时，返回空
    }

    // 获取请求结果
    auto result = result_future.get(); // 获取结果
    // 检查结果是否有效
    if (!result) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to spawn turtle: %s", turtle_name.c_str());
        return std::nullopt; // 结果无效，返回空
    }

    // 创建和存储新海龟
    auto turtle = std::make_shared<Turtle>(node_, turtle_name);
    turtle_map_[turtle_name] = turtle; // 将新创建的海龟存储到地图中
    RCLCPP_INFO(node_->get_logger(), "Successfully spawned turtle: %s", turtle_name.c_str());
    return turtle; // 返回新海龟的指针
}


// 获取指定名称的海龟
std::optional<std::shared_ptr<Turtle>> TurtleManager::get_turtle(const std::string &turtle_name) {
    auto it = turtle_map_.find(turtle_name);
    if (it != turtle_map_.end()) {
        return it->second; // 返回找到的海龟
    }
    return std::nullopt; // 如果未找到则返回空
}
