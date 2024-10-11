cmake_minimum_required(VERSION 3.16)
project(hi_turtle)

# 查找必要的包
find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)

# 定义库
add_library(hi_turtle_lib
  src/map_to_string.cpp
  src/stop_once_achieved.cpp
  src/turtle_pose_subscriber.cpp
  src/turtle_twist_publisher.cpp
)

# 依赖关系
ament_target_dependencies(hi_turtle_lib
  behaviortree_ros2
  std_msgs
  std_srvs
  geometry_msgs
  turtlesim
  rclcpp
)

# 安装库
install(TARGETS hi_turtle_lib
  EXPORT export_hi_turtle
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 导出目标
ament_export_targets(export_hi_turtle HAS_LIBRARY_TARGET)

ament_export_dependencies(
  behaviortree_ros2
  std_msgs
  turtlesim
  rclcpp
  std_srvs
  geometry_msgs
)

# 设置头文件包含目录
target_include_directories(hi_turtle_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_package()
