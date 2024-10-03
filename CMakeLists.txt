cmake_minimum_required(VERSION 3.5)
project(hi_turtle)

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# 创建库
add_library(${PROJECT_NAME}
  src/controller.cpp
  src/turtle.cpp
  src/turtle_manager.cpp
  src/movement.cpp
)

# 指定头文件目录
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

# 添加依赖项
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  geometry_msgs
  turtlesim
)

# 导出头文件目录
ament_export_include_directories(include)

# 导出依赖项
ament_export_dependencies(rclcpp std_msgs geometry_msgs turtlesim)

# 导出库
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

# 安装库
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# 安装头文件
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
