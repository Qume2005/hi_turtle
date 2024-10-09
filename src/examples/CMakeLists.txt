cmake_minimum_required(VERSION 3.5)
project(examples)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)

add_executable(example1 src/example1.cpp)

ament_target_dependencies(example1 
  behaviortree_ros2 
  btcpp_ros2_interfaces
  geometry_msgs
  std_msgs 
  std_srvs
  turtlesim )

install(TARGETS
  example1
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
