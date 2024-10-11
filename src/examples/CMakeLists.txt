cmake_minimum_required(VERSION 3.16)
project(examples)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hi_turtle REQUIRED)

add_executable(example1 src/example1.cpp)

ament_target_dependencies(example1 
  rclcpp
  hi_turtle )

target_include_directories(example1 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../hi_turtle/include>
)

install(TARGETS
  example1
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
