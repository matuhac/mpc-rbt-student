cmake_minimum_required(VERSION 3.8)
project(battery_monitor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(battery_node src/battery_node.cpp)

ament_target_dependencies(
  battery_node
  rclcpp
  std_msgs
)

install(TARGETS
  battery_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
