cmake_minimum_required(VERSION 3.8)
project(upper_payload_hold_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)
find_package(upper_motor_bridge REQUIRED)

include_directories(include)

add_executable(upper_payload_hold_controller_node
  src/main.cpp
  src/PayloadHoldNode.cpp
)

ament_target_dependencies(upper_payload_hold_controller_node
  rclcpp
  std_msgs
  sensor_msgs
  interfaces
  upper_motor_bridge
)

target_compile_features(upper_payload_hold_controller_node PUBLIC cxx_std_17)

install(TARGETS
  upper_payload_hold_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  include/
  DESTINATION include/
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()