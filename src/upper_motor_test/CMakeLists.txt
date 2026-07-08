cmake_minimum_required(VERSION 3.8)
project(upper_motor_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(upper_motor_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(interfaces REQUIRED)

add_executable(upper_sine_test_node
  src/upper_sine_test_node.cpp
)

ament_target_dependencies(upper_sine_test_node
  rclcpp
  upper_motor_bridge
  sensor_msgs
  interfaces
)

install(TARGETS upper_sine_test_node
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(humanoid_pseudo
  src/humanoid_pseudo.cpp
)

ament_target_dependencies(humanoid_pseudo
  rclcpp
  sensor_msgs
  upper_motor_bridge
  interfaces
)

install(TARGETS humanoid_pseudo
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(lower_humanoid_pseudo
  src/lower_humanoid_pseudo.cpp
)

ament_target_dependencies(lower_humanoid_pseudo
  rclcpp
  sensor_msgs
  upper_motor_bridge
  interfaces
)

install(TARGETS lower_humanoid_pseudo
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  src/upper_data_recorder.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()