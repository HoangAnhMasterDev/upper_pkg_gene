cmake_minimum_required(VERSION 3.8)
project(upper_momentum_compensator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(upper_motor_bridge REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(vectornav_msgs REQUIRED)
find_package(interfaces REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_executable(upper_momentum_compensator_node
  src/upper_momentum_compensator_node.cpp
  src/StateSynchronizer.cpp
  src/MomentumCompensator.cpp
  src/ArmCommandGenerator.cpp
  src/SafetyFilter.cpp
  src/Kinematics.cpp
  src/UpperMomentumCompensatorNode.cpp
)

ament_target_dependencies(upper_momentum_compensator_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  upper_motor_bridge
  interfaces
  vectornav_msgs
  Eigen3
)

install(TARGETS upper_momentum_compensator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
