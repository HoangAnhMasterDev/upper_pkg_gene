cmake_minimum_required(VERSION 3.8)
project(upper_motor_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UpperMotorState.msg"
  "msg/UpperMotorCommand.msg"
  DEPENDENCIES std_msgs
)

include_directories(include)

add_executable(upper_motor_bridge_node
  src/upper_motor_bridge_node.cpp
  src/UpperMotorBridgeNode.cpp
)

ament_target_dependencies(upper_motor_bridge_node
  rclcpp
  sensor_msgs
  std_msgs
  interfaces
)

rosidl_target_interfaces(upper_motor_bridge_node
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp"
)

target_link_libraries(upper_motor_bridge_node rt)

install(TARGETS upper_motor_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
