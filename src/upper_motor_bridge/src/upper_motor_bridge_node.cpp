#include <rclcpp/rclcpp.hpp>
#include "upper_motor_bridge/UpperMotorBridgeNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<upper_motor_bridge::UpperMotorBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
