#include <rclcpp/rclcpp.hpp>
#include "upper_momentum_compensator/UpperMomentumCompensatorNode.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<upper_momentum_compensator::UpperMomentumCompensatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
