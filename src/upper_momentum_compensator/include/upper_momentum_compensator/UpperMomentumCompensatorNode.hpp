#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include "interfaces/msg/custom_joint_state.hpp"

#include "upper_momentum_compensator/StateSynchronizer.hpp"
#include "upper_momentum_compensator/MomentumCompensator.hpp"
#include "upper_momentum_compensator/ArmCommandGenerator.hpp"
#include "upper_momentum_compensator/SafetyFilter.hpp"

namespace upper_momentum_compensator
{

class UpperMomentumCompensatorNode : public rclcpp::Node
{
public:
  UpperMomentumCompensatorNode();

private:
  void timerCallback();
  void enableCallback(const std_msgs::msg::Bool::SharedPtr msg);
  void publishInactiveCommand();

  std::unique_ptr<StateSynchronizer> state_sync_;
  MomentumCompensator compensator_;
  ArmCommandGenerator command_generator_;
  SafetyFilter safety_filter_;

  rclcpp::Publisher<interfaces::msg::CustomJointState>::SharedPtr command_pub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool enabled_{true};
  bool nominal_initialized_{false};
  double loop_rate_hz_{200.0};
  double state_timeout_sec_{0.1};
};

}  // namespace upper_momentum_compensator
