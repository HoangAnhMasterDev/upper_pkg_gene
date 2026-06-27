#pragma once

#include <array>
#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "upper_motor_bridge/SharedMemoryTypes.hpp"
#include "upper_motor_bridge/msg/upper_motor_state.hpp"
#include "interfaces/msg/custom_joint_state.hpp"

namespace upper_motor_bridge
{

class UpperMotorBridgeNode : public rclcpp::Node
{
public:
  UpperMotorBridgeNode();
  ~UpperMotorBridgeNode() override;

private:
  bool openStateSharedMemory();
  bool openCommandSharedMemory();
  void closeSharedMemory();

  void timerCallback();
  void commandCallback(const interfaces::msg::CustomJointState::SharedPtr msg);

  int state_fd_{-1};
  int command_fd_{-1};
  UpperMotorStateShm* state_shm_{nullptr};
  UpperMotorCommandShm* command_shm_{nullptr};

  uint64_t command_sequence_{0};

  rclcpp::Publisher<upper_motor_bridge::msg::UpperMotorState>::SharedPtr state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Subscription<interfaces::msg::CustomJointState>::SharedPtr command_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string state_shm_name_;
  std::string command_shm_name_;
  double publish_rate_hz_{200.0};
};

}  // namespace upper_motor_bridge
