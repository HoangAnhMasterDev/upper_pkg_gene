#pragma once

#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <vectornav_msgs/msg/common_group.hpp>

#include "upper_motor_bridge/msg/upper_motor_state.hpp"
#include "upper_momentum_compensator/WholeBodyState.hpp"

namespace upper_momentum_compensator
{

class StateSynchronizer
{
public:
  explicit StateSynchronizer(rclcpp::Node* node);

  WholeBodyState getState() const;
  bool isReady(double now_sec, double timeout_sec) const;

private:
  void upperStateCallback(const upper_motor_bridge::msg::UpperMotorState::SharedPtr msg);
  void lowerJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void imuCallback(const vectornav_msgs::msg::CommonGroup::SharedPtr msg);

  rclcpp::Node* node_{nullptr};
  mutable std::mutex mutex_;
  WholeBodyState state_;

  std::vector<std::string> upper_joint_order_;
  std::vector<std::string> lower_joint_order_;

  rclcpp::Subscription<upper_motor_bridge::msg::UpperMotorState>::SharedPtr upper_state_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr lower_joint_state_sub_;
  rclcpp::Subscription<vectornav_msgs::msg::CommonGroup>::SharedPtr imu_sub_;
};

}  // namespace upper_momentum_compensator
