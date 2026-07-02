#include "upper_momentum_compensator/StateSynchronizer.hpp"
#include <vectornav_msgs/msg/common_group.hpp>
#include <algorithm>

namespace upper_momentum_compensator
{

StateSynchronizer::StateSynchronizer(rclcpp::Node* node)
: node_(node)
{
  upper_joint_order_ = node_->declare_parameter<std::vector<std::string>>(
    "upper_joint_order",
    {"L_shoulder_pitch_joint", "L_shoulder_roll_joint", "L_shoulder_yaw_joint", "L_elbow_joint",
     "R_shoulder_pitch_joint", "R_shoulder_roll_joint", "R_shoulder_yaw_joint", "R_elbow_joint"});

  lower_joint_order_ = node_->declare_parameter<std::vector<std::string>>(
    "lower_joint_order",
    {"R_hip_joint", "R_hip2_joint", "R_thigh_joint", "R_calf_joint", "L_hip_joint",
     "L_hip2_joint", "L_thigh_joint", "L_calf_joint", "L_toe_joint", "R_toe_joint"});

  upper_state_sub_ = node_->create_subscription<upper_motor_bridge::msg::UpperMotorState>(
    "/upper_motor/state", 10,
    std::bind(&StateSynchronizer::upperStateCallback, this, std::placeholders::_1));

  lower_joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    std::bind(&StateSynchronizer::lowerJointStateCallback, this, std::placeholders::_1));

  imu_sub_ = node_->create_subscription<vectornav_msgs::msg::CommonGroup>(
    "/vectornav/raw/common", 10,
    std::bind(&StateSynchronizer::imuCallback, this, std::placeholders::_1));
}

WholeBodyState StateSynchronizer::getState() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

bool StateSynchronizer::isReady(double now_sec, double timeout_sec) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const bool upper_ok = state_.upper_valid && (now_sec - state_.upper_stamp_sec) < timeout_sec && !state_.upper_fault;
  const bool lower_ok = state_.lower_valid && (now_sec - state_.lower_stamp_sec) < timeout_sec;
  // IMU is allowed to be missing for first hold tests; base angular velocity will be zero.
  // std::cout << "upper_ok: " << upper_ok << ", lower_ok: " << lower_ok << std::endl;
  return upper_ok && lower_ok;
}

void StateSynchronizer::upperStateCallback(const upper_motor_bridge::msg::UpperMotorState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (int i = 0; i < NUM_UPPER_ACT; ++i) {
    state_.upper_joint_names[i] = upper_joint_order_[i];
    auto it = std::find(msg->joint_names.begin(), msg->joint_names.end(), upper_joint_order_[i]);
    if (it == msg->joint_names.end()) {
      state_.upper_valid = false;
      return;
    }
    const size_t idx = std::distance(msg->joint_names.begin(), it);
    if (idx >= msg->position.size() || idx >= msg->velocity.size()) {
      state_.upper_valid = false;
      return;
    }
    state_.q_upper[i] = msg->position[idx];
    state_.dq_upper[i] = msg->velocity[idx];
  }

  state_.upper_fault = msg->fault || msg->fault_status || msg->fault_limit || msg->fault_command_timeout;
  state_.upper_stamp_sec = node_->now().seconds();
  state_.upper_valid = true;
}

void StateSynchronizer::lowerJointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (int i = 0; i < NUM_LOWER_ACT; ++i) {
    state_.lower_joint_names[i] = lower_joint_order_[i];
    auto it = std::find(msg->name.begin(), msg->name.end(), lower_joint_order_[i]);
    if (it == msg->name.end()) {
      state_.lower_valid = false;
      return;
    }
    const size_t idx = std::distance(msg->name.begin(), it);
    if (idx >= msg->position.size()) {
      state_.lower_valid = false;
      return;
    }
    state_.q_lower[i] = msg->position[idx];
    state_.dq_lower[i] = (idx < msg->velocity.size()) ? msg->velocity[idx] : 0.0;
  }

  state_.lower_stamp_sec = node_->now().seconds();
  state_.lower_valid = true;
}

void StateSynchronizer::imuCallback(
  const vectornav_msgs::msg::CommonGroup::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  state_.base_orientation = Eigen::Quaterniond(
    msg->quaternion.w,
    msg->quaternion.x,
    msg->quaternion.y,
    msg->quaternion.z);

  state_.base_orientation.normalize();

  state_.base_angular_velocity = Eigen::Vector3d(
    msg->angularrate.x,
    msg->angularrate.y,
    msg->angularrate.z);

  state_.base_linear_acceleration = Eigen::Vector3d(
    msg->accel.x,
    msg->accel.y,
    msg->accel.z);

  state_.imu_stamp_sec = node_->now().seconds();
  state_.imu_valid = true;
}

}  // namespace upper_momentum_compensator
