#include "upper_motor_bridge/UpperMotorBridgeNode.hpp"
#include "interfaces/msg/custom_joint_state.hpp"

#include <algorithm>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <sys/mman.h>
#include <unistd.h>

namespace upper_motor_bridge
{

UpperMotorBridgeNode::UpperMotorBridgeNode()
: Node("upper_motor_bridge_node")
{
  state_shm_name_ = this->declare_parameter<std::string>("state_shm_name", SHM_UPPER_MOTOR_STATE_NAME);
  command_shm_name_ = this->declare_parameter<std::string>("command_shm_name", SHM_UPPER_MOTOR_CMD_NAME);
  publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 200.0);

  state_pub_ = this->create_publisher<upper_motor_bridge::msg::UpperMotorState>("/upper_motor/state", 10);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/upper_motor/joint_states", 10);

  command_sub_ = this->create_subscription<interfaces::msg::CustomJointState>(
    "/upper_motor/command", 10,
    std::bind(&UpperMotorBridgeNode::commandCallback, this, std::placeholders::_1));

  if (!openStateSharedMemory()) {
    RCLCPP_WARN(this->get_logger(), "Failed to open upper motor state shared memory. Will retry in timer.");
  }
  if (!openCommandSharedMemory()) {
    RCLCPP_WARN(this->get_logger(), "Failed to open upper motor command shared memory. Will retry in timer.");
  }

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  timer_ = this->create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&UpperMotorBridgeNode::timerCallback, this));
}

UpperMotorBridgeNode::~UpperMotorBridgeNode()
{
  closeSharedMemory();
}

bool UpperMotorBridgeNode::openStateSharedMemory()
{
  if (state_shm_ != nullptr) {
    return true;
  }

  state_fd_ = shm_open(state_shm_name_.c_str(), O_RDONLY, 0666);
  if (state_fd_ < 0) {
    return false;
  }

  void* ptr = mmap(nullptr, sizeof(UpperMotorStateShm), PROT_READ, MAP_SHARED, state_fd_, 0);
  if (ptr == MAP_FAILED) {
    close(state_fd_);
    state_fd_ = -1;
    return false;
  }

  state_shm_ = static_cast<UpperMotorStateShm*>(ptr);
  RCLCPP_INFO(this->get_logger(), "Opened state shared memory: %s", state_shm_name_.c_str());
  return true;
}

bool UpperMotorBridgeNode::openCommandSharedMemory()
{
  if (command_shm_ != nullptr) {
    return true;
  }

  command_fd_ = shm_open(command_shm_name_.c_str(), O_RDWR, 0666);
  if (command_fd_ < 0) {
    return false;
  }

  void* ptr = mmap(nullptr, sizeof(UpperMotorCommandShm), PROT_READ | PROT_WRITE, MAP_SHARED, command_fd_, 0);
  if (ptr == MAP_FAILED) {
    close(command_fd_);
    command_fd_ = -1;
    return false;
  }

  command_shm_ = static_cast<UpperMotorCommandShm*>(ptr);
  RCLCPP_INFO(this->get_logger(), "Opened command shared memory: %s", command_shm_name_.c_str());
  return true;
}

void UpperMotorBridgeNode::closeSharedMemory()
{
  if (state_shm_ != nullptr) {
    munmap(state_shm_, sizeof(UpperMotorStateShm));
    state_shm_ = nullptr;
  }
  if (state_fd_ >= 0) {
    close(state_fd_);
    state_fd_ = -1;
  }

  if (command_shm_ != nullptr) {
    munmap(command_shm_, sizeof(UpperMotorCommandShm));
    command_shm_ = nullptr;
  }
  if (command_fd_ >= 0) {
    close(command_fd_);
    command_fd_ = -1;
  }
}

void UpperMotorBridgeNode::timerCallback()
{
  if (state_shm_ == nullptr) {
    openStateSharedMemory();
    return;
  }

  auto msg = upper_motor_bridge::msg::UpperMotorState();
  msg.joint_names.resize(NUM_UPPER_ACT);
  msg.position.resize(NUM_UPPER_ACT);
  msg.velocity.resize(NUM_UPPER_ACT);
  msg.torque_cmd.resize(NUM_UPPER_ACT);
  msg.feedback_torque.resize(NUM_UPPER_ACT);

  auto js = sensor_msgs::msg::JointState();
  js.header.stamp = this->now();
  js.name.resize(NUM_UPPER_ACT);
  js.position.resize(NUM_UPPER_ACT);
  js.velocity.resize(NUM_UPPER_ACT);
  js.effort.resize(NUM_UPPER_ACT);

  for (int i = 0; i < NUM_UPPER_ACT; ++i) {
    const std::string name(state_shm_->joint_names[i]);
    msg.joint_names[i] = name;
    msg.position[i] = state_shm_->position[i];
    msg.velocity[i] = state_shm_->angular_velocity[i];
    msg.torque_cmd[i] = state_shm_->torque_cmd[i];
    msg.feedback_torque[i] = state_shm_->fb_torque[i];

    js.name[i] = name;
    js.position[i] = state_shm_->position[i];
    js.velocity[i] = state_shm_->angular_velocity[i];
    js.effort[i] = state_shm_->fb_torque[i];
  }

  msg.elapsed_time_ns = state_shm_->elapsed_time_ns;
  msg.is_ethercat_ready = state_shm_->is_ethercat_ready;
  msg.is_controller_ready = state_shm_->is_controller_ready;
  msg.is_control_active = state_shm_->is_control_active;
  msg.fault = state_shm_->fault;
  msg.fault_status = state_shm_->fault_status;
  msg.fault_limit = state_shm_->fault_limit;
  msg.fault_command_timeout = state_shm_->fault_command_timeout;

  state_pub_->publish(msg);
  joint_state_pub_->publish(js);
}

void UpperMotorBridgeNode::commandCallback(
  const interfaces::msg::CustomJointState::SharedPtr msg)
{
  if (command_shm_ == nullptr) {
    if (!openCommandSharedMemory()) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Command shared memory is not available yet.");
      return;
    }
  }

  if (msg->state.position.size() < NUM_UPPER_ACT ||
      msg->state.velocity.size() < NUM_UPPER_ACT ||
      msg->kp.size() < NUM_UPPER_ACT ||
      msg->kd.size() < NUM_UPPER_ACT) {
    RCLCPP_ERROR(
      this->get_logger(),
      "CustomJointState arrays must have at least %d elements.",
      NUM_UPPER_ACT);
    return;
  }

  for (int i = 0; i < NUM_UPPER_ACT; ++i) {
    command_shm_->position[i] = msg->state.position[i];
    command_shm_->velocity[i] = msg->state.velocity[i];
    command_shm_->kp[i] = msg->kp[i];
    command_shm_->kd[i] = msg->kd[i];

    if (msg->state.effort.size() >= NUM_UPPER_ACT) {
      command_shm_->torque_ff[i] = msg->state.effort[i];
    } else {
      command_shm_->torque_ff[i] = 0.0;
    }
  }

  command_shm_->is_activated = true;
  command_shm_->sequence = ++command_sequence_;
  command_shm_->timestamp_ns = this->now().nanoseconds();
}
}  // namespace upper_motor_bridge
