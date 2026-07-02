#include "upper_momentum_compensator/UpperMomentumCompensatorNode.hpp"
#include "interfaces/msg/custom_joint_state.hpp"

#include <chrono>

namespace upper_momentum_compensator
{

UpperMomentumCompensatorNode::UpperMomentumCompensatorNode()
: Node("upper_momentum_compensator_node")
{
  loop_rate_hz_ =
    this->declare_parameter<double>("loop_rate_hz", 200.0);

  state_timeout_sec_ =
    this->declare_parameter<double>("state_timeout_sec", 0.1);

  const double dt =
    this->declare_parameter<double>("dt", 0.01);

  const double angular_gain =
    this->declare_parameter<double>("angular_gain", 1.0);

  const double damping =
    this->declare_parameter<double>("damping", 0.03);

  const double max_dq =
    this->declare_parameter<double>("max_dq", 3.0);

  const double max_position_step =
    this->declare_parameter<double>("max_position_step", 0.02);

  const double arm_to_leg_momentum_ratio =
    this->declare_parameter<double>("arm_to_leg_momentum_ratio", 1.0);

  const double kp_default =
    this->declare_parameter<double>("kp_default", 10.0);

  const double kd_default =
    this->declare_parameter<double>("kd_default", 0.8);

  const double max_kp =
    this->declare_parameter<double>("max_kp", 200.0);

  const double max_kd =
    this->declare_parameter<double>("max_kd", 20.0);

  const auto lower_limits =
    this->declare_parameter<std::vector<double>>(
      "upper_position_lower_limits",
      std::vector<double>(NUM_UPPER_ACT, -3.14));

  const auto upper_limits =
    this->declare_parameter<std::vector<double>>(
      "upper_position_upper_limits",
      std::vector<double>(NUM_UPPER_ACT, 3.14));

  state_sync_ =
    std::make_unique<StateSynchronizer>(this);

  compensator_.configure(
    dt,
    angular_gain,
    damping,
    max_dq,
    max_position_step,
    arm_to_leg_momentum_ratio);

  /*
   * ArmCommandGenerator mới chỉ convert q_cmd/dq_cmd thành ROS2 message.
   * Nó không còn tự tích phân dq nữa.
   */
  command_generator_.configure(
    kp_default,
    kd_default);

  safety_filter_.configure(
    max_kp,
    max_kd,
    lower_limits,
    upper_limits);

  command_pub_ =
    this->create_publisher<interfaces::msg::CustomJointState>(
      "/upper_motor/command",
      10);

  enable_sub_ =
    this->create_subscription<std_msgs::msg::Bool>(
      "/upper_momentum/enable",
      10,
      std::bind(
        &UpperMomentumCompensatorNode::enableCallback,
        this,
        std::placeholders::_1));

  const auto period =
    std::chrono::duration<double>(1.0 / loop_rate_hz_);

  timer_ =
    this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UpperMomentumCompensatorNode::timerCallback, this));

  RCLCPP_INFO(
    this->get_logger(),
    "Upper momentum compensator initialized. Enable with /upper_momentum/enable std_msgs/Bool.");
}

void UpperMomentumCompensatorNode::enableCallback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  enabled_ = msg->data;

  if (enabled_)
  {
    const auto state =
      state_sync_->getState();

    if (state.upper_valid)
    {
      /*
       * Reset MomentumCompensator internal q_cmd_prev_ from current posture.
       * This prevents position command jump when enabling.
       */
      MomentumCompensator::Vec8 q_now;

      for (int i = 0; i < NUM_UPPER_ACT; ++i)
      {
        q_now(i) = state.q_upper(i);
      }

      compensator_.reset(q_now);
      nominal_initialized_ = true;

      RCLCPP_INFO(
        this->get_logger(),
        "Upper momentum compensation enabled. Compensator reset from current upper posture.");
    }
    else
    {
      nominal_initialized_ = false;

      RCLCPP_WARN(
        this->get_logger(),
        "Enable requested but upper state is not valid yet.");
    }
  }
  else
  {
    nominal_initialized_ = false;

    RCLCPP_INFO(
      this->get_logger(),
      "Upper momentum compensation disabled.");
  }
}

void UpperMomentumCompensatorNode::timerCallback()
{
  const double now_sec =
    this->now().seconds();

  if (!state_sync_->isReady(now_sec, state_timeout_sec_))
  {
    // std::cout << "error here " << std::endl;
    publishInactiveCommand();
    return;
  }

  const auto state =
    state_sync_->getState();

  if (!enabled_)
  {
    // std::cout << "enable errror" << std::endl;
    publishInactiveCommand();
    return;
  }

  /*
   * If enabled before valid upper state was available,
   * reset compensator here once state becomes available.
   */
  if (!nominal_initialized_)
  {
    MomentumCompensator::Vec8 q_now;

    for (int i = 0; i < NUM_UPPER_ACT; ++i)
    {
      q_now(i) = state.q_upper(i);
    }

    compensator_.reset(q_now);
    nominal_initialized_ = true;
  }

  /*
   * MomentumCompensator now returns both q_cmd and dq_cmd.
   */
  // std::cout << "state.q_upper: " << state.q_upper.transpose() << std::endl;
  // std::cout << "state.dq_upper: " << state.dq_upper.transpose() << std::endl;
  // std::cout << "state.q_lower: " << state.q_lower.transpose() << std::endl;
  // std::cout << "state.dq_lower: " << state.dq_lower.transpose() << std::endl;
  // std::cout << "base angular velocity: " << state.base_angular_velocity.transpose() << std::endl;
  const auto arm_cmd = compensator_.computeArmCommand(state);
  // std::cout << "arm_cmd.q_cmd: " << arm_cmd.q_cmd.transpose() << std::endl;
  // std::cout << "arm_cmd.dq_cmd: " << arm_cmd.dq_cmd.transpose() << std::endl;
  auto cmd =
    command_generator_.makeCommand(
      state,
      arm_cmd.q_cmd,
      arm_cmd.dq_cmd);

  safety_filter_.filter(cmd);

  command_pub_->publish(cmd);
}

void UpperMomentumCompensatorNode::publishInactiveCommand()
{
  interfaces::msg::CustomJointState cmd;

  cmd.state.name = {
    "L_shoulder_pitch_joint",
    "L_shoulder_roll_joint",
    "L_shoulder_yaw_joint",
    "L_elbow_joint",
    "R_shoulder_pitch_joint",
    "R_shoulder_roll_joint",
    "R_shoulder_yaw_joint",
    "R_elbow_joint"
  };

  cmd.state.position.assign(NUM_UPPER_ACT, 0.0);
  cmd.state.velocity.assign(NUM_UPPER_ACT, 0.0);
  cmd.state.effort.assign(NUM_UPPER_ACT, 0.0);

  cmd.kp.assign(NUM_UPPER_ACT, 0.0);
  cmd.kd.assign(NUM_UPPER_ACT, 0.0);

  command_pub_->publish(cmd);
}

}  // namespace upper_momentum_compensator