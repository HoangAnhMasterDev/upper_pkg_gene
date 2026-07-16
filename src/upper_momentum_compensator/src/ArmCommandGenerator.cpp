#include "upper_momentum_compensator/ArmCommandGenerator.hpp"
#include "interfaces/msg/custom_joint_state.hpp"
#include <iostream>

namespace upper_momentum_compensator
{

void ArmCommandGenerator::configure(
  double kp_default,
  double kd_default)
{
  kp_default_ = kp_default;
  kd_default_ = kd_default;
}

interfaces::msg::CustomJointState ArmCommandGenerator::makeCommand(
  const WholeBodyState& state,
  const Eigen::Matrix<double, NUM_UPPER_ACT, 1>& q_cmd,
  const Eigen::Matrix<double, NUM_UPPER_ACT, 1>& dq_cmd) const
{
  interfaces::msg::CustomJointState cmd;

  cmd.state.name.resize(NUM_UPPER_ACT);
  cmd.state.position.resize(NUM_UPPER_ACT);
  cmd.state.velocity.resize(NUM_UPPER_ACT);
  cmd.state.effort.resize(NUM_UPPER_ACT, 0.0);

  cmd.kp.resize(NUM_UPPER_ACT);
  cmd.kd.resize(NUM_UPPER_ACT);

  for (int i = 0; i < NUM_UPPER_ACT; ++i)
  {
    cmd.state.name[i] = state.upper_joint_names[i];
    cmd.state.position[i] = q_cmd(i);
    cmd.state.velocity[i] = dq_cmd(i);
    cmd.state.effort[i] = 0.0;

    cmd.kp[i] = kp_default_;
    cmd.kd[i] = kd_default_;
    if (i == 0 && i == 4)
    {
        cmd.kp[i] = 0.4 * kp_default_;
        cmd.kd[i] = 3.0 * kd_default_;
    }
  }

  // std::cout << "ArmCommandGenerator::makeCommand: q_cmd = " << q_cmd.transpose() << std::endl;
  // std::cout << "ArmCommandGenerator::makeCommand: dq_cmd = " << dq_cmd.transpose() << std::endl;


  return cmd;
}

}  // namespace upper_momentum_compensator