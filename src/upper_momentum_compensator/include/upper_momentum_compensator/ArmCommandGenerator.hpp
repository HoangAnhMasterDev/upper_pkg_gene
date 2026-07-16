#pragma once

#include <Eigen/Dense>

#include "upper_momentum_compensator/WholeBodyState.hpp"
#include "interfaces/msg/custom_joint_state.hpp"

namespace upper_momentum_compensator
{

class ArmCommandGenerator
{
public:
  using Vec8 = Eigen::Matrix<double, NUM_UPPER_ACT, 1>;

  void configure(
    double kp_default,
    double kd_default);

  interfaces::msg::CustomJointState makeCommand(
    const WholeBodyState& state,
    const Eigen::Matrix<double, NUM_UPPER_ACT, 1>& q_cmd,
    const Eigen::Matrix<double, NUM_UPPER_ACT, 1>& dq_cmd) const;

private:
  double kp_default_{80.0};
  double kd_default_{0.5};
};

}  // namespace upper_momentum_compensator