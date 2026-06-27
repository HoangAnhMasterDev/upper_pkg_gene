#pragma once

#include <vector>
#include <utility>
#include "interfaces/msg/custom_joint_state.hpp"

namespace upper_momentum_compensator
{

class SafetyFilter
{
public:
  void configure(double max_kp, double max_kd, const std::vector<double>& lower_limits, const std::vector<double>& upper_limits);
  bool filter(interfaces::msg::CustomJointState& cmd) const;

private:
  double max_kp_{200.0};
  double max_kd_{20.0};
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
};

}  // namespace upper_momentum_compensator
