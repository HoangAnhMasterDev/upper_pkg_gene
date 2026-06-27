#include "interfaces/msg/custom_joint_state.hpp"
#include "upper_momentum_compensator/SafetyFilter.hpp"
#include <algorithm>
#include <cmath>

namespace upper_momentum_compensator
{

void SafetyFilter::configure(double max_kp, double max_kd,
                             const std::vector<double>& lower_limits,
                             const std::vector<double>& upper_limits)
{
  max_kp_ = max_kp;
  max_kd_ = max_kd;
  lower_limits_ = lower_limits;
  upper_limits_ = upper_limits;
};

bool SafetyFilter::filter(interfaces::msg::CustomJointState& cmd) const
{
  const size_t n = cmd.state.position.size();
  if (cmd.state.velocity.size() != n || cmd.kp.size() != n || cmd.kd.size() != n) {
    return false;
  }

  for (size_t i = 0; i < n; ++i) {
    if (!std::isfinite(cmd.state.position[i]) || !std::isfinite(cmd.state.velocity[i]) ||
        !std::isfinite(cmd.kp[i]) || !std::isfinite(cmd.kd[i])) {
      return false;
    }

    if (i < lower_limits_.size() && i < upper_limits_.size()) {
      cmd.state.position[i] = std::clamp(cmd.state.position[i], lower_limits_[i], upper_limits_[i]);
    }

    cmd.kp[i] = std::clamp(cmd.kp[i], 0.0, max_kp_);
    cmd.kd[i] = std::clamp(cmd.kd[i], 0.0, max_kd_);
  }
  return true;
};

}  // namespace upper_momentum_compensator
