#pragma once

#include <cstdint>

namespace upper_motor_bridge
{

// Must match the ROS1 upper_motor_driver shared-memory structs exactly.
constexpr int NUM_UPPER_ACT = 8;
constexpr const char* SHM_UPPER_MOTOR_STATE_NAME = "/upper_motor_state";
constexpr const char* SHM_UPPER_MOTOR_CMD_NAME   = "/upper_motor_command";

struct UpperMotorStateShm
{
  char joint_names[NUM_UPPER_ACT][64];

  double position[NUM_UPPER_ACT];
  double angular_velocity[NUM_UPPER_ACT];
  double torque_cmd[NUM_UPPER_ACT];
  double fb_torque[NUM_UPPER_ACT];

  double elapsed_time_ns;

  bool is_ethercat_ready;
  bool is_controller_ready;
  bool is_control_active;

  bool fault;
  bool fault_status;
  bool fault_limit;
  bool fault_command_timeout;
};

struct UpperMotorCommandShm
{
  double position[NUM_UPPER_ACT];
  double velocity[NUM_UPPER_ACT];
  double torque_ff[NUM_UPPER_ACT];

  double kp[NUM_UPPER_ACT];
  double kd[NUM_UPPER_ACT];

  bool is_activated;
  uint64_t sequence;
  int64_t timestamp_ns;
};

}  // namespace upper_motor_bridge
