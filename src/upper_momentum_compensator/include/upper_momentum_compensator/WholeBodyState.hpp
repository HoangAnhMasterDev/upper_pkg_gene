#pragma once

#include <array>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace upper_momentum_compensator
{

constexpr int NUM_UPPER_ACT = 8;
constexpr int NUM_LOWER_ACT = 10;

struct WholeBodyState
{
  std::array<std::string, NUM_UPPER_ACT> upper_joint_names;
  std::array<std::string, NUM_LOWER_ACT> lower_joint_names;

  Eigen::Matrix<double, NUM_UPPER_ACT, 1> q_upper = Eigen::Matrix<double, NUM_UPPER_ACT, 1>::Zero();
  Eigen::Matrix<double, NUM_UPPER_ACT, 1> dq_upper = Eigen::Matrix<double, NUM_UPPER_ACT, 1>::Zero();

  Eigen::Matrix<double, NUM_LOWER_ACT, 1> q_lower = Eigen::Matrix<double, NUM_LOWER_ACT, 1>::Zero();
  Eigen::Matrix<double, NUM_LOWER_ACT, 1> dq_lower = Eigen::Matrix<double, NUM_LOWER_ACT, 1>::Zero();

  Eigen::Quaterniond base_orientation = Eigen::Quaterniond::Identity();
  Eigen::Vector3d base_angular_velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d base_linear_acceleration = Eigen::Vector3d::Zero();

  bool upper_valid = false;
  bool lower_valid = false;
  bool imu_valid = false;
  bool upper_fault = false;

  double upper_stamp_sec = 0.0;
  double lower_stamp_sec = 0.0;
  double imu_stamp_sec = 0.0;
};

}  // namespace upper_momentum_compensator
