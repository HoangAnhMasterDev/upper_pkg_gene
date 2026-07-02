#pragma once

#include <array>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "upper_momentum_compensator/WholeBodyState.hpp"
#include "upper_momentum_compensator/Kinematics.h"

namespace upper_momentum_compensator
{

class MomentumCompensator
{
public:
    using Vec3  = Eigen::Vector3d;
    using Vec4  = Eigen::Matrix<double, 4, 1>;
    using Vec5  = Eigen::Matrix<double, 5, 1>;
    using Vec8  = Eigen::Matrix<double, 8, 1>;
    using Vec10 = Eigen::Matrix<double, 10, 1>;
    using Mat34 = Eigen::Matrix<double, 3, 4>;

    struct ArmCommand
    {
        Vec8 q_cmd;
        Vec8 dq_cmd;
    };

    struct LinkInfo
    {
        double mass{0.0};

        Vec3 p_com_base{Vec3::Zero()};
        Vec3 v_com_base{Vec3::Zero()};
        Vec3 w_base{Vec3::Zero()};

        Eigen::Matrix3d R_base_link{Eigen::Matrix3d::Identity()};
        Eigen::Matrix3d I_link{Eigen::Matrix3d::Zero()};
    };

public:
    MomentumCompensator();

    void configure(
        double dt,
        double angular_gain,
        double damping,
        double max_dq,
        double max_position_step,
        double arm_to_leg_momentum_ratio);

    void reset(const Vec8& q_upper_current);

    ArmCommand computeArmCommand(const WholeBodyState& state);

private:
    struct ArmFrameIds
    {
        std::array<int, 4> link_com{{-1, -1, -1, -1}};
        int ee{-1};
        int ee_com{-1};
    };

    struct LegFrameIds
    {
        std::array<int, 5> link_com{{-1, -1, -1, -1, -1}};
    };

private:
    std::vector<LinkInfo> computeLeftArmLinkInfo(
        const Vec4& q,
        const Vec4& dq,
        const Eigen::Isometry3d& T_base_root) const;

    std::vector<LinkInfo> computeRightArmLinkInfo(
        const Vec4& q,
        const Vec4& dq,
        const Eigen::Isometry3d& T_base_root) const;

    std::vector<LinkInfo> computeLeftLegLinkInfo(
        const Vec5& q,
        const Vec5& dq,
        const Eigen::Isometry3d& T_base_root) const;

    std::vector<LinkInfo> computeRightLegLinkInfo(
        const Vec5& q,
        const Vec5& dq,
        const Eigen::Isometry3d& T_base_root) const;

    std::vector<LinkInfo> makeStaticBaseAndTorsoLinks() const;

    Vec3 computeRobotCOMBase(
        const std::vector<LinkInfo>& all_links) const;

    Vec3 computeAngularMomentumWorld(
        const std::vector<LinkInfo>& links,
        const Vec3& robot_com_base,
        const Eigen::Quaterniond& q_world_base) const;

    Mat34 computeMomentumMatrix(
        bool is_left_arm,
        const Vec4& q_arm,
        const Vec3& robot_com_base,
        const Eigen::Quaterniond& q_world_base) const;

    Vec4 solveArmVelocity(
        const Mat34& A_arm_world,
        const Vec3& H_target_world) const;

    static double clamp(double x, double lo, double hi);

private:
    double dt_{0.01};
    double angular_gain_{1.0};
    double damping_{0.03};
    double max_dq_{3.0};
    double max_position_step_{0.02};
    double arm_to_leg_momentum_ratio_{1.0};

    Vec8 q_cmd_prev_;
    bool has_q_cmd_prev_{false};

    ArmKinematics arm_kinematics_;
    LegKinematics leg_kinematics_;

    ArmFrameIds left_arm_frame_ids_;
    ArmFrameIds right_arm_frame_ids_;

    LegFrameIds left_leg_frame_ids_;
    LegFrameIds right_leg_frame_ids_;
};

}  // namespace upper_momentum_compensator