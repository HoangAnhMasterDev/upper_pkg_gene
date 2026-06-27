#include "upper_momentum_compensator/MomentumCompensator.hpp"

#include <array>
#include <vector>
#include <cmath>

namespace upper_momentum_compensator
{
namespace
{

using Vec3  = Eigen::Vector3d;
using Vec4  = Eigen::Matrix<double, 4, 1>;
using Vec5  = Eigen::Matrix<double, 5, 1>;
using Vec8  = Eigen::Matrix<double, 8, 1>;
using Vec10 = Eigen::Matrix<double, 10, 1>;
using Mat3  = Eigen::Matrix3d;
using Iso3  = Eigen::Isometry3d;

struct LinkInertia
{
    double mass{0.0};
    Vec3 com{Vec3::Zero()};
    Mat3 inertia{Mat3::Zero()};
};

template <int N>
struct SerialChain
{
    std::array<LinkInertia, N> links;

    bool has_fixed_ee{false};

    /*
     * These are kept as metadata for fixed EE.
     * Actual FK/Jacobian of EE is computed through Kinematics frames:
     *   L_EE, L_EE_COM, R_EE, R_EE_COM
     */
    Vec3 fixed_ee_xyz{Vec3::Zero()};
    Vec3 fixed_ee_rpy{Vec3::Zero()};
    LinkInertia fixed_ee_link;
};

Mat3 makeInertia(
    double ixx, double ixy, double ixz,
    double iyy, double iyz, double izz)
{
    Mat3 I;
    I << ixx, ixy, ixz,
         ixy, iyy, iyz,
         ixz, iyz, izz;
    return I;
}

LinkInertia makeLink(
    double mass,
    const Vec3& com,
    const Mat3& inertia)
{
    LinkInertia link;
    link.mass = mass;
    link.com = com;
    link.inertia = inertia;
    return link;
}

Iso3 torsoInBaseTransform()
{
    /*
     * Waist_joint origin from URDF:
     * xyz = [0, 0, 0.0825]
     *
     * Waist is treated fixed at q_waist = 0.
     */
    Iso3 T = Iso3::Identity();
    T.translation() = Vec3(0.0, 0.0, 0.0825);
    return T;
}

/**************************************************
 * Upper arm inertial chains from URDF
 **************************************************/

SerialChain<4> makeLeftArmChain()
{
    SerialChain<4> c;

    c.links = {{
        // L_shoulder_pitch
        makeLink(
            0.47258,
            Vec3(0.002118, 0.049276, 0.0),
            makeInertia(
                0.000904, 6.4e-05, 0.0,
                0.000955, 0.0, 0.001372)
        ),

        // L_shoulder_roll
        makeLink(
            2.0026,
            Vec3(0.001469, -7.0e-05, -0.079989),
            makeInertia(
                0.020339, -4.0e-06, 0.000225,
                0.020072, 4.8e-05, 0.002315)
        ),

        // L_shoulder_yaw
        makeLink(
            0.60851,
            Vec3(4.8e-05, 0.00162, -0.045384),
            makeInertia(
                0.000608, 0.0, 0.0,
                0.00049, -2.0e-06, 0.000483)
        ),

        // L_elbow
        makeLink(
            0.29715,
            Vec3(0.0, -5.2e-05, -0.075944),
            makeInertia(
                0.001705, 0.0, 0.0,
                0.001534, -1.0e-06, 0.000304)
        )
    }};

    c.has_fixed_ee = true;
    c.fixed_ee_xyz = Vec3(0.0, 0.0, -0.3);
    c.fixed_ee_rpy = Vec3::Zero();

    c.fixed_ee_link =
        makeLink(
            0.15,
            Vec3(0.0, 0.0, -0.003514),
            makeInertia(
                5.5e-05, 0.0, 0.0,
                5.5e-05, 0.0, 6.4e-05)
        );

    return c;
}

SerialChain<4> makeRightArmChain()
{
    SerialChain<4> c;

    c.links = {{
        // R_shoulder_pitch
        makeLink(
            0.47258,
            Vec3(0.002118, -0.049276, 0.0),
            makeInertia(
                0.000904, -6.4e-05, 0.0,
                0.000955, 0.0, 0.001372)
        ),

        // R_shoulder_roll
        makeLink(
            2.0026,
            Vec3(0.001469, -7.0e-05, -0.079989),
            makeInertia(
                0.020339, -4.0e-06, 0.000225,
                0.020072, 4.8e-05, 0.002315)
        ),

        // R_shoulder_yaw
        makeLink(
            0.60851,
            Vec3(-4.8e-05, -0.00162, -0.045384),
            makeInertia(
                0.000608, 0.0, 0.0,
                0.00049, 2.0e-06, 0.000483)
        ),

        // R_elbow
        makeLink(
            0.29715,
            Vec3(0.0, 5.2e-05, -0.075944),
            makeInertia(
                0.001705, 0.0, 0.0,
                0.001534, 1.0e-06, 0.000304)
        )
    }};

    c.has_fixed_ee = true;
    c.fixed_ee_xyz = Vec3(0.0, 0.0, -0.3);
    c.fixed_ee_rpy = Vec3::Zero();

    c.fixed_ee_link =
        makeLink(
            0.15,
            Vec3(0.0, 0.0, -0.003514),
            makeInertia(
                5.5e-05, 0.0, 0.0,
                5.5e-05, 0.0, 6.4e-05)
        );

    return c;
}

/**************************************************
 * Leg inertial chains from URDF
 *
 * Order:
 * Left  = [L_hip, L_hip2, L_thigh, L_calf, L_toe]
 * Right = [R_hip, R_hip2, R_thigh, R_calf, R_toe]
 **************************************************/

SerialChain<5> makeLeftLegChain()
{
    SerialChain<5> c;

    c.links = {{
        // L_hip
        makeLink(
            1.5262,
            Vec3(-0.076146, -0.000144, -0.068558),
            makeInertia(
                0.004624, 1.1e-05, 0.001652,
                0.005293, 6.0e-06, 0.003353)
        ),

        // L_hip2
        makeLink(
            1.3333,
            Vec3(0.062812, -0.057927, -0.015123),
            makeInertia(
                0.002186, -0.000631, -0.00026,
                0.003313, 0.000174, 0.002911)
        ),

        // L_thigh
        makeLink(
            4.91,
            Vec3(-2.0e-05, 0.050143, -0.086535),
            makeInertia(
                0.078865, 0.0, -9.0e-06,
                0.079352, 0.00221, 0.009803)
        ),

        // L_calf
        makeLink(
            1.5,
            Vec3(3.6e-05, 0.000712, -0.1237),
            makeInertia(
                0.014686, 0.0, 3.0e-06,
                0.014996, -9.0e-06, 0.001006)
        ),

        // L_toe
        makeLink(
            0.52014,
            Vec3(0.01468, 1.4e-05, -0.035597),
            makeInertia(
                0.000142, -1.0e-06, -2.6e-05,
                0.001621, 0.0, 0.001685)
        )
    }};

    return c;
}

SerialChain<5> makeRightLegChain()
{
    SerialChain<5> c;

    c.links = {{
        // R_hip
        makeLink(
            1.5262,
            Vec3(-0.076146, -0.000144, -0.068558),
            makeInertia(
                0.004624, 1.1e-05, 0.001652,
                0.005293, 6.0e-06, 0.003353)
        ),

        // R_hip2
        makeLink(
            1.3333,
            Vec3(0.062812, 0.057927, -0.015123),
            makeInertia(
                0.002186, 0.000631, -0.00026,
                0.003313, -0.000174, 0.002911)
        ),

        // R_thigh
        makeLink(
            4.91,
            Vec3(2.0e-05, -0.050143, -0.086535),
            makeInertia(
                0.078865, 0.0, 9.0e-06,
                0.079352, -0.00221, 0.009803)
        ),

        // R_calf
        makeLink(
            1.5,
            Vec3(-3.6e-05, -0.000712, -0.1237),
            makeInertia(
                0.014686, 0.0, -3.0e-06,
                0.014996, 9.0e-06, 0.001006)
        ),

        // R_toe
        makeLink(
            0.52014,
            Vec3(0.01468, 1.4e-05, -0.035597),
            makeInertia(
                0.000142, -1.0e-06, -2.6e-05,
                0.001621, 0.0, 0.001685)
        )
    }};

    return c;
}

/**************************************************
 * Generic link info computation using Kinematics API
 **************************************************/

template <int N>
std::vector<MomentumCompensator::LinkInfo>
computeLinkInfoGeneric(
    const SerialChain<N>& inertial_chain,
    const SerialChainKinematics<N>& kinematics,
    const std::array<int, N>& com_frame_ids,
    const Eigen::Matrix<double, N, 1>& q,
    const Eigen::Matrix<double, N, 1>& dq,
    const Iso3& T_base_root)
{
    std::vector<MomentumCompensator::LinkInfo> links;
    links.reserve(N + (inertial_chain.has_fixed_ee ? 1 : 0));

    for (int i = 0; i < N; ++i)
    {
        const LinkInertia& li =
            inertial_chain.links[i];

        const Iso3 T_com_root =
            kinematics.computeFrameFK(
                q,
                com_frame_ids[i]);

        const auto J_com_root =
            kinematics.computeFrameJacobian(
                q,
                com_frame_ids[i]);

        const Iso3 T_link_root =
            kinematics.computeLinkFK(
                q,
                i);

        const Vec3 v_com_root =
            J_com_root.template block<3, N>(0, 0) * dq;

        const Vec3 w_root =
            J_com_root.template block<3, N>(3, 0) * dq;

        const Iso3 T_base_link =
            T_base_root * T_link_root;

        MomentumCompensator::LinkInfo info;
        info.mass = li.mass;
        info.p_com_base = T_base_root * T_com_root.translation();
        info.v_com_base = T_base_root.linear() * v_com_root;
        info.w_base = T_base_root.linear() * w_root;
        info.R_base_link = T_base_link.linear();
        info.I_link = li.inertia;

        links.push_back(info);
    }

    return links;
}

void appendFixedEELinkInfo(
    const SerialChain<4>& inertial_chain,
    const SerialChainKinematics<4>& kinematics,
    int ee_frame_id,
    int ee_com_frame_id,
    const Vec4& q,
    const Vec4& dq,
    const Iso3& T_base_root,
    std::vector<MomentumCompensator::LinkInfo>& links)
{
    if (!inertial_chain.has_fixed_ee)
    {
        return;
    }

    const LinkInertia& li =
        inertial_chain.fixed_ee_link;

    const Iso3 T_ee_root =
        kinematics.computeFrameFK(
            q,
            ee_frame_id);

    const Iso3 T_ee_com_root =
        kinematics.computeFrameFK(
            q,
            ee_com_frame_id);

    const auto J_ee_com_root =
        kinematics.computeFrameJacobian(
            q,
            ee_com_frame_id);

    const Vec3 v_com_root =
        J_ee_com_root.template block<3, 4>(0, 0) * dq;

    const Vec3 w_root =
        J_ee_com_root.template block<3, 4>(3, 0) * dq;

    const Iso3 T_base_ee =
        T_base_root * T_ee_root;

    MomentumCompensator::LinkInfo info;
    info.mass = li.mass;
    info.p_com_base = T_base_root * T_ee_com_root.translation();
    info.v_com_base = T_base_root.linear() * v_com_root;
    info.w_base = T_base_root.linear() * w_root;
    info.R_base_link = T_base_ee.linear();
    info.I_link = li.inertia;

    links.push_back(info);
}

/**************************************************
 * Mapping helpers
 **************************************************/

void splitUpperHardwareOrder(
    const Vec8& q_upper_hw,
    Vec4& q_left_arm,
    Vec4& q_right_arm)
{
    q_left_arm << q_upper_hw(0),
                  q_upper_hw(1),
                  q_upper_hw(2),
                  q_upper_hw(3);

    q_right_arm << q_upper_hw(4),
                   q_upper_hw(5),
                   q_upper_hw(6),
                   q_upper_hw(7);
}

Vec8 mergeUpperHardwareOrder(
    const Vec4& left,
    const Vec4& right)
{
    Vec8 out;

    out << left(0),
           left(1),
           left(2),
           left(3),
           right(0),
           right(1),
           right(2),
           right(3);

    return out;
}

void splitLowerHardwareOrder(
    const Vec10& q_lower_hw,
    Vec5& q_left_leg,
    Vec5& q_right_leg)
{
    /*
     * Lower hardware order:
     *
     * 0 R_hip
     * 1 R_hip2
     * 2 R_thigh
     * 3 R_calf
     * 4 L_hip
     * 5 L_hip2
     * 6 L_thigh
     * 7 L_calf
     * 8 L_toe
     * 9 R_toe
     */

    q_right_leg << q_lower_hw(0),
                   q_lower_hw(1),
                   q_lower_hw(2),
                   q_lower_hw(3),
                   q_lower_hw(9);

    q_left_leg << q_lower_hw(4),
                  q_lower_hw(5),
                  q_lower_hw(6),
                  q_lower_hw(7),
                  q_lower_hw(8);
}

}  // anonymous namespace

/**************************************************
 * MomentumCompensator public methods
 **************************************************/

MomentumCompensator::MomentumCompensator()
{
    q_cmd_prev_.setZero();

    /*
     * Cache frame ids once.
     * String lookup is only used here, not in the realtime loop.
     */

    left_arm_frame_ids_.link_com = {{
        arm_kinematics_.left().getFrameId("L_shoulder_pitch_COM"),
        arm_kinematics_.left().getFrameId("L_shoulder_roll_COM"),
        arm_kinematics_.left().getFrameId("L_shoulder_yaw_COM"),
        arm_kinematics_.left().getFrameId("L_elbow_COM")
    }};

    left_arm_frame_ids_.ee =
        arm_kinematics_.left().getFrameId("L_EE");

    left_arm_frame_ids_.ee_com =
        arm_kinematics_.left().getFrameId("L_EE_COM");


    right_arm_frame_ids_.link_com = {{
        arm_kinematics_.right().getFrameId("R_shoulder_pitch_COM"),
        arm_kinematics_.right().getFrameId("R_shoulder_roll_COM"),
        arm_kinematics_.right().getFrameId("R_shoulder_yaw_COM"),
        arm_kinematics_.right().getFrameId("R_elbow_COM")
    }};

    right_arm_frame_ids_.ee =
        arm_kinematics_.right().getFrameId("R_EE");

    right_arm_frame_ids_.ee_com =
        arm_kinematics_.right().getFrameId("R_EE_COM");


    left_leg_frame_ids_.link_com = {{
        leg_kinematics_.left().getFrameId("L_hip_COM"),
        leg_kinematics_.left().getFrameId("L_hip2_COM"),
        leg_kinematics_.left().getFrameId("L_thigh_COM"),
        leg_kinematics_.left().getFrameId("L_calf_COM"),
        leg_kinematics_.left().getFrameId("L_toe_COM")
    }};

    right_leg_frame_ids_.link_com = {{
        leg_kinematics_.right().getFrameId("R_hip_COM"),
        leg_kinematics_.right().getFrameId("R_hip2_COM"),
        leg_kinematics_.right().getFrameId("R_thigh_COM"),
        leg_kinematics_.right().getFrameId("R_calf_COM"),
        leg_kinematics_.right().getFrameId("R_toe_COM")
    }};

    /*
     * Prevent accidental frame allocation/update in realtime phase.
     */
    arm_kinematics_.left().lockFrames();
    arm_kinematics_.right().lockFrames();

    leg_kinematics_.left().lockFrames();
    leg_kinematics_.right().lockFrames();
}

void MomentumCompensator::configure(
    double dt,
    double angular_gain,
    double damping,
    double max_dq,
    double max_position_step,
    double arm_to_leg_momentum_ratio)
{
    dt_ = dt;
    angular_gain_ = angular_gain;
    damping_ = damping;
    max_dq_ = max_dq;
    max_position_step_ = max_position_step;
    arm_to_leg_momentum_ratio_ = arm_to_leg_momentum_ratio;
}

void MomentumCompensator::reset(const Vec8& q_upper_current)
{
    q_cmd_prev_ = q_upper_current;
    has_q_cmd_prev_ = true;
}

double MomentumCompensator::clamp(double x, double lo, double hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

/**************************************************
 * LinkInfo wrappers
 **************************************************/

std::vector<MomentumCompensator::LinkInfo>
MomentumCompensator::computeLeftArmLinkInfo(
    const Vec4& q,
    const Vec4& dq,
    const Eigen::Isometry3d& T_base_root) const
{
    const auto chain =
        makeLeftArmChain();

    auto links =
        computeLinkInfoGeneric<4>(
            chain,
            arm_kinematics_.left(),
            left_arm_frame_ids_.link_com,
            q,
            dq,
            T_base_root);

    appendFixedEELinkInfo(
        chain,
        arm_kinematics_.left(),
        left_arm_frame_ids_.ee,
        left_arm_frame_ids_.ee_com,
        q,
        dq,
        T_base_root,
        links);

    return links;
}

std::vector<MomentumCompensator::LinkInfo>
MomentumCompensator::computeRightArmLinkInfo(
    const Vec4& q,
    const Vec4& dq,
    const Eigen::Isometry3d& T_base_root) const
{
    const auto chain =
        makeRightArmChain();

    auto links =
        computeLinkInfoGeneric<4>(
            chain,
            arm_kinematics_.right(),
            right_arm_frame_ids_.link_com,
            q,
            dq,
            T_base_root);

    appendFixedEELinkInfo(
        chain,
        arm_kinematics_.right(),
        right_arm_frame_ids_.ee,
        right_arm_frame_ids_.ee_com,
        q,
        dq,
        T_base_root,
        links);

    return links;
}

std::vector<MomentumCompensator::LinkInfo>
MomentumCompensator::computeLeftLegLinkInfo(
    const Vec5& q,
    const Vec5& dq,
    const Eigen::Isometry3d& T_base_root) const
{
    return computeLinkInfoGeneric<5>(
        makeLeftLegChain(),
        leg_kinematics_.left(),
        left_leg_frame_ids_.link_com,
        q,
        dq,
        T_base_root);
}

std::vector<MomentumCompensator::LinkInfo>
MomentumCompensator::computeRightLegLinkInfo(
    const Vec5& q,
    const Vec5& dq,
    const Eigen::Isometry3d& T_base_root) const
{
    return computeLinkInfoGeneric<5>(
        makeRightLegChain(),
        leg_kinematics_.right(),
        right_leg_frame_ids_.link_com,
        q,
        dq,
        T_base_root);
}

/**************************************************
 * Static base and torso
 **************************************************/

std::vector<MomentumCompensator::LinkInfo>
MomentumCompensator::makeStaticBaseAndTorsoLinks() const
{
    std::vector<LinkInfo> links;
    links.reserve(2);

    /*
     * Base link.
     */
    LinkInfo base;
    base.mass = 5.124;
    base.p_com_base = Vec3(-4.0e-06, 1.0e-05, 0.030876);
    base.v_com_base.setZero();
    base.w_base.setZero();
    base.R_base_link.setIdentity();
    base.I_link =
        makeInertia(
            0.051784, 0.0, 1.0e-06,
            0.01018, -1.0e-06, 0.055756);

    links.push_back(base);

    /*
     * Torso link.
     *
     * Waist_joint origin: [0, 0, 0.0825]
     * Torso COM in Torso frame: [-0.004339, 0, 0.21611]
     */
    LinkInfo torso;
    torso.mass = 16.637;
    torso.p_com_base =
        Vec3(0.0, 0.0, 0.0825) + Vec3(-0.004339, 0.0, 0.21611);
    torso.v_com_base.setZero();
    torso.w_base.setZero();
    torso.R_base_link.setIdentity();
    torso.I_link =
        makeInertia(
            0.29678, -2.9e-05, -0.0052,
            0.23294, 0.0, 0.11448);

    links.push_back(torso);

    return links;
}

/**************************************************
 * COM and angular momentum
 **************************************************/

MomentumCompensator::Vec3
MomentumCompensator::computeRobotCOMBase(
    const std::vector<LinkInfo>& all_links) const
{
    double total_mass = 0.0;
    Vec3 weighted_sum = Vec3::Zero();

    for (const auto& link : all_links)
    {
        total_mass += link.mass;
        weighted_sum += link.mass * link.p_com_base;
    }

    if (total_mass <= 1.0e-12)
    {
        return Vec3::Zero();
    }

    return weighted_sum / total_mass;
}

MomentumCompensator::Vec3
MomentumCompensator::computeAngularMomentumWorld(
    const std::vector<LinkInfo>& links,
    const Vec3& robot_com_base,
    const Eigen::Quaterniond& q_world_base) const
{
    Eigen::Quaterniond q = q_world_base;
    q.normalize();

    const Mat3 R_world_base = q.toRotationMatrix();

    Vec3 H_world = Vec3::Zero();

    for (const auto& link : links)
    {
        const Vec3 r_world =
            R_world_base * (link.p_com_base - robot_com_base);

        const Vec3 v_world =
            R_world_base * link.v_com_base;

        const Vec3 w_world =
            R_world_base * link.w_base;

        const Mat3 R_world_link =
            R_world_base * link.R_base_link;

        const Mat3 I_world =
            R_world_link * link.I_link * R_world_link.transpose();

        H_world +=
            I_world * w_world
            + r_world.cross(link.mass * v_world);
    }

    return H_world;
}

/**************************************************
 * Momentum matrix and solver
 **************************************************/

MomentumCompensator::Mat34
MomentumCompensator::computeMomentumMatrix(
    bool is_left_arm,
    const Vec4& q_arm,
    const Vec3& robot_com_base,
    const Eigen::Quaterniond& q_world_base) const
{
    Mat34 A;
    A.setZero();

    const Iso3 T_torso =
        torsoInBaseTransform();

    /*
     * H_arm = A(q) dq_arm.
     *
     * Since H is linear in dq, each column is obtained by setting
     * dq_j = 1 rad/s and all other joint velocities to zero.
     */
    for (int j = 0; j < 4; ++j)
    {
        Vec4 dq_unit = Vec4::Zero();
        dq_unit(j) = 1.0;

        std::vector<LinkInfo> arm_links;

        if (is_left_arm)
        {
            arm_links =
                computeLeftArmLinkInfo(
                    q_arm,
                    dq_unit,
                    T_torso);
        }
        else
        {
            arm_links =
                computeRightArmLinkInfo(
                    q_arm,
                    dq_unit,
                    T_torso);
        }

        A.col(j) =
            computeAngularMomentumWorld(
                arm_links,
                robot_com_base,
                q_world_base);
    }

    return A;
}

MomentumCompensator::Vec4
MomentumCompensator::solveArmVelocity(
    const Mat34& A_arm_world,
    const Vec3& H_target_world) const
{
    /*
     * Damped least-squares:
     *
     * dq = A^T (A A^T + lambda^2 I)^-1 H_target
     */
    const double lambda2 =
        damping_ * damping_;

    const Mat3 lhs =
        A_arm_world * A_arm_world.transpose()
        + lambda2 * Mat3::Identity();

    return A_arm_world.transpose() * lhs.ldlt().solve(H_target_world);
}

/**************************************************
 * Main command
 **************************************************/

MomentumCompensator::ArmCommand
MomentumCompensator::computeArmCommand(const WholeBodyState& state)
{
    /*
     * 1. Copy hardware-order state.
     */
    Vec8 q_upper_hw;
    Vec8 dq_upper_hw;

    for (int i = 0; i < 8; ++i)
    {
        q_upper_hw(i) = state.q_upper(i);
        dq_upper_hw(i) = state.dq_upper(i);
    }

    Vec10 q_lower_hw;
    Vec10 dq_lower_hw;

    for (int i = 0; i < 10; ++i)
    {
        q_lower_hw(i) = state.q_lower(i);
        dq_lower_hw(i) = state.dq_lower(i);
    }

    if (!has_q_cmd_prev_)
    {
        reset(q_upper_hw);
    }

    /*
     * 2. Split hardware order into left/right kinematic order.
     */
    Vec4 q_left_arm;
    Vec4 q_right_arm;
    Vec4 dq_left_arm;
    Vec4 dq_right_arm;

    splitUpperHardwareOrder(
        q_upper_hw,
        q_left_arm,
        q_right_arm);

    splitUpperHardwareOrder(
        dq_upper_hw,
        dq_left_arm,
        dq_right_arm);

    Vec5 q_left_leg;
    Vec5 q_right_leg;
    Vec5 dq_left_leg;
    Vec5 dq_right_leg;

    splitLowerHardwareOrder(
        q_lower_hw,
        q_left_leg,
        q_right_leg);

    splitLowerHardwareOrder(
        dq_lower_hw,
        dq_left_leg,
        dq_right_leg);

    /*
     * 3. Compute link information.
     */
    const Iso3 T_base =
        Iso3::Identity();

    const Iso3 T_torso =
        torsoInBaseTransform();

    const auto left_leg_links =
        computeLeftLegLinkInfo(
            q_left_leg,
            dq_left_leg,
            T_base);

    const auto right_leg_links =
        computeRightLegLinkInfo(
            q_right_leg,
            dq_right_leg,
            T_base);

    const auto left_arm_links =
        computeLeftArmLinkInfo(
            q_left_arm,
            dq_left_arm,
            T_torso);

    const auto right_arm_links =
        computeRightArmLinkInfo(
            q_right_arm,
            dq_right_arm,
            T_torso);

    /*
     * 4. Compute whole-body COM in base frame.
     */
    std::vector<LinkInfo> all_links =
        makeStaticBaseAndTorsoLinks();

    all_links.insert(
        all_links.end(),
        left_leg_links.begin(),
        left_leg_links.end());

    all_links.insert(
        all_links.end(),
        right_leg_links.begin(),
        right_leg_links.end());

    all_links.insert(
        all_links.end(),
        left_arm_links.begin(),
        left_arm_links.end());

    all_links.insert(
        all_links.end(),
        right_arm_links.begin(),
        right_arm_links.end());

    const Vec3 robot_com_base =
        computeRobotCOMBase(all_links);

    /*
     * 5. Compute leg angular momentum in world frame.
     */
    const Vec3 H_left_leg_world =
        computeAngularMomentumWorld(
            left_leg_links,
            robot_com_base,
            state.base_orientation);

    const Vec3 H_right_leg_world =
        computeAngularMomentumWorld(
            right_leg_links,
            robot_com_base,
            state.base_orientation);

    // /*
    //  * 6. Pair compensation target:
    //  *
    //  * arm_to_leg_momentum_ratio * H_arm + H_leg = 0
    //  */
    // const Vec3 H_left_arm_target_world = -H_left_leg_world / arm_to_leg_momentum_ratio_;

    // const Vec3 H_right_arm_target_world = -H_right_leg_world / arm_to_leg_momentum_ratio_;

    /*
    * 6. Pair compensation target around world Z axis only.
    *
    * We only compensate yaw angular momentum:
    *   H_z = projection of H onto world Z axis.
    */
    const Vec3 z_axis_world = Vec3::UnitZ();

    const Vec3 H_left_leg_z_world = z_axis_world * z_axis_world.dot(H_left_leg_world);

    const Vec3 H_right_leg_z_world = z_axis_world * z_axis_world.dot(H_right_leg_world);

    const Vec3 H_left_arm_target_world = -H_left_leg_z_world / arm_to_leg_momentum_ratio_;

    const Vec3 H_right_arm_target_world = -H_right_leg_z_world / arm_to_leg_momentum_ratio_;

    /*
     * 7. Compute arm momentum matrices.
     */
    const Mat34 A_left_arm_world =
        computeMomentumMatrix(
            true,
            q_left_arm,
            robot_com_base,
            state.base_orientation);

    const Mat34 A_right_arm_world =
        computeMomentumMatrix(
            false,
            q_right_arm,
            robot_com_base,
            state.base_orientation);

    /*
     * 8. Solve desired arm joint velocities.
     */
    Vec4 dq_left_cmd =
        solveArmVelocity(
            A_left_arm_world,
            H_left_arm_target_world);

    Vec4 dq_right_cmd =
        solveArmVelocity(
            A_right_arm_world,
            H_right_arm_target_world);

    dq_left_cmd *= angular_gain_;
    dq_right_cmd *= angular_gain_;

    Vec8 dq_upper_cmd =
        mergeUpperHardwareOrder(
            dq_left_cmd,
            dq_right_cmd);

    /*
     * 9. Velocity saturation.
     */
    for (int i = 0; i < 8; ++i)
    {
        dq_upper_cmd(i) =
            clamp(
                dq_upper_cmd(i),
                -max_dq_,
                max_dq_);
    }

    /*
     * 10. Integrate dq_cmd to q_cmd.
     */
    Vec8 q_cmd_integrated =
        q_cmd_prev_ + dq_upper_cmd * dt_;

    Vec8 q_cmd_safe;
    q_cmd_safe.setZero();

    for (int i = 0; i < 8; ++i)
    {
        const double lo =
            q_upper_hw(i) - max_position_step_;

        const double hi =
            q_upper_hw(i) + max_position_step_;

        q_cmd_safe(i) =
            clamp(
                q_cmd_integrated(i),
                lo,
                hi);
    }

    q_cmd_prev_ =
        q_cmd_safe;

    /*
     * 11. Return command.
     */
    ArmCommand cmd;
    cmd.q_cmd = q_cmd_safe;
    cmd.dq_cmd = dq_upper_cmd;

    return cmd;
}

}  // namespace upper_momentum_compensator