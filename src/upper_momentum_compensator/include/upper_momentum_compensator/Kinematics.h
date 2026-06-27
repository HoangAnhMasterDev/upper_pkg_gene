#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace upper_momentum_compensator
{

template <int N>
class SerialChainKinematics
{
public:
    static_assert(N > 0, "SerialChainKinematics<N>: N must be positive.");

    using Vec3  = Eigen::Vector3d;
    using Iso3  = Eigen::Isometry3d;
    using VecN  = Eigen::Matrix<double, N, 1>;
    using Mat6N = Eigen::Matrix<double, 6, N>;

    struct JointGeom
    {
        Vec3 xyz{Vec3::Zero()};
        Vec3 rpy{Vec3::Zero()};
        Vec3 axis{Vec3::UnitZ()};
    };

    struct FrameSpec
    {
        std::string name;
        int link_index{0};

        Vec3 xyz{Vec3::Zero()};
        Vec3 rpy{Vec3::Zero()};
    };

public:
    SerialChainKinematics();
    explicit SerialChainKinematics(const std::array<JointGeom, N>& chain);

    void setChain(const std::array<JointGeom, N>& chain);
    const std::array<JointGeom, N>& chain() const;

    void reserveFrames(std::size_t n);

    void lockFrames();
    bool framesLocked() const;

    int setFrame(
        const std::string& frame_name,
        int link_index,
        const Vec3& xyz_in_link_frame,
        const Vec3& rpy_in_link_frame = Vec3::Zero());

    bool hasFrame(const std::string& frame_name) const;

    int getFrameId(const std::string& frame_name) const;

    const FrameSpec& getFrame(int frame_id) const;
    const FrameSpec& getFrame(const std::string& frame_name) const;

    int numFrames() const;

    Iso3 computeLinkFK(
        const VecN& q,
        int link_index) const;

    Mat6N computeLinkJacobian(
        const VecN& q,
        int link_index) const;

    Iso3 computeFrameFK(
        const VecN& q,
        int frame_id) const;

    Iso3 computeFrameFK(
        const VecN& q,
        const std::string& frame_name) const;

    Mat6N computeFrameJacobian(
        const VecN& q,
        int frame_id) const;

    Mat6N computeFrameJacobian(
        const VecN& q,
        const std::string& frame_name) const;

    void computeFrameKinematics(
        const VecN& q,
        int frame_id,
        Iso3& T_root_frame_out,
        Mat6N& J_frame_out) const;

private:
    static int clampLinkIndex(int link_index);

    static Iso3 makeTranslation(const Vec3& xyz);
    static Iso3 makeRPY(const Vec3& rpy);
    static Iso3 makeRotation(const Vec3& axis, double angle);

private:
    std::array<JointGeom, N> chain_{};

    std::vector<FrameSpec> frames_;
    std::unordered_map<std::string, int> frame_name_to_id_;

    bool frames_locked_{false};
};


class ArmKinematics
{
public:
    using Chain = SerialChainKinematics<4>;

    using Vec3      = Chain::Vec3;
    using Iso3      = Chain::Iso3;
    using Vec4      = Chain::VecN;
    using Mat64     = Chain::Mat6N;
    using JointGeom = Chain::JointGeom;

public:
    ArmKinematics();

    Chain& left();
    Chain& right();

    const Chain& left() const;
    const Chain& right() const;

    int setLeftFrame(
        const std::string& frame_name,
        int link_index,
        const Vec3& xyz,
        const Vec3& rpy = Vec3::Zero());

    int setRightFrame(
        const std::string& frame_name,
        int link_index,
        const Vec3& xyz,
        const Vec3& rpy = Vec3::Zero());

    Iso3 computeLeftFrameFK(
        const Vec4& q,
        int frame_id) const;

    Iso3 computeRightFrameFK(
        const Vec4& q,
        int frame_id) const;

    Mat64 computeLeftFrameJacobian(
        const Vec4& q,
        int frame_id) const;

    Mat64 computeRightFrameJacobian(
        const Vec4& q,
        int frame_id) const;

private:
    Chain left_chain_;
    Chain right_chain_;
};


class LegKinematics
{
public:
    using Chain = SerialChainKinematics<5>;

    using Vec3      = Chain::Vec3;
    using Iso3      = Chain::Iso3;
    using Vec5      = Chain::VecN;
    using Mat65     = Chain::Mat6N;
    using JointGeom = Chain::JointGeom;

public:
    LegKinematics();

    Chain& left();
    Chain& right();

    const Chain& left() const;
    const Chain& right() const;

    int setLeftFrame(
        const std::string& frame_name,
        int link_index,
        const Vec3& xyz,
        const Vec3& rpy = Vec3::Zero());

    int setRightFrame(
        const std::string& frame_name,
        int link_index,
        const Vec3& xyz,
        const Vec3& rpy = Vec3::Zero());

    Iso3 computeLeftFrameFK(
        const Vec5& q,
        int frame_id) const;

    Iso3 computeRightFrameFK(
        const Vec5& q,
        int frame_id) const;

    Mat65 computeLeftFrameJacobian(
        const Vec5& q,
        int frame_id) const;

    Mat65 computeRightFrameJacobian(
        const Vec5& q,
        int frame_id) const;

private:
    Chain left_chain_;
    Chain right_chain_;
};

}  // namespace upper_momentum_compensator