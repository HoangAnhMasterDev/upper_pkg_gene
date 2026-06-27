#include "upper_momentum_compensator/Kinematics.h"

#include <stdexcept>

namespace upper_momentum_compensator
{

template <int N>
SerialChainKinematics<N>::SerialChainKinematics()
{
    frames_.reserve(16);
    frame_name_to_id_.reserve(16);
}

template <int N>
SerialChainKinematics<N>::SerialChainKinematics(
    const std::array<JointGeom, N>& chain)
: chain_(chain)
{
    frames_.reserve(16);
    frame_name_to_id_.reserve(16);
}

template <int N>
void SerialChainKinematics<N>::setChain(
    const std::array<JointGeom, N>& chain)
{
    chain_ = chain;
}

template <int N>
const std::array<typename SerialChainKinematics<N>::JointGeom, N>&
SerialChainKinematics<N>::chain() const
{
    return chain_;
}

template <int N>
void SerialChainKinematics<N>::reserveFrames(std::size_t n)
{
    if (frames_locked_)
    {
        throw std::runtime_error(
            "SerialChainKinematics::reserveFrames(): frames are already locked.");
    }

    frames_.reserve(n);
    frame_name_to_id_.reserve(n);
}

template <int N>
void SerialChainKinematics<N>::lockFrames()
{
    frames_locked_ = true;
}

template <int N>
bool SerialChainKinematics<N>::framesLocked() const
{
    return frames_locked_;
}

template <int N>
int SerialChainKinematics<N>::setFrame(
    const std::string& frame_name,
    int link_index,
    const Vec3& xyz_in_link_frame,
    const Vec3& rpy_in_link_frame)
{
    if (frames_locked_)
    {
        throw std::runtime_error(
            "SerialChainKinematics::setFrame(): cannot set frame after lockFrames().");
    }

    const auto it = frame_name_to_id_.find(frame_name);

    if (it != frame_name_to_id_.end())
    {
        const int existing_id = it->second;

        FrameSpec& frame = frames_[existing_id];
        frame.name = frame_name;
        frame.link_index = clampLinkIndex(link_index);
        frame.xyz = xyz_in_link_frame;
        frame.rpy = rpy_in_link_frame;

        return existing_id;
    }

    FrameSpec frame;
    frame.name = frame_name;
    frame.link_index = clampLinkIndex(link_index);
    frame.xyz = xyz_in_link_frame;
    frame.rpy = rpy_in_link_frame;

    const int frame_id = static_cast<int>(frames_.size());

    frames_.push_back(frame);
    frame_name_to_id_[frame_name] = frame_id;

    return frame_id;
}

template <int N>
bool SerialChainKinematics<N>::hasFrame(
    const std::string& frame_name) const
{
    return frame_name_to_id_.find(frame_name) != frame_name_to_id_.end();
}

template <int N>
int SerialChainKinematics<N>::getFrameId(
    const std::string& frame_name) const
{
    const auto it = frame_name_to_id_.find(frame_name);

    if (it == frame_name_to_id_.end())
    {
        throw std::runtime_error(
            "SerialChainKinematics::getFrameId(): frame not found: " + frame_name);
    }

    return it->second;
}

template <int N>
const typename SerialChainKinematics<N>::FrameSpec&
SerialChainKinematics<N>::getFrame(int frame_id) const
{
    if (frame_id < 0 || frame_id >= static_cast<int>(frames_.size()))
    {
        throw std::runtime_error(
            "SerialChainKinematics::getFrame(): invalid frame id.");
    }

    return frames_[frame_id];
}

template <int N>
const typename SerialChainKinematics<N>::FrameSpec&
SerialChainKinematics<N>::getFrame(
    const std::string& frame_name) const
{
    return getFrame(getFrameId(frame_name));
}

template <int N>
int SerialChainKinematics<N>::numFrames() const
{
    return static_cast<int>(frames_.size());
}

template <int N>
typename SerialChainKinematics<N>::Iso3
SerialChainKinematics<N>::computeLinkFK(
    const VecN& q,
    int link_index) const
{
    link_index = clampLinkIndex(link_index);

    Iso3 T = Iso3::Identity();

    for (int i = 0; i <= link_index; ++i)
    {
        T = T * makeTranslation(chain_[i].xyz);
        T = T * makeRPY(chain_[i].rpy);
        T = T * makeRotation(chain_[i].axis, q(i));
    }

    return T;
}

template <int N>
typename SerialChainKinematics<N>::Mat6N
SerialChainKinematics<N>::computeLinkJacobian(
    const VecN& q,
    int link_index) const
{
    link_index = clampLinkIndex(link_index);

    std::array<Vec3, N> p_joint;
    std::array<Vec3, N> axis_root;

    Iso3 T = Iso3::Identity();
    Iso3 T_link = Iso3::Identity();

    for (int i = 0; i <= link_index; ++i)
    {
        /*
         * Parent link frame -> joint frame.
         */
        T = T * makeTranslation(chain_[i].xyz);
        T = T * makeRPY(chain_[i].rpy);

        p_joint[i] = T.translation();
        axis_root[i] = T.linear() * chain_[i].axis.normalized();

        /*
         * Joint frame -> child link frame.
         */
        T = T * makeRotation(chain_[i].axis, q(i));

        if (i == link_index)
        {
            T_link = T;
        }
    }

    const Vec3 p_link = T_link.translation();

    Mat6N J;
    J.setZero();

    for (int j = 0; j <= link_index; ++j)
    {
        J.template block<3, 1>(0, j) =
            axis_root[j].cross(p_link - p_joint[j]);

        J.template block<3, 1>(3, j) =
            axis_root[j];
    }

    return J;
}

template <int N>
typename SerialChainKinematics<N>::Iso3
SerialChainKinematics<N>::computeFrameFK(
    const VecN& q,
    int frame_id) const
{
    const FrameSpec& frame = getFrame(frame_id);

    const Iso3 T_root_link =
        computeLinkFK(q, frame.link_index);

    const Iso3 T_link_frame =
        makeTranslation(frame.xyz) * makeRPY(frame.rpy);

    return T_root_link * T_link_frame;
}

template <int N>
typename SerialChainKinematics<N>::Iso3
SerialChainKinematics<N>::computeFrameFK(
    const VecN& q,
    const std::string& frame_name) const
{
    return computeFrameFK(q, getFrameId(frame_name));
}

template <int N>
typename SerialChainKinematics<N>::Mat6N
SerialChainKinematics<N>::computeFrameJacobian(
    const VecN& q,
    int frame_id) const
{
    const FrameSpec& frame = getFrame(frame_id);

    const Iso3 T_root_link =
        computeLinkFK(q, frame.link_index);

    const Mat6N J_link =
        computeLinkJacobian(q, frame.link_index);

    /*
     * Vector from link origin to frame origin,
     * expressed in root frame.
     */
    const Vec3 r_link_to_frame_root =
        T_root_link.linear() * frame.xyz;

    Mat6N J_frame = J_link;

    for (int j = 0; j < N; ++j)
    {
        const Vec3 Jw_j =
            J_link.template block<3, 1>(3, j);

        J_frame.template block<3, 1>(0, j) =
            J_link.template block<3, 1>(0, j)
            + Jw_j.cross(r_link_to_frame_root);

        J_frame.template block<3, 1>(3, j) =
            Jw_j;
    }

    return J_frame;
}

template <int N>
typename SerialChainKinematics<N>::Mat6N
SerialChainKinematics<N>::computeFrameJacobian(
    const VecN& q,
    const std::string& frame_name) const
{
    return computeFrameJacobian(q, getFrameId(frame_name));
}

template <int N>
void SerialChainKinematics<N>::computeFrameKinematics(
    const VecN& q,
    int frame_id,
    Iso3& T_root_frame_out,
    Mat6N& J_frame_out) const
{
    T_root_frame_out = computeFrameFK(q, frame_id);
    J_frame_out = computeFrameJacobian(q, frame_id);
}

template <int N>
int SerialChainKinematics<N>::clampLinkIndex(int link_index)
{
    if (link_index < 0)
    {
        return 0;
    }

    if (link_index >= N)
    {
        return N - 1;
    }

    return link_index;
}

template <int N>
typename SerialChainKinematics<N>::Iso3
SerialChainKinematics<N>::makeTranslation(
    const Vec3& xyz)
{
    Iso3 T = Iso3::Identity();
    T.translation() = xyz;
    return T;
}

template <int N>
typename SerialChainKinematics<N>::Iso3
SerialChainKinematics<N>::makeRPY(
    const Vec3& rpy)
{
    const double roll  = rpy.x();
    const double pitch = rpy.y();
    const double yaw   = rpy.z();

    const Eigen::AngleAxisd Rx(roll,  Vec3::UnitX());
    const Eigen::AngleAxisd Ry(pitch, Vec3::UnitY());
    const Eigen::AngleAxisd Rz(yaw,   Vec3::UnitZ());

    Iso3 T = Iso3::Identity();

    /*
     * URDF convention:
     * R = Rz(yaw) * Ry(pitch) * Rx(roll)
     */
    T.linear() = (Rz * Ry * Rx).toRotationMatrix();

    return T;
}

template <int N>
typename SerialChainKinematics<N>::Iso3
SerialChainKinematics<N>::makeRotation(
    const Vec3& axis,
    double angle)
{
    Iso3 T = Iso3::Identity();

    const Vec3 a = axis.normalized();
    T.linear() = Eigen::AngleAxisd(angle, a).toRotationMatrix();

    return T;
}


/*
 * Explicit template instantiation.
 *
 * Because template implementation is in this .cpp file,
 * we explicitly instantiate the only two chain sizes used here:
 *   arm: 4 DOF
 *   leg: 5 DOF
 */
template class SerialChainKinematics<4>;
template class SerialChainKinematics<5>;


/**************************************************
 * ArmKinematics
 **************************************************/

ArmKinematics::ArmKinematics()
{
    left().reserveFrames(16);
    right().reserveFrames(16);

    std::array<JointGeom, 4> left_chain;
    std::array<JointGeom, 4> right_chain;

    /*
     * LEFT ARM
     *
     * q_left[0] = L_shoulder_pitch
     * q_left[1] = L_shoulder_roll
     * q_left[2] = L_shoulder_yaw
     * q_left[3] = L_elbow
     */

    left_chain[0].xyz  = Vec3(0.011233, 0.15918, 0.36224);
    left_chain[0].rpy  = Vec3(0.34907, 0.0, 0.0);
    left_chain[0].axis = Vec3(0.0, 1.0, 0.0);

    left_chain[1].xyz  = Vec3(-0.011233, 0.082768, -0.00027966);
    left_chain[1].rpy  = Vec3(-0.34907, 0.0, 0.0);
    left_chain[1].axis = Vec3(1.0, 0.0, 0.0);

    left_chain[2].xyz  = Vec3(0.0, 0.0, -0.2505);
    left_chain[2].rpy  = Vec3(0.0, 0.0, 0.0);
    left_chain[2].axis = Vec3(0.0, 0.0, -1.0);

    left_chain[3].xyz  = Vec3(0.0, 0.0, -0.0545);
    left_chain[3].rpy  = Vec3(0.0, 0.0, 0.0);
    left_chain[3].axis = Vec3(0.0, 1.0, 0.0);


    /*
     * RIGHT ARM
     *
     * q_right[0] = R_shoulder_pitch
     * q_right[1] = R_shoulder_roll
     * q_right[2] = R_shoulder_yaw
     * q_right[3] = R_elbow
     */

    right_chain[0].xyz  = Vec3(-0.011263, -0.16003, 0.36224);
    right_chain[0].rpy  = Vec3(-0.34907, 0.0, 0.0);
    right_chain[0].axis = Vec3(0.0, 1.0, 0.0);

    right_chain[1].xyz  = Vec3(0.011263, -0.081976, 0.0);
    right_chain[1].rpy  = Vec3(0.34907, 0.0, 0.0);
    right_chain[1].axis = Vec3(1.0, 0.0, 0.0);

    right_chain[2].xyz  = Vec3(0.0, 0.0, -0.2505);
    right_chain[2].rpy  = Vec3(0.0, 0.0, 0.0);
    right_chain[2].axis = Vec3(0.0, 0.0, -1.0);

    right_chain[3].xyz  = Vec3(0.0, 0.0, -0.0545);
    right_chain[3].rpy  = Vec3(0.0, 0.0, 0.0);
    right_chain[3].axis = Vec3(0.0, 1.0, 0.0);

    left().setChain(left_chain);
    right().setChain(right_chain);

    /*
     * Default frames from URDF.
     *
     * These are registered by string at initialization.
     * In realtime, use getFrameId(...) once and then compute by integer id.
     */

    left().setFrame("L_shoulder_pitch_COM", 0, Vec3(0.002118, 0.049276, 0.0));
    left().setFrame("L_shoulder_roll_COM",  1, Vec3(0.001469, -0.00007, -0.079989));
    left().setFrame("L_shoulder_yaw_COM",   2, Vec3(0.000048, 0.00162, -0.045384));
    left().setFrame("L_elbow_COM",          3, Vec3(0.0, -0.000052, -0.075944));

    left().setFrame("L_EE",     3, Vec3(0.0, 0.0, -0.3));
    left().setFrame("L_EE_COM", 3, Vec3(0.0, 0.0, -0.303514));

    right().setFrame("R_shoulder_pitch_COM", 0, Vec3(0.002118, -0.049276, 0.0));
    right().setFrame("R_shoulder_roll_COM",  1, Vec3(0.001469, -0.00007, -0.079989));
    right().setFrame("R_shoulder_yaw_COM",   2, Vec3(-0.000048, -0.00162, -0.045384));
    right().setFrame("R_elbow_COM",          3, Vec3(0.0, 0.000052, -0.075944));

    right().setFrame("R_EE",     3, Vec3(0.0, 0.0, -0.3));
    right().setFrame("R_EE_COM", 3, Vec3(0.0, 0.0, -0.303514));
}

ArmKinematics::Chain& ArmKinematics::left()
{
    return left_chain_;
}

ArmKinematics::Chain& ArmKinematics::right()
{
    return right_chain_;
}

const ArmKinematics::Chain& ArmKinematics::left() const
{
    return left_chain_;
}

const ArmKinematics::Chain& ArmKinematics::right() const
{
    return right_chain_;
}

int ArmKinematics::setLeftFrame(
    const std::string& frame_name,
    int link_index,
    const Vec3& xyz,
    const Vec3& rpy)
{
    return left_chain_.setFrame(frame_name, link_index, xyz, rpy);
}

int ArmKinematics::setRightFrame(
    const std::string& frame_name,
    int link_index,
    const Vec3& xyz,
    const Vec3& rpy)
{
    return right_chain_.setFrame(frame_name, link_index, xyz, rpy);
}

ArmKinematics::Iso3 ArmKinematics::computeLeftFrameFK(
    const Vec4& q,
    int frame_id) const
{
    return left_chain_.computeFrameFK(q, frame_id);
}

ArmKinematics::Iso3 ArmKinematics::computeRightFrameFK(
    const Vec4& q,
    int frame_id) const
{
    return right_chain_.computeFrameFK(q, frame_id);
}

ArmKinematics::Mat64 ArmKinematics::computeLeftFrameJacobian(
    const Vec4& q,
    int frame_id) const
{
    return left_chain_.computeFrameJacobian(q, frame_id);
}

ArmKinematics::Mat64 ArmKinematics::computeRightFrameJacobian(
    const Vec4& q,
    int frame_id) const
{
    return right_chain_.computeFrameJacobian(q, frame_id);
}


/**************************************************
 * LegKinematics
 **************************************************/

LegKinematics::LegKinematics()
{
    left().reserveFrames(16);
    right().reserveFrames(16);

    std::array<JointGeom, 5> left_chain;
    std::array<JointGeom, 5> right_chain;

    /*
     * LEFT LEG
     *
     * q_left[0] = L_hip
     * q_left[1] = L_hip2
     * q_left[2] = L_thigh
     * q_left[3] = L_calf
     * q_left[4] = L_toe
     */

    left_chain[0].xyz  = Vec3(0.0, 0.1175, 0.0058);
    left_chain[0].rpy  = Vec3(0.0, 0.0, 0.0);
    left_chain[0].axis = Vec3(0.0, 0.0, 1.0);

    left_chain[1].xyz  = Vec3(-0.0733, 0.0, -0.0965);
    left_chain[1].rpy  = Vec3(0.0, 0.0, 0.0);
    left_chain[1].axis = Vec3(-1.0, 0.0, 0.0);

    left_chain[2].xyz  = Vec3(0.075, -0.0443, -0.0185);
    left_chain[2].rpy  = Vec3(0.0, 0.0, 0.0);
    left_chain[2].axis = Vec3(0.0, 1.0, 0.0);

    left_chain[3].xyz  = Vec3(0.0, 0.045966, -0.35);
    left_chain[3].rpy  = Vec3(0.0, 0.0, 0.0);
    left_chain[3].axis = Vec3(0.0, -1.0, 0.0);

    left_chain[4].xyz  = Vec3(-0.019446, 0.0, -0.34946);
    left_chain[4].rpy  = Vec3(0.0, 0.0, 0.0);
    left_chain[4].axis = Vec3(0.0, -1.0, 0.0);


    /*
     * RIGHT LEG
     *
     * q_right[0] = R_hip
     * q_right[1] = R_hip2
     * q_right[2] = R_thigh
     * q_right[3] = R_calf
     * q_right[4] = R_toe
     */

    right_chain[0].xyz  = Vec3(0.0, -0.1175, 0.0058);
    right_chain[0].rpy  = Vec3(0.0, 0.0, 0.0);
    right_chain[0].axis = Vec3(0.0, 0.0, 1.0);

    right_chain[1].xyz  = Vec3(-0.0733, 0.0, -0.0965);
    right_chain[1].rpy  = Vec3(0.0, 0.0, 0.0);
    right_chain[1].axis = Vec3(-1.0, 0.0, 0.0);

    right_chain[2].xyz  = Vec3(0.075, 0.0443, -0.0185);
    right_chain[2].rpy  = Vec3(0.0, 0.0, 0.0);
    right_chain[2].axis = Vec3(0.0, 1.0, 0.0);

    right_chain[3].xyz  = Vec3(0.0, -0.045966, -0.35);
    right_chain[3].rpy  = Vec3(0.0, 0.0, 0.0);
    right_chain[3].axis = Vec3(0.0, -1.0, 0.0);

    right_chain[4].xyz  = Vec3(-0.019446, 0.0, -0.34946);
    right_chain[4].rpy  = Vec3(0.0, 0.0, 0.0);
    right_chain[4].axis = Vec3(0.0, -1.0, 0.0);

    left().setChain(left_chain);
    right().setChain(right_chain);

    /*
     * Default COM frames from URDF inertial origins.
     */

    left().setFrame("L_hip_COM",    0, Vec3(-0.076146, -0.000144, -0.068558));
    left().setFrame("L_hip2_COM",   1, Vec3(0.062812, -0.057927, -0.015123));
    left().setFrame("L_thigh_COM",  2, Vec3(-0.00002, 0.050143, -0.086535));
    left().setFrame("L_calf_COM",   3, Vec3(0.000036, 0.000712, -0.1237));
    left().setFrame("L_toe_COM",    4, Vec3(0.01468, 0.000014, -0.035597));

    right().setFrame("R_hip_COM",    0, Vec3(-0.076146, -0.000144, -0.068558));
    right().setFrame("R_hip2_COM",   1, Vec3(0.062812, 0.057927, -0.015123));
    right().setFrame("R_thigh_COM",  2, Vec3(0.00002, -0.050143, -0.086535));
    right().setFrame("R_calf_COM",   3, Vec3(-0.000036, -0.000712, -0.1237));
    right().setFrame("R_toe_COM",    4, Vec3(0.01468, 0.000014, -0.035597));

    /*
     * Optional approximate foot/toe origin frame.
     * This is the origin of the toe link, not a contact point under sole.
     */
    left().setFrame("L_toe_origin", 4, Vec3(0.0, 0.0, 0.0));
    right().setFrame("R_toe_origin", 4, Vec3(0.0, 0.0, 0.0));
}

LegKinematics::Chain& LegKinematics::left()
{
    return left_chain_;
}

LegKinematics::Chain& LegKinematics::right()
{
    return right_chain_;
}

const LegKinematics::Chain& LegKinematics::left() const
{
    return left_chain_;
}

const LegKinematics::Chain& LegKinematics::right() const
{
    return right_chain_;
}

int LegKinematics::setLeftFrame(
    const std::string& frame_name,
    int link_index,
    const Vec3& xyz,
    const Vec3& rpy)
{
    return left_chain_.setFrame(frame_name, link_index, xyz, rpy);
}

int LegKinematics::setRightFrame(
    const std::string& frame_name,
    int link_index,
    const Vec3& xyz,
    const Vec3& rpy)
{
    return right_chain_.setFrame(frame_name, link_index, xyz, rpy);
}

LegKinematics::Iso3 LegKinematics::computeLeftFrameFK(
    const Vec5& q,
    int frame_id) const
{
    return left_chain_.computeFrameFK(q, frame_id);
}

LegKinematics::Iso3 LegKinematics::computeRightFrameFK(
    const Vec5& q,
    int frame_id) const
{
    return right_chain_.computeFrameFK(q, frame_id);
}

LegKinematics::Mat65 LegKinematics::computeLeftFrameJacobian(
    const Vec5& q,
    int frame_id) const
{
    return left_chain_.computeFrameJacobian(q, frame_id);
}

LegKinematics::Mat65 LegKinematics::computeRightFrameJacobian(
    const Vec5& q,
    int frame_id) const
{
    return right_chain_.computeFrameJacobian(q, frame_id);
}

}  // namespace upper_momentum_compensator