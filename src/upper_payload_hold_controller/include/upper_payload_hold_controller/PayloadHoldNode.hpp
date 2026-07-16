#pragma once

#include <array>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/custom_joint_state.hpp"
#include "upper_motor_bridge/msg/upper_motor_state.hpp"

namespace upper_payload_hold_controller
{

class PayloadHoldNode : public rclcpp::Node
{
public:
    PayloadHoldNode();

private:
    static constexpr int kNumJoints = 8;

    using Vec8 = std::array<double, kNumJoints>;
    using CommandMsg = interfaces::msg::CustomJointState;
    using UpperStateMsg = upper_motor_bridge::msg::UpperMotorState;

    enum class Phase
    {
        WAIT_FOR_STATE = 0,
        REACHING_DEFAULT,
        HOLDING
    };

private:
    void declareAndLoadParameters();
    void initRosInterfaces();

    void upperStateCallback(const UpperStateMsg::SharedPtr msg);
    void timerCallback();

    void initializeFromCurrentState();

    Vec8 interpolateToDefault() const;

    double smooth01(double x) const;

    Vec8 interpolateVec8(
        const Vec8& from,
        const Vec8& to,
        double alpha) const;

    bool isReachFinished() const;

    CommandMsg makeCommand(
        const Vec8& q_cmd,
        const Vec8& kp_cmd,
        const Vec8& kd_cmd,
        const Vec8& tau_ff_cmd) const;

    Vec8 vectorToVec8(
        const std::vector<double>& input,
        const Vec8& default_value,
        const std::string& param_name) const;

    static Vec8 zeros();
    static Vec8 filled(double value);

    std::string phaseName() const;

private:
    /*
     * ROS topics.
     */
    std::string command_topic_{"/upper_motor/command"};
    std::string upper_state_topic_{"/upper_motor/state"};

    /*
     * Joint order:
     * 0 L_shoulder_pitch
     * 1 L_shoulder_roll
     * 2 L_shoulder_yaw
     * 3 L_elbow
     * 4 R_shoulder_pitch
     * 5 R_shoulder_roll
     * 6 R_shoulder_yaw
     * 7 R_elbow
     */
    std::vector<std::string> joint_names_;

    /*
     * Control parameters.
     */
    double loop_rate_hz_{100.0};
    double dt_{0.01};

    double reach_duration_sec_{3.0};
    double reach_finish_error_threshold_{0.03};

    double hold_ramp_duration_sec_{2.0};

    Vec8 q_default_{};
    Vec8 kp_reach_{};
    Vec8 kd_reach_{};
    Vec8 kp_hold_{};
    Vec8 kd_hold_{};
    Vec8 tau_payload_ff_{};

    /*
     * State feedback.
     */
    Vec8 q_meas_{};
    Vec8 dq_meas_{};

    bool has_state_{false};
    bool initialized_{false};

    /*
     * Reaching interpolation.
     */
    Vec8 q_start_{};
    double reach_elapsed_sec_{0.0};
    double hold_ramp_elapsed_sec_{0.0};

    Phase phase_{Phase::WAIT_FOR_STATE};

    /*
     * ROS interfaces.
     */
    rclcpp::Publisher<CommandMsg>::SharedPtr command_pub_;
    rclcpp::Subscription<UpperStateMsg>::SharedPtr upper_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace upper_payload_hold_controller