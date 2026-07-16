#include "upper_payload_hold_controller/PayloadHoldNode.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

namespace upper_payload_hold_controller
{

PayloadHoldNode::PayloadHoldNode()
: Node("upper_payload_hold_controller_node")
{
    declareAndLoadParameters();
    initRosInterfaces();

    RCLCPP_INFO(
        get_logger(),
        "upper_payload_hold_controller_node started.");

    RCLCPP_INFO(
        get_logger(),
        "Mode: reach q_default with kp_reach/kd_reach, then hold with kp_hold/kd_hold + fixed tau_payload_ff.");

    RCLCPP_INFO(
        get_logger(),
        "Command topic: %s",
        command_topic_.c_str());

    RCLCPP_INFO(
        get_logger(),
        "Upper state topic: %s",
        upper_state_topic_.c_str());
}

void PayloadHoldNode::declareAndLoadParameters()
{
    command_topic_ =
        declare_parameter<std::string>(
            "command_topic",
            "/upper_motor/command");

    upper_state_topic_ =
        declare_parameter<std::string>(
            "upper_state_topic",
            "/upper_motor/state");

    loop_rate_hz_ =
        declare_parameter<double>(
            "loop_rate_hz",
            100.0);

    reach_duration_sec_ =
        declare_parameter<double>(
            "reach_duration_sec",
            3.0);
    
    hold_ramp_duration_sec_ =
        declare_parameter<double>(
            "hold_ramp_duration_sec",
            2.0);

    reach_finish_error_threshold_ =
        declare_parameter<double>(
            "reach_finish_error_threshold",
            0.03);

    joint_names_ =
        declare_parameter<std::vector<std::string>>(
            "joint_names",
            {
                "L_shoulder_pitch",
                "L_shoulder_roll",
                "L_shoulder_yaw",
                "L_elbow",
                "R_shoulder_pitch",
                "R_shoulder_roll",
                "R_shoulder_yaw",
                "R_elbow"
            });

    const Vec8 default_zero = zeros();

    q_default_ =
        vectorToVec8(
            declare_parameter<std::vector<double>>(
                "q_default",
                std::vector<double>(kNumJoints, 0.0)),
            default_zero,
            "q_default");

    kp_reach_ =
        vectorToVec8(
            declare_parameter<std::vector<double>>(
                "kp_reach",
                std::vector<double>(kNumJoints, 30.0)),
            filled(30.0),
            "kp_reach");

    kd_reach_ =
        vectorToVec8(
            declare_parameter<std::vector<double>>(
                "kd_reach",
                std::vector<double>(kNumJoints, 0.5)),
            filled(0.5),
            "kd_reach");

    kp_hold_ =
        vectorToVec8(
            declare_parameter<std::vector<double>>(
                "kp_hold",
                std::vector<double>(kNumJoints, 120.0)),
            filled(120.0),
            "kp_hold");

    kd_hold_ =
        vectorToVec8(
            declare_parameter<std::vector<double>>(
                "kd_hold",
                std::vector<double>(kNumJoints, 0.5)),
            filled(0.5),
            "kd_hold");

    tau_payload_ff_ =
        vectorToVec8(
            declare_parameter<std::vector<double>>(
                "tau_payload_ff",
                std::vector<double>(kNumJoints, 0.0)),
            default_zero,
            "tau_payload_ff");

    loop_rate_hz_ =
        std::max(1.0, loop_rate_hz_);

    dt_ = 1.0 / loop_rate_hz_;

    reach_duration_sec_ = std::max(0.1, reach_duration_sec_);

    hold_ramp_duration_sec_ = std::max(0.1, hold_ramp_duration_sec_);

    reach_finish_error_threshold_ = std::max(0.001, reach_finish_error_threshold_);
}

void PayloadHoldNode::initRosInterfaces()
{
    command_pub_ =
        create_publisher<CommandMsg>(
            command_topic_,
            rclcpp::QoS(10));

    upper_state_sub_ =
        create_subscription<UpperStateMsg>(
            upper_state_topic_,
            rclcpp::QoS(10),
            std::bind(
                &PayloadHoldNode::upperStateCallback,
                this,
                std::placeholders::_1));

    const auto period =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(dt_));

    timer_ =
        create_wall_timer(
            period,
            std::bind(
                &PayloadHoldNode::timerCallback,
                this));
}

void PayloadHoldNode::upperStateCallback(
    const UpperStateMsg::SharedPtr msg)
{
    if (msg->position.size() < kNumJoints ||
        msg->velocity.size() < kNumJoints)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Invalid /upper_motor/state size: position=%zu, velocity=%zu",
            msg->position.size(),
            msg->velocity.size());
        return;
    }

    for (int i = 0; i < kNumJoints; ++i)
    {
        q_meas_[i] = msg->position[i];
        dq_meas_[i] = msg->velocity[i];
    }

    has_state_ = true;

    if (!initialized_)
    {
        initializeFromCurrentState();
    }
}

void PayloadHoldNode::initializeFromCurrentState()
{
    q_start_ = q_meas_;
    reach_elapsed_sec_ = 0.0;

    phase_ = Phase::REACHING_DEFAULT;
    initialized_ = true;

    RCLCPP_WARN(
        get_logger(),
        "Controller initialized. Reaching q_default with kp_reach/kd_reach.");
}

void PayloadHoldNode::timerCallback()
{
    if (!has_state_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Waiting for /upper_motor/state...");
        return;
    }

    Vec8 q_cmd = q_default_;
    Vec8 kp_cmd = kp_hold_;
    Vec8 kd_cmd = kd_hold_;
    Vec8 tau_ff_cmd = tau_payload_ff_;

    if (phase_ == Phase::REACHING_DEFAULT)
    {
        reach_elapsed_sec_ += dt_;

        q_cmd = interpolateToDefault();

        kp_cmd = kp_reach_;
        kd_cmd = kd_reach_;

        /*
         * During reaching, do not add payload torque.
         * This avoids a sudden unexpected torque before the arm reaches q_default.
         */
        tau_ff_cmd = zeros();

        if (isReachFinished())
        {
            phase_ = Phase::HOLDING;
            hold_ramp_elapsed_sec_ = 0.0;

            RCLCPP_WARN(
                get_logger(),
                "Reached q_default. Switching to kp_hold/kd_hold + fixed tau_payload_ff.");
        }
    }
    else if (phase_ == Phase::HOLDING)
    {
        hold_ramp_elapsed_sec_ += dt_;

        const double raw_alpha =
            std::clamp(
                hold_ramp_elapsed_sec_ / hold_ramp_duration_sec_,
                0.0,
                1.0);

        const double alpha = smooth01(raw_alpha);

        q_cmd = q_default_;

        /*
        * Ramp gains inside HOLDING.
        * At the beginning of HOLDING:
        *   kp/kd = kp_reach/kd_reach
        *
        * After hold_ramp_duration_sec:
        *   kp/kd = kp_hold/kd_hold
        */
        kp_cmd =
            interpolateVec8(
                kp_reach_,
                kp_hold_,
                alpha);

        kd_cmd =
            interpolateVec8(
                kd_reach_,
                kd_hold_,
                alpha);

        /*
        * Ramp fixed payload feedforward together with holding gains.
        * If tau_payload_ff is all zero, this does nothing.
        */
        tau_ff_cmd =
            interpolateVec8(
                zeros(),
                tau_payload_ff_,
                alpha);
    }
    else
    {
        /*
         * WAIT_FOR_STATE should already have returned above.
         */
        q_cmd = q_meas_;
        kp_cmd = zeros();
        kd_cmd = zeros();
        tau_ff_cmd = zeros();
    }

    const auto cmd =
        makeCommand(
            q_cmd,
            kp_cmd,
            kd_cmd,
            tau_ff_cmd);

    command_pub_->publish(cmd);

    RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        500,
        "phase=%s qerr[Lsh=%.4f Lel=%.4f Rsh=%.4f Rel=%.4f] "
        "kp[Lsh=%.1f Lel=%.1f Rsh=%.1f Rel=%.1f] "
        "kd[Lsh=%.1f Lel=%.1f Rsh=%.1f Rel=%.1f] "
        "tau_ff[Lsh=%.2f Lel=%.2f Rsh=%.2f Rel=%.2f]",
        phaseName().c_str(),
        q_cmd[0] - q_meas_[0],
        q_cmd[3] - q_meas_[3],
        q_cmd[4] - q_meas_[4],
        q_cmd[7] - q_meas_[7],
        kp_cmd[0],
        kp_cmd[3],
        kp_cmd[4],
        kp_cmd[7],
        kd_cmd[0],
        kd_cmd[3],
        kd_cmd[4],
        kd_cmd[7],
        tau_ff_cmd[0],
        tau_ff_cmd[3],
        tau_ff_cmd[4],
        tau_ff_cmd[7]);
}

PayloadHoldNode::Vec8 PayloadHoldNode::interpolateToDefault() const
{
    Vec8 q_cmd{};

    const double s = smooth01(reach_elapsed_sec_ / reach_duration_sec_);

    for (int i = 0; i < kNumJoints; ++i)
    {
        q_cmd[i] = q_start_[i] + s * (q_default_[i] - q_start_[i]);
    }

    return q_cmd;
}

double PayloadHoldNode::smooth01(double x) const
{
    x =
        std::clamp(
            x,
            0.0,
            1.0);

    return x * x * (3.0 - 2.0 * x);
}

PayloadHoldNode::Vec8 PayloadHoldNode::interpolateVec8(
    const Vec8& from,
    const Vec8& to,
    double alpha) const
{
    Vec8 out{};

    alpha =
        std::clamp(
            alpha,
            0.0,
            1.0);

    for (int i = 0; i < kNumJoints; ++i)
    {
        out[i] =
            from[i] + alpha * (to[i] - from[i]);
    }

    return out;
}

bool PayloadHoldNode::isReachFinished() const
{
    if (reach_elapsed_sec_ < reach_duration_sec_)
    {
        return false;
    }

    double max_error = 0.0;

    for (int i = 0; i < kNumJoints; ++i)
    {
        const double err = std::abs(q_default_[i] - q_meas_[i]);

        max_error = std::max(max_error, err);
    }

    return max_error < reach_finish_error_threshold_;
}

PayloadHoldNode::CommandMsg PayloadHoldNode::makeCommand(
    const Vec8& q_cmd,
    const Vec8& kp_cmd,
    const Vec8& kd_cmd,
    const Vec8& tau_ff_cmd) const
{
    CommandMsg cmd;

    cmd.state.name = joint_names_;

    cmd.state.position.resize(kNumJoints, 0.0);
    cmd.state.velocity.resize(kNumJoints, 0.0);
    cmd.state.effort.resize(kNumJoints, 0.0);

    cmd.kp.resize(kNumJoints, 0.0);
    cmd.kd.resize(kNumJoints, 0.0);

    for (int i = 0; i < kNumJoints; ++i)
    {
        cmd.state.position[i] = q_cmd[i];
        cmd.state.velocity[i] = 0.0;
        cmd.state.effort[i] = tau_ff_cmd[i];

        cmd.kp[i] = kp_cmd[i];
        cmd.kd[i] = kd_cmd[i];
    }

    return cmd;
}

PayloadHoldNode::Vec8 PayloadHoldNode::vectorToVec8(
    const std::vector<double>& input,
    const Vec8& default_value,
    const std::string& param_name) const
{
    Vec8 output = default_value;

    if (input.size() != kNumJoints)
    {
        RCLCPP_WARN(
            get_logger(),
            "Parameter '%s' size is %zu, expected %d. Missing values use default.",
            param_name.c_str(),
            input.size(),
            kNumJoints);
    }

    const size_t n =
        std::min(
            input.size(),
            static_cast<size_t>(kNumJoints));

    for (size_t i = 0; i < n; ++i)
    {
        output[i] = input[i];
    }

    return output;
}

PayloadHoldNode::Vec8 PayloadHoldNode::zeros()
{
    Vec8 v{};

    for (double& x : v)
    {
        x = 0.0;
    }

    return v;
}

PayloadHoldNode::Vec8 PayloadHoldNode::filled(double value)
{
    Vec8 v{};

    for (double& x : v)
    {
        x = value;
    }

    return v;
}

std::string PayloadHoldNode::phaseName() const
{
    switch (phase_)
    {
    case Phase::WAIT_FOR_STATE:
        return "WAIT_FOR_STATE";

    case Phase::REACHING_DEFAULT:
        return "REACHING_DEFAULT";

    case Phase::HOLDING:
        return "HOLDING";

    default:
        return "UNKNOWN";
    }
}

}  // namespace upper_payload_hold_controller