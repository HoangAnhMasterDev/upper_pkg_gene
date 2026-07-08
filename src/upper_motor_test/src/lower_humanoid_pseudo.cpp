#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>
#include <unordered_map>

#include "interfaces/msg/custom_joint_state.hpp"

class LowerBodyPseudoDriverNode : public rclcpp::Node
{
public:
    LowerBodyPseudoDriverNode()
    : Node("lower_body_pseudo_driver_node")
    {
        declareParameters();
        initJointNames();
        initBuffers();
        initRosInterfaces();

        RCLCPP_INFO(get_logger(), "LowerBodyPseudoDriverNode started.");
        RCLCPP_INFO(get_logger(), "Lower command topic: %s", lower_command_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Publishing /joint_states for RViz.");
        RCLCPP_INFO(get_logger(), "Publishing /joint_states_fb_torque.");
    }

private:
    using CommandMsg = interfaces::msg::CustomJointState;

    void declareParameters()
    {
        lower_command_topic_ =
            declare_parameter<std::string>(
                "lower_command_topic",
                "/joint_cmds");

        publish_rate_hz_ =
            declare_parameter<double>(
                "publish_rate_hz",
                100.0);

        accept_command_without_names_ =
            declare_parameter<bool>(
                "accept_command_without_names",
                true);

        max_fake_dq_ =
            declare_parameter<double>(
                "max_fake_dq",
                10.0);

        use_visual_offset_ =
            declare_parameter<bool>(
                "use_visual_offset",
                true);
    }

    void initJointNames()
    {
        /*
         * Lower-body-only joint list for RViz.
         *
         * The order here is the order published in /joint_states.
         * Waist_joint is kept at zero for visualization compatibility.
         */
        urdf_joint_names_ = {
            "Waist_joint",

            "L_hip_joint",
            "L_hip2_joint",
            "L_thigh_joint",
            "L_calf_joint",
            "L_toe_joint",

            "R_hip_joint",
            "R_hip2_joint",
            "R_thigh_joint",
            "R_calf_joint",
            "R_toe_joint"
        };

        /*
         * Lower command order expected when msg.state.name is empty:
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
        lower_command_names_ = {
            "R_hip",
            "R_hip2",
            "R_thigh",
            "R_calf",

            "L_hip",
            "L_hip2",
            "L_thigh",
            "L_calf",

            "L_toe",
            "R_toe"
        };

        /*
         * Short lower command names.
         */
        command_to_urdf_index_["L_hip"] =
            indexOf("L_hip_joint");
        command_to_urdf_index_["L_hip2"] =
            indexOf("L_hip2_joint");
        command_to_urdf_index_["L_thigh"] =
            indexOf("L_thigh_joint");
        command_to_urdf_index_["L_calf"] =
            indexOf("L_calf_joint");
        command_to_urdf_index_["L_toe"] =
            indexOf("L_toe_joint");

        command_to_urdf_index_["R_hip"] =
            indexOf("R_hip_joint");
        command_to_urdf_index_["R_hip2"] =
            indexOf("R_hip2_joint");
        command_to_urdf_index_["R_thigh"] =
            indexOf("R_thigh_joint");
        command_to_urdf_index_["R_calf"] =
            indexOf("R_calf_joint");
        command_to_urdf_index_["R_toe"] =
            indexOf("R_toe_joint");

        /*
         * Also accept full URDF joint names directly.
         */
        for (size_t i = 0; i < urdf_joint_names_.size(); ++i)
        {
            command_to_urdf_index_[urdf_joint_names_[i]] =
                static_cast<int>(i);
        }
    }

    int indexOf(const std::string& name) const
    {
        const auto it =
            std::find(
                urdf_joint_names_.begin(),
                urdf_joint_names_.end(),
                name);

        if (it == urdf_joint_names_.end())
        {
            return -1;
        }

        return static_cast<int>(
            std::distance(
                urdf_joint_names_.begin(),
                it));
    }

    void setOffsetByJointName(
        const std::string& joint_name,
        double offset)
    {
        const int idx = indexOf(joint_name);

        if (idx < 0 || idx >= static_cast<int>(q_cmd_offset_.size()))
        {
            RCLCPP_WARN(
                get_logger(),
                "Cannot set offset for joint '%s': joint not found.",
                joint_name.c_str());
            return;
        }

        q_cmd_offset_[idx] = offset;

        RCLCPP_INFO(
            get_logger(),
            "Hardware-to-URDF visualization offset: %s = %.6f rad",
            joint_name.c_str(),
            offset);
    }

    void initBuffers()
    {
        const size_t n = urdf_joint_names_.size();

        q_.assign(n, 0.0);
        dq_.assign(n, 0.0);
        tau_.assign(n, 0.0);

        q_cmd_offset_.assign(n, 0.0);

        /*
         * Hardware zero offset relative to URDF.
         *
         * q_visual_urdf = q_command_hardware + offset
         *
         * Current lower-body convention:
         *
         * hip pitch / thigh = -45 deg
         * knee / calf       = -90 deg
         * ankle / toe       = +45 deg
         */
        constexpr double kDeg45 = 0.7853981633974483;
        constexpr double kDeg90 = 1.5707963267948966;

        /*
         * Right leg.
         */
        setOffsetByJointName("R_thigh_joint", -kDeg45);
        setOffsetByJointName("R_calf_joint",  -kDeg90);
        setOffsetByJointName("R_toe_joint",    kDeg45);

        /*
         * Left leg.
         */
        setOffsetByJointName("L_thigh_joint", -kDeg45);
        setOffsetByJointName("L_calf_joint",  -kDeg90);
        setOffsetByJointName("L_toe_joint",    kDeg45);

        q_cmd_prev_.assign(n, 0.0);
        q_cmd_prev_time_sec_.assign(n, 0.0);
        has_q_cmd_prev_.assign(n, false);

        joint_state_msg_.name = urdf_joint_names_;
        joint_state_msg_.position = q_;
        joint_state_msg_.velocity = dq_;
        joint_state_msg_.effort = tau_;
    }

    void initRosInterfaces()
    {
        joint_state_pub_ =
            create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states",
                rclcpp::QoS(10));

        fb_torque_pub_ =
            create_publisher<sensor_msgs::msg::JointState>(
                "/joint_states_fb_torque",
                rclcpp::QoS(10));

        lower_command_sub_ =
            create_subscription<CommandMsg>(
                lower_command_topic_,
                rclcpp::QoS(10),
                std::bind(
                    &LowerBodyPseudoDriverNode::lowerCommandCallback,
                    this,
                    std::placeholders::_1));

        const auto period =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz_));

        timer_ =
            create_wall_timer(
                period,
                std::bind(
                    &LowerBodyPseudoDriverNode::publishFeedback,
                    this));
    }

    void lowerCommandCallback(const CommandMsg::SharedPtr msg)
    {
        const double now_sec = now().seconds();

        if (!msg->state.name.empty())
        {
            /*
             * Lower command with names.
             */
            applyNamedCommand(
                *msg,
                now_sec);
        }
        else if (accept_command_without_names_)
        {
            /*
             * Lower command without names.
             * Use fixed lower hardware order.
             */
            applyLowerOrderCommand(
                *msg,
                now_sec);
        }

        has_command_ = true;
    }

    /*
     * Position feedback setter.
     *
     * If allow_finite_difference_velocity = true:
     *   dq_feedback is estimated from derivative of q_feedback.
     *
     * If allow_finite_difference_velocity = false:
     *   this function only updates q_feedback.
     *   dq_feedback should be set separately from dq_cmd.
     */
    void setJointPositionFromCommand(
        int urdf_idx,
        double q_new,
        double now_sec,
        bool allow_finite_difference_velocity)
    {
        if (urdf_idx < 0 || urdf_idx >= static_cast<int>(q_.size()))
        {
            return;
        }

        if (allow_finite_difference_velocity && has_q_cmd_prev_[urdf_idx])
        {
            const double dt =
                now_sec - q_cmd_prev_time_sec_[urdf_idx];

            if (dt > 1.0e-6)
            {
                const double dq_raw =
                    (q_new - q_cmd_prev_[urdf_idx]) / dt;

                dq_[urdf_idx] =
                    std::clamp(
                        dq_raw,
                        -max_fake_dq_,
                        max_fake_dq_);
            }
        }

        q_[urdf_idx] = q_new;

        q_cmd_prev_[urdf_idx] = q_new;
        q_cmd_prev_time_sec_[urdf_idx] = now_sec;
        has_q_cmd_prev_[urdf_idx] = true;
    }

    /*
     * Velocity feedback setter.
     *
     * In pseudo driver:
     *
     *   dq_feedback = dq_cmd
     *
     * No offset is applied to velocity because the position offset is constant.
     */
    void setJointVelocityFromCommand(
        int urdf_idx,
        double dq_cmd)
    {
        if (urdf_idx < 0 || urdf_idx >= static_cast<int>(dq_.size()))
        {
            return;
        }

        dq_[urdf_idx] =
            std::clamp(
                dq_cmd,
                -max_fake_dq_,
                max_fake_dq_);
    }

    double applyOffsetIfEnabled(
        int urdf_idx,
        double q_cmd) const
    {
        if (!use_visual_offset_)
        {
            return q_cmd;
        }

        if (urdf_idx < 0 || urdf_idx >= static_cast<int>(q_cmd_offset_.size()))
        {
            return q_cmd;
        }

        return q_cmd + q_cmd_offset_[urdf_idx];
    }

    void applyNamedCommand(
        const CommandMsg& msg,
        double now_sec)
    {
        const size_t n =
            std::min(
                msg.state.name.size(),
                msg.state.position.size());

        for (size_t i = 0; i < n; ++i)
        {
            const auto it =
                command_to_urdf_index_.find(
                    msg.state.name[i]);

            if (it == command_to_urdf_index_.end())
            {
                continue;
            }

            const int urdf_idx = it->second;

            if (urdf_idx < 0 || urdf_idx >= static_cast<int>(q_.size()))
            {
                continue;
            }

            /*
             * Lower body position visualization:
             *
             *   q_feedback_visual = q_cmd + offset
             *
             * Offset can be disabled by parameter:
             *
             *   use_visual_offset:=false
             */
            const double q_visual =
                applyOffsetIfEnabled(
                    urdf_idx,
                    msg.state.position[i]);

            const bool has_velocity =
                i < msg.state.velocity.size();

            setJointPositionFromCommand(
                urdf_idx,
                q_visual,
                now_sec,
                !has_velocity);

            /*
             * Lower body velocity feedback:
             *
             *   dq_feedback = dq_cmd
             *
             * No offset is added to velocity.
             */
            if (has_velocity)
            {
                setJointVelocityFromCommand(
                    urdf_idx,
                    msg.state.velocity[i]);
            }

            if (i < msg.state.effort.size())
            {
                tau_[urdf_idx] = msg.state.effort[i];
            }
        }
    }

    void applyLowerOrderCommand(
        const CommandMsg& msg,
        double now_sec)
    {
        const size_t n =
            std::min(
                lower_command_names_.size(),
                msg.state.position.size());

        for (size_t i = 0; i < n; ++i)
        {
            const auto it =
                command_to_urdf_index_.find(
                    lower_command_names_[i]);

            if (it == command_to_urdf_index_.end())
            {
                continue;
            }

            const int urdf_idx = it->second;

            if (urdf_idx < 0 || urdf_idx >= static_cast<int>(q_.size()))
            {
                continue;
            }

            /*
             * Lower body position visualization:
             *
             *   q_feedback_visual = q_cmd + offset
             */
            const double q_visual =
                applyOffsetIfEnabled(
                    urdf_idx,
                    msg.state.position[i]);

            const bool has_velocity =
                i < msg.state.velocity.size();

            setJointPositionFromCommand(
                urdf_idx,
                q_visual,
                now_sec,
                !has_velocity);

            /*
             * Lower body velocity feedback:
             *
             *   dq_feedback = dq_cmd
             */
            if (has_velocity)
            {
                setJointVelocityFromCommand(
                    urdf_idx,
                    msg.state.velocity[i]);
            }

            if (i < msg.state.effort.size())
            {
                tau_[urdf_idx] = msg.state.effort[i];
            }
        }
    }

    void publishFeedback()
    {
        const auto stamp = now();

        joint_state_msg_.header.stamp = stamp;
        joint_state_msg_.position = q_;
        joint_state_msg_.velocity = dq_;
        joint_state_msg_.effort = tau_;

        joint_state_pub_->publish(joint_state_msg_);
        fb_torque_pub_->publish(joint_state_msg_);
    }

private:
    std::string lower_command_topic_;

    double publish_rate_hz_{100.0};
    bool accept_command_without_names_{true};
    bool use_visual_offset_{true};

    double max_fake_dq_{10.0};

    bool has_command_{false};

    std::vector<std::string> urdf_joint_names_;
    std::vector<std::string> lower_command_names_;

    std::unordered_map<std::string, int> command_to_urdf_index_;

    std::vector<double> q_;
    std::vector<double> dq_;
    std::vector<double> tau_;

    /*
     * Hardware-to-URDF offset for visualization.
     * Size follows urdf_joint_names_ order.
     */
    std::vector<double> q_cmd_offset_;

    std::vector<double> q_cmd_prev_;
    std::vector<double> q_cmd_prev_time_sec_;
    std::vector<bool> has_q_cmd_prev_;

    sensor_msgs::msg::JointState joint_state_msg_;

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr fb_torque_pub_;

    rclcpp::Subscription<CommandMsg>::SharedPtr lower_command_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<LowerBodyPseudoDriverNode>());

    rclcpp::shutdown();

    return 0;
}