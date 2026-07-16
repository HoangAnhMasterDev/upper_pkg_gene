// Swing leg with hip pitch only to have perfect leg angular momentum

#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/custom_joint_state.hpp"

using namespace std::chrono_literals;

class LowerSineTestNode : public rclcpp::Node
{
public:
    LowerSineTestNode()
    : Node("lower_sine_test_node")
    {
        topic_name_ = declare_parameter<std::string>("topic_name", "/joint_cmds");

        amplitude_ = declare_parameter<double>("amplitude", 0.1);      // rad
        frequency_ = declare_parameter<double>("frequency", 1.0);      // Hz
        q_center_  = declare_parameter<double>("q_center", 0.0);       // rad

        kp_ = declare_parameter<double>("kp", 100.0);
        kd_ = declare_parameter<double>("kd", 3.0);

        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 1000.0);

        joint_names_ = {
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

        pub_ = create_publisher<interfaces::msg::CustomJointState>(
            topic_name_,
            rclcpp::QoS(10));

        start_time_ = now();

        const auto period_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::duration<double>(1.0 / publish_rate_hz_));

        timer_ = create_wall_timer(
            period_ns,
            std::bind(&LowerSineTestNode::onTimer, this));

        RCLCPP_INFO(get_logger(), "Lower sine test started.");
        RCLCPP_INFO(get_logger(), "Topic: %s", topic_name_.c_str());
        RCLCPP_INFO(get_logger(), "Hip pitch sine: amp=%.3f rad, freq=%.3f Hz",
                    amplitude_, frequency_);
        RCLCPP_INFO(get_logger(), "Left/right hip pitch are opposite phase.");
    }

private:
    void onTimer()
    {
        const double t = (now() - start_time_).seconds();
        const double omega = 2.0 * M_PI * frequency_;

        const double s = std::sin(omega * t);
        const double c = std::cos(omega * t);

        /*
         * Shoulder pitch commands
         *
         * Left:
         *   q_L = q_center + A sin(wt)
         *
         * Right:
         *   q_R = q_center - A sin(wt)
         *
         * Therefore left and right shoulder pitch are 180 deg out of phase.
         */
        const double q_l_hip  = q_center_ + amplitude_ * s;
        const double dq_l_hip = amplitude_ * omega * c;

        const double q_r_hip  = q_center_ - amplitude_ * s;
        const double dq_r_hip = -amplitude_ * omega * c;

        /*
         * Elbow commands as functions of shoulder phase.
         *
         * elbow_max = 0.3 rad
         *
         * For left arm:
         *   s = -1 -> elbow = 0.0
         *   s = +1 -> elbow = 0.3
         *
         * For right arm, because shoulder is opposite phase:
         *   -s = -1 -> elbow = 0.0
         *   -s = +1 -> elbow = 0.3
         */
        // const double elbow_max = 0.3;

        // const double q_l_elbow  = -0.5 * elbow_max * (1.0 - s);
        // const double dq_l_elbow =  0.5 * elbow_max * omega * c;

        // const double q_r_elbow  = -0.5 * elbow_max * (1.0 + s);
        // const double dq_r_elbow = -0.5 * elbow_max * omega * c;

        interfaces::msg::CustomJointState msg;

        msg.state.name = joint_names_;

        msg.state.position.assign(10, 0.0);
        msg.state.velocity.assign(10, 0.0);
        msg.state.effort.assign(10, 0.0);
        msg.kp.assign(10, 0.0);
        msg.kd.assign(10, 0.0);

        // Index 6 = L_hip_pitch
        msg.state.position[6] = q_l_hip;
        msg.state.velocity[6] = dq_l_hip;
        msg.kp[6] = kp_;
        msg.kd[6] = kd_;

        // Index 2 = R_hip pitch
        msg.state.position[2] = q_r_hip;
        msg.state.velocity[2] = dq_l_hip;
        msg.kp[2] = kp_;
        msg.kd[2] = kd_;

        // // Index 4 = R_shoulder_pitch
        // msg.state.position[4] = q_r_shoulder;
        // msg.state.velocity[4] = dq_r_shoulder;
        // msg.kp[4] = kp_;
        // msg.kd[4] = kd_;

        // // Index 7 = R_elbow
        // msg.state.position[7] = q_r_elbow;
        // msg.state.velocity[7] = dq_r_elbow;
        // msg.kp[7] = kp_;
        // msg.kd[7] = kd_;

        pub_->publish(msg);
    }

private:
    std::string topic_name_;

    double amplitude_{0.1};
    double frequency_{1.0};
    double q_center_{0.0};

    double kp_{100.0};
    double kd_{3.0};

    double publish_rate_hz_{1000.0};

    std::vector<std::string> joint_names_;

    rclcpp::Time start_time_;

    rclcpp::Publisher<interfaces::msg::CustomJointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LowerSineTestNode>());
    rclcpp::shutdown();
    return 0;
}