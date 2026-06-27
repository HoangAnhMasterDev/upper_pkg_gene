#include <cmath>
#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/custom_joint_state.hpp"
using namespace std::chrono_literals;

class UpperSineTestNode : public rclcpp::Node
{
public:
    UpperSineTestNode()
    : Node("upper_sine_test_node")
    {
        topic_name_ = declare_parameter<std::string>("topic_name", "/upper_motor/command");

        amplitude_ = declare_parameter<double>("amplitude", 0.2);      // rad
        frequency_ = declare_parameter<double>("frequency", 0.5);      // Hz
        q_center_  = declare_parameter<double>("q_center", 0.0);       // rad

        kp_ = declare_parameter<double>("kp", 5.0);
        kd_ = declare_parameter<double>("kd", 0.5);

        publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 100.0);

        joint_names_ = {
            "L_shoulder_pitch",
            "L_shoulder_roll",
            "L_shoulder_yaw",
            "L_elbow",
            "R_shoulder_pitch",
            "R_shoulder_roll",
            "R_shoulder_yaw",
            "R_elbow"
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
            std::bind(&UpperSineTestNode::onTimer, this));

        RCLCPP_INFO(get_logger(), "Upper sine test started.");
        RCLCPP_INFO(get_logger(), "Topic: %s", topic_name_.c_str());
        RCLCPP_INFO(get_logger(), "L_shoulder_pitch sine: amp=%.3f rad, freq=%.3f Hz",
                    amplitude_, frequency_);
    }

private:
    void onTimer()
    {
        const double t = (now() - start_time_).seconds();
        const double omega = 2.0 * M_PI * frequency_;

        const double q_cmd  = q_center_ + amplitude_ * std::sin(omega * t);
        const double dq_cmd = amplitude_ * omega * std::cos(omega * t);

        interfaces::msg::CustomJointState msg;

        msg.state.name = joint_names_;

        msg.state.position.assign(8, 0.0);
        msg.state.velocity.assign(8, 0.0);
        msg.state.effort.assign(8, 0.0);
        msg.kp.assign(8, 0.0);
        msg.kd.assign(8, 0.0);

        // Index 0 = L_shoulder_pitch
        msg.state.position[0] = q_cmd;
        msg.state.velocity[0] = dq_cmd;
        msg.kp[0] = kp_;
        msg.kd[0] = kd_;

        pub_->publish(msg);
    }

private:
    std::string topic_name_;

    double amplitude_{0.2};
    double frequency_{0.1};
    double q_center_{0.0};

    double kp_{5.0};
    double kd_{0.5};

    double publish_rate_hz_{100.0};

    std::vector<std::string> joint_names_;

    rclcpp::Time start_time_;

    rclcpp::Publisher<interfaces::msg::CustomJointState>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UpperSineTestNode>());
    rclcpp::shutdown();
    return 0;
}