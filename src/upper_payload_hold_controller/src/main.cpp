#include "rclcpp/rclcpp.hpp"
#include "upper_payload_hold_controller/PayloadHoldNode.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<upper_payload_hold_controller::PayloadHoldNode>());

    rclcpp::shutdown();

    return 0;
}