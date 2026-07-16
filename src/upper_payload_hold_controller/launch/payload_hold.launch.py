from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory("upper_payload_hold_controller")

    config_file = os.path.join(
        pkg_share,
        "config",
        "payload_hold.yaml"
    )

    return LaunchDescription([
        Node(
            package="upper_payload_hold_controller",
            executable="upper_payload_hold_controller_node",
            name="upper_payload_hold_controller_node",
            output="screen",
            parameters=[config_file],
        )
    ])