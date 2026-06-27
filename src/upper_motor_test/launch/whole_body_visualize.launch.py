from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare("upper_motor_test")

    model = DeclareLaunchArgument(
        "model",
        default_value=PathJoinSubstitution(
            [pkg_share, "urdf", "wholebody_4_dof_arm_20deg_with_head_RL.urdf"]
        ),
    )

    rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [pkg_share, "rviz", "whole_body.rviz"]
        ),
    )

    use_gui = DeclareLaunchArgument(
        "use_gui",
        default_value="false",
    )

    use_rviz = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
    )

    robot_description_content = Command([
        "cat ",
        LaunchConfiguration("model")
    ])

    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description},
            {"publish_frequency": 50.0},
        ],
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gui")),
        parameters=[
            {"robot_description": robot_description},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        arguments=["-d", LaunchConfiguration("rviz_config")],
    )

    static_map_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_base",
        arguments=[
            "0", "0", "0.90",   # x y z
            "0", "0", "0",      # roll pitch yaw
            "map",              # parent frame
            "base",             # child frame
        ],
    )

    return LaunchDescription([
        model,
        rviz_config,
        use_gui,
        use_rviz,
        robot_state_publisher_node,
        static_map_to_base,
        joint_state_publisher_gui_node,
        rviz_node,
    ])