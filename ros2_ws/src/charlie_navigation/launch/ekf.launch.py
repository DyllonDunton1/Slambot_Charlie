from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    odom_topic = LaunchConfiguration("odom_topic")
    publish_tf = LaunchConfiguration("publish_tf")

    ekf_config = PathJoinSubstitution([
        FindPackageShare("charlie_navigation"),
        "config",
        "ekf.yaml",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/odom",
            description="Wheel odometry topic for robot_localization",
        ),

        DeclareLaunchArgument(
            "publish_tf",
            default_value="false",
            description="Whether robot_localization should publish odom -> base_link",
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[
                ekf_config,
                {
                    "odom0": odom_topic,
                    "publish_tf": ParameterValue(publish_tf, value_type=bool),
                },
            ],
        )
    ])
