from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ekf_config = PathJoinSubstitution([
        FindPackageShare("charlie_navigation"),
        "config",
        "ekf.yaml",
    ])

    return LaunchDescription([
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config],
        )
    ])