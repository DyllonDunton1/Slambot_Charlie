from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    wheel_separation = LaunchConfiguration("wheel_separation")

    video_device = LaunchConfiguration("video_device")
    pixel_format = LaunchConfiguration("pixel_format")

    web_dashboard_camera_launch = PathJoinSubstitution([
        FindPackageShare("charlie_web_dashboard"),
        "launch",
        "web_dashboard_camera.launch.py",
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "serial_port",
            default_value="/dev/ttyACM0",
            description="Serial port connected to the Teensy",
        ),

        DeclareLaunchArgument(
            "baud_rate",
            default_value="115200",
            description="Serial baud rate for Teensy communication",
        ),

        DeclareLaunchArgument(
            "wheel_separation",
            default_value="0.210",
            description="Distance between left and right wheels in meters",
        ),

        DeclareLaunchArgument(
            "video_device",
            default_value="/dev/video0",
            description="Video device path for the Logitech C270 webcam",
        ),

        DeclareLaunchArgument(
            "pixel_format",
            default_value="YUYV",
            description="Camera pixel format. YUYV works reliably with v4l2_camera.",
        ),

        Node(
            package="charlie_base_driver",
            executable="base_driver_node",
            name="base_driver_node",
            output="screen",
            parameters=[{
                "serial_port": serial_port,
                "baud_rate": baud_rate,
                "wheel_separation": wheel_separation,
                "cmd_timeout_s": 0.5,
                "command_rate_hz": 50.0,
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_tf": True,
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_dashboard_camera_launch),
            launch_arguments={
                "video_device": video_device,
                "pixel_format": pixel_format,
            }.items(),
        ),
    ])