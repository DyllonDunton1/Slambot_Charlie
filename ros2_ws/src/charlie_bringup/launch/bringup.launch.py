from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.conditions import IfCondition


def generate_launch_description():
    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    wheel_separation = LaunchConfiguration("wheel_separation")

    video_device = LaunchConfiguration("video_device")
    pixel_format = LaunchConfiguration("pixel_format")

    lidar_port = LaunchConfiguration("lidar_port")

    mapping = LaunchConfiguration("mapping")
    ekf = LaunchConfiguration("ekf")

    base_odom_topic = PythonExpression([
        "'/wheel/odom' if '", ekf, "' == 'true' else '/odom'",
    ])
    base_publish_tf = ParameterValue(
        PythonExpression(["'", ekf, "' != 'true'"]),
        value_type=bool,
    )

    web_dashboard_camera_launch = PathJoinSubstitution([
        FindPackageShare("charlie_web_dashboard"),
        "launch",
        "web_dashboard_camera.launch.py",
    ])

    description_launch = PathJoinSubstitution([
        FindPackageShare("charlie_description"),
        "launch",
        "description.launch.py",
    ])

    imu_launch = PathJoinSubstitution([
        FindPackageShare("charlie_imu_driver"),
        "launch",
        "imu.launch.py",
    ])

    ekf_launch = PathJoinSubstitution([
        FindPackageShare("charlie_navigation"),
        "launch",
        "ekf.launch.py",
    ])

    mapping_launch = PathJoinSubstitution([
        FindPackageShare("charlie_navigation"),
        "launch",
        "mapping.launch.py",
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
            default_value="0.221",
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

        DeclareLaunchArgument(
            "lidar_port",
            default_value="/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
            description="Serial port for the ROBOTIS LDS LiDAR",
        ),

        DeclareLaunchArgument(
            "mapping",
            default_value="true",
            description="Start slam_toolbox mapping",
        ),

        DeclareLaunchArgument(
            "ekf",
            default_value="false",
            description="Start EKF localization and let robot_localization own odom -> base_link",
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
                "publish_tf": base_publish_tf,
            }],
            remappings=[
                ("/odom", base_odom_topic),
            ],
        ),

        Node(
            package="hls_lfcd_lds_driver",
            executable="hlds_laser_publisher",
            name="lidar",
            output="screen",
            parameters=[{
                "port": lidar_port,
                "frame_id": "laser_frame",
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_dashboard_camera_launch),
            launch_arguments={
                "video_device": video_device,
                "pixel_format": pixel_format,
            }.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(imu_launch),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch),
            launch_arguments={
                "odom_topic": "/wheel/odom",
                "publish_tf": "true",
            }.items(),
            condition=IfCondition(ekf),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch),
            condition=IfCondition(mapping),
        ),

    ])
