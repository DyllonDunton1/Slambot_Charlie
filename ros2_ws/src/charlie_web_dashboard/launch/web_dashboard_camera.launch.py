from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    video_device = LaunchConfiguration("video_device")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    pixel_format = LaunchConfiguration("pixel_format")
    camera_frame_id = LaunchConfiguration("camera_frame_id")
    web_port = LaunchConfiguration("web_port")

    return LaunchDescription([
        DeclareLaunchArgument(
            "video_device",
            default_value="/dev/video0",
            description="Video device path for the Logitech C270 webcam",
        ),


        DeclareLaunchArgument(
            "pixel_format",
            default_value="YUYV",
            description="Camera pixel format.",
        ),

        DeclareLaunchArgument(
            "camera_frame_id",
            default_value="camera_link",
            description="TF frame for the camera",
        ),

        DeclareLaunchArgument(
            "web_port",
            default_value="8000",
            description="Port for the Charlie web dashboard",
        ),

        Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="c270_camera",
            output="screen",
            parameters=[{
                "video_device": video_device,
                "image_size": [640, 480],
                "pixel_format": pixel_format,
                "camera_frame_id": camera_frame_id,
            }],
            remappings=[
                ("image_raw", "/camera/image_raw"),
                ("camera_info", "/camera/camera_info"),
            ],
        ),

        Node(
            package="charlie_web_dashboard",
            executable="charlie_web_dashboard",
            name="charlie_web_dashboard",
            output="screen",
            parameters=[{
                "web_port": web_port,
                "camera_topic": "/camera/image_raw",
            }],
        ),
    ])