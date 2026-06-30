from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="charlie_imu_driver",
            executable="charlie_imu_node",
            name="charlie_imu_node",
            output="screen",
            parameters=[{
                "i2c_bus": 1,
                "i2c_addr": 0x69,
                "frame_id": "imu_link",
                "publish_rate_hz": 100.0,
                "gyro_z_sign": 1.0,
                "bias_samples": 2000,
            }],
        )
    ])
