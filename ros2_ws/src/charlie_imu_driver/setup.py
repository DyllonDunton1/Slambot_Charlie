from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'charlie_imu_driver'

data_files = [
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    (
        os.path.join('share', package_name),
        ['package.xml'],
    ),
]

launch_files = glob(os.path.join('launch', '*.launch.py'))
if launch_files:
    data_files.append(
        (
            os.path.join('share', package_name, 'launch'),
            launch_files,
        )
    )


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dyllon Dunton',
    maintainer_email='duntondyllon@gmail.com',
    description=(
        "Charlie-specific ROS 2 IMU driver for the ICM-20948. "
        "Publishes gyro yaw-rate as sensor_msgs/Imu on /imu/data."
    ),
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'charlie_imu_node = charlie_imu_driver.icm20948_node:main',
        ],
    },
)