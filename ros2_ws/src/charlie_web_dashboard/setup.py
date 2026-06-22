from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'charlie_web_dashboard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        package_name: [
            'templates/*',
            'static/*',
        ],
    },
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            os.path.join('share', package_name),
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py')),
        ),
        (
            os.path.join('share', package_name, 'templates'),
            glob(os.path.join(package_name, 'templates', '*')),
        ),
        (
            os.path.join('share', package_name, 'static'),
            glob(os.path.join(package_name, 'static', '*')),
        ),
    ],
    install_requires=[
        'setuptools',
        'fastapi',
        'uvicorn[standard]',
    ],
    zip_safe=False,
    maintainer='Dyllon Dunton',
    maintainer_email='duntondyllon@gmail.com',
    description='Web dashboard and ROS interface for Charlie.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'charlie_web_dashboard = charlie_web_dashboard.web_dashboard_node:main',
        ],
    },
)