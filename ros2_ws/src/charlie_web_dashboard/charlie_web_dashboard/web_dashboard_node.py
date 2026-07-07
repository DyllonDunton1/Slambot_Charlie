import threading
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor

import uvicorn

from charlie_web_dashboard.api import create_app
from charlie_web_dashboard.ros_interface import CharlieRosInterface


def spin_ros_node(node: CharlieRosInterface):
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    finally:
        executor.shutdown()


def main(args=None):
    rclpy.init(args=args)

    ros_interface = CharlieRosInterface()

    package_share_dir = Path(__file__).parent
    app = create_app(ros_interface, package_share_dir)

    web_host = str(ros_interface.get_parameter("web_host").value)
    web_port = int(ros_interface.get_parameter("web_port").value)

    ros_thread = threading.Thread(
        target=spin_ros_node,
        args=(ros_interface,),
        daemon=True,
    )
    ros_thread.start()

    try:
        uvicorn.run(
            app,
            host=web_host,
            port=web_port,
            log_level="info",
        )
    finally:
        ros_interface.stop()
        ros_interface.destroy_node()
        rclpy.shutdown()
