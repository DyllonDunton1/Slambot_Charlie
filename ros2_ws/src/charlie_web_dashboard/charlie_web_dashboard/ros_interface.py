import csv
import io
import json
import math
import threading
import time
import numpy as np
from pathlib import Path
from datetime import datetime

import cv2
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node

import tf2_ros
from tf2_ros import TransformException

from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import Image, Imu, BatteryState
from std_msgs.msg import String
from slam_toolbox.srv import SerializePoseGraph, DeserializePoseGraph


class CharlieRosInterface(Node):
    def __init__(self):
        super().__init__("charlie_web_dashboard")

        # ROS topics
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("debug_topic", "/base_debug")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("tuning_command_topic", "/base_tuning_command")
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("battery_topic", "/battery/state")

        # Command behavior
        self.declare_parameter("cmd_publish_rate_hz", 10.0)
        self.declare_parameter("cmd_timeout_s", 0.35)
        self.declare_parameter("max_linear_mps", 0.30)
        self.declare_parameter("max_angular_radps", 1.50)

        # Camera behavior
        self.declare_parameter("jpeg_quality", 70)

        # Logging behavior
        self.declare_parameter("max_debug_log_samples", 20000)        

        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.debug_topic = self.get_parameter("debug_topic").value
        self.camera_topic = self.get_parameter("camera_topic").value
        self.map_topic = self.get_parameter("map_topic").value
        self.tuning_command_topic = self.get_parameter("tuning_command_topic").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.battery_topic = self.get_parameter("battery_topic").value

        self.cmd_publish_rate_hz = float(self.get_parameter("cmd_publish_rate_hz").value)
        self.cmd_timeout_s = float(self.get_parameter("cmd_timeout_s").value)
        self.max_linear_mps = float(self.get_parameter("max_linear_mps").value)
        self.max_angular_radps = float(self.get_parameter("max_angular_radps").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.max_debug_log_samples = int(self.get_parameter("max_debug_log_samples").value)

        self.lock = threading.Lock()

        # Manual command state
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0
        self.last_command_time = time.monotonic()

        # Odometry state
        self.odom_state = {
            "received": False,
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "linear_x": 0.0,
            "angular_z": 0.0,
            "last_update_age_s": None,
        }
        self.last_odom_time = None

        # IMU state
        self.imu_state = {
            "received": False,
            "topic": self.imu_topic,
            "frame_id": "",
            "angular_velocity_z_radps": 0.0,
            "angular_velocity_z_dps": 0.0,
            "abs_angular_velocity_z_radps": 0.0,
            "last_update_age_s": None,
        }
        self.last_imu_time = None

        # Battery state
        self.battery_state = {
            "received": False,
            "topic": self.battery_topic,
            "voltage": None,
            "percentage": None,
            "present": False,
            "status": 0,
            "technology": 0,
            "last_update_age_s": None,
        }
        self.last_battery_time = None

        # Map state
        self.latest_map_msg = None
        self.latest_map_png = None
        self.last_map_time = None
        self.map_metadata = {
            "received": False,
            "topic": self.map_topic,
            "frame_id": "",
            "width": 0,
            "height": 0,
            "resolution": 0.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "last_update_age_s": None,
        }

        # Base debug state
        self.debug_string = "{}"
        self.debug_data = {}
        self.last_debug_time = None

        # Debug logging state
        self.debug_log_enabled = False
        self.debug_log_start_time = None
        self.debug_log_samples = []

        # Camera state
        self.bridge = CvBridge()
        self.latest_jpeg = None
        self.last_camera_time = None

        # Pose State
        self.robot_pose_state = {
            "received": False,
            "frame_id": "map",
            "child_frame_id": "base_link",
            "x": 0.0,
            "y": 0.0,
            "yaw": 0.0,
            "yaw_deg": 0.0,
            "last_update_age_s": None,
            "error": "",
        }
        self.last_robot_pose_time = None

        # Mapping checkpoint state
        self.checkpoint_root = Path.home() / "Slambot_Charlie" / "runtime" / "checkpoints"
        self.checkpoint_root.mkdir(parents=True, exist_ok=True)

        self.last_checkpoint_status = {
            "ok": False,
            "message": "No checkpoint action yet.",
            "latest_checkpoint": None,
            "checkpoint_root": str(self.checkpoint_root),
        }

        self.serialize_map_client = self.create_client(
            SerializePoseGraph,
            "/slam_toolbox/serialize_map",
        )

        self.deserialize_map_client = self.create_client(
            DeserializePoseGraph,
            "/slam_toolbox/deserialize_map",
        )


        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10,
        )

        self.tuning_pub = self.create_publisher(
            String,
            self.tuning_command_topic,
            10,
        )


        # Transform listeneres
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10,
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            self.battery_topic,
            self.battery_callback,
            10,
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10,
        )

        self.debug_sub = self.create_subscription(
            String,
            self.debug_topic,
            self.debug_callback,
            10,
        )

        self.camera_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.camera_callback,
            10,
        )

        # Timers
        self.cmd_timer = self.create_timer(
            1.0 / self.cmd_publish_rate_hz,
            self.publish_cmd_vel,
        )

        self.pose_timer = self.create_timer(
            0.1,
            self.update_robot_pose_from_tf,
        )

        self.get_logger().info("Charlie web dashboard ROS interface started")
        self.get_logger().info(f"Publishing cmd_vel on {self.cmd_vel_topic}")
        self.get_logger().info(f"Subscribing odom on {self.odom_topic}")
        self.get_logger().info(f"Subscribing debug on {self.debug_topic}")
        self.get_logger().info(f"Subscribing camera on {self.camera_topic}")
        self.get_logger().info(f"Subscribing map on {self.map_topic}")
        self.get_logger().info(f"Publishing tuning commands on {self.tuning_command_topic}")
        self.get_logger().info("Mapping checkpoint services configured")
        self.get_logger().info("serialize_map service: /slam_toolbox/serialize_map")
        self.get_logger().info("deserialize_map service: /slam_toolbox/deserialize_map")
        self.get_logger().info(f"Checkpoint root: {self.checkpoint_root}")
        self.get_logger().info(f"Subscribing IMU on {self.imu_topic}")
        self.get_logger().info(f"Subscribing battery on {self.battery_topic}")

    def set_manual_command(self, linear_x: float, angular_z: float):
        linear_x = self.clamp(
            linear_x,
            -self.max_linear_mps,
            self.max_linear_mps,
        )
        angular_z = self.clamp(
            angular_z,
            -self.max_angular_radps,
            self.max_angular_radps,
        )

        with self.lock:
            self.target_linear_x = linear_x
            self.target_angular_z = angular_z
            self.last_command_time = time.monotonic()

    def stop(self):
        self.set_manual_command(0.0, 0.0)

    def publish_cmd_vel(self):
        with self.lock:
            command_age = time.monotonic() - self.last_command_time

            if command_age > self.cmd_timeout_s:
                linear_x = 0.0
                angular_z = 0.0
            else:
                linear_x = self.target_linear_x
                angular_z = self.target_angular_z

        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        self.cmd_vel_pub.publish(msg)

    def odom_callback(self, msg: Odometry):
        q = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

        with self.lock:
            self.last_odom_time = time.monotonic()
            self.odom_state = {
                "received": True,
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "yaw": yaw,
                "linear_x": msg.twist.twist.linear.x,
                "angular_z": msg.twist.twist.angular.z,
                "last_update_age_s": 0.0,
            }
    
    def imu_callback(self, msg: Imu):
        now = time.monotonic()
        gz = msg.angular_velocity.z

        with self.lock:
            self.last_imu_time = now
            self.imu_state = {
                "received": True,
                "topic": self.imu_topic,
                "frame_id": msg.header.frame_id,
                "angular_velocity_z_radps": gz,
                "angular_velocity_z_dps": math.degrees(gz),
                "abs_angular_velocity_z_radps": abs(gz),
                "last_update_age_s": 0.0,
            }

    def battery_callback(self, msg: BatteryState):
        now = time.monotonic()

        percentage = None
        if msg.percentage >= 0.0:
            percentage = max(0.0, min(100.0, float(msg.percentage) * 100.0))

        with self.lock:
            self.last_battery_time = now
            self.battery_state = {
                "received": True,
                "topic": self.battery_topic,
                "voltage": float(msg.voltage),
                "percentage": percentage,
                "present": bool(msg.present),
                "status": int(msg.power_supply_status),
                "technology": int(msg.power_supply_technology),
                "last_update_age_s": 0.0,
            }

    def map_callback(self, msg: OccupancyGrid):
        now = time.monotonic()

        try:
            png_bytes = self.occupancy_grid_to_png(msg)
        except Exception as exc:
            self.get_logger().warn(f"Map PNG conversion failed: {exc}")
            return

        with self.lock:
            self.latest_map_msg = msg
            self.latest_map_png = png_bytes
            self.last_map_time = now

            self.map_metadata = {
                "received": True,
                "topic": self.map_topic,
                "frame_id": msg.header.frame_id,
                "width": msg.info.width,
                "height": msg.info.height,
                "resolution": msg.info.resolution,
                "origin_x": msg.info.origin.position.x,
                "origin_y": msg.info.origin.position.y,
                "last_update_age_s": 0.0,
            }


    def occupancy_grid_to_png(self, msg: OccupancyGrid) -> bytes:
        width = msg.info.width
        height = msg.info.height

        if width == 0 or height == 0:
            raise ValueError("Map width or height is zero")

        grid = np.array(msg.data, dtype=np.int16).reshape((height, width))

        # Create grayscale image.
        # ROS occupancy values:
        #   -1 = unknown
        #    0 = free
        #  100 = occupied
        image = np.zeros((height, width), dtype=np.uint8)

        # Unknown = medium gray
        image[grid < 0] = 128

        # Free = white
        image[grid == 0] = 255

        # Occupied = black
        image[grid > 50] = 0

        # Semi-occupied/probability cells = gradient
        uncertain_mask = (grid > 0) & (grid <= 50)
        image[uncertain_mask] = 255 - (grid[uncertain_mask] * 2).astype(np.uint8)

        # ROS map origin is bottom-left-ish; image origin is top-left.
        image = np.flipud(image)

        success, encoded = cv2.imencode(".png", image)

        if not success:
            raise RuntimeError("cv2.imencode failed for map PNG")

        return encoded.tobytes()


    def get_latest_map_png(self):
        with self.lock:
            return self.latest_map_png


    def get_map_png_filename(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"charlie_map_{timestamp}.png"

    def debug_callback(self, msg: String):
        now = time.monotonic()

        try:
            parsed_debug = json.loads(msg.data)
        except json.JSONDecodeError:
            parsed_debug = {
                "parse_error": True,
                "raw": msg.data,
            }

        with self.lock:
            self.debug_string = msg.data
            self.debug_data = parsed_debug
            self.last_debug_time = now

            if self.debug_log_enabled:
                self.append_debug_log_sample_locked(now, parsed_debug)

    def append_debug_log_sample_locked(self, now: float, debug_data: dict):
        if self.debug_log_start_time is None:
            self.debug_log_start_time = now

        sample = {
            "time_s": now - self.debug_log_start_time,

            "cmd_linear_x": self.target_linear_x,
            "cmd_angular_z": self.target_angular_z,

            "odom_x": self.odom_state.get("x", 0.0),
            "odom_y": self.odom_state.get("y", 0.0),
            "odom_yaw": self.odom_state.get("yaw", 0.0),
            "odom_linear_x": self.odom_state.get("linear_x", 0.0),
            "odom_angular_z": self.odom_state.get("angular_z", 0.0),

            "imu_yaw_rate_radps": self.imu_state.get("angular_velocity_z_radps", 0.0),
            "imu_yaw_rate_dps": self.imu_state.get("angular_velocity_z_dps", 0.0),
        }

        for key, value in debug_data.items():
            sample[key] = value

        self.debug_log_samples.append(sample)

        if len(self.debug_log_samples) > self.max_debug_log_samples:
            self.debug_log_samples = self.debug_log_samples[-self.max_debug_log_samples:]

    def start_debug_log(self):
        with self.lock:
            self.debug_log_enabled = True
            self.debug_log_start_time = time.monotonic()
            self.debug_log_samples = []

        return self.get_debug_log_state()

    def stop_debug_log(self):
        with self.lock:
            self.debug_log_enabled = False

        return self.get_debug_log_state()

    def clear_debug_log(self):
        with self.lock:
            self.debug_log_enabled = False
            self.debug_log_start_time = None
            self.debug_log_samples = []

        return self.get_debug_log_state()

    def get_debug_log_state(self):
        with self.lock:
            duration_s = None
            if self.debug_log_start_time is not None:
                duration_s = time.monotonic() - self.debug_log_start_time

            return {
                "enabled": self.debug_log_enabled,
                "sample_count": len(self.debug_log_samples),
                "duration_s": duration_s,
                "max_samples": self.max_debug_log_samples,
            }

    def get_debug_log_csv(self):
        with self.lock:
            samples = list(self.debug_log_samples)

        output = io.StringIO()

        if not samples:
            output.write("time_s\n")
            return output.getvalue()

        preferred_fields = [
            "time_s",

            "cmd_linear_x",
            "cmd_angular_z",

            "left_target_mps",
            "right_target_mps",
            "left_measured_mps",
            "right_measured_mps",
            "left_error_mps",
            "right_error_mps",
            "left_integral_error_mps",
            "right_integral_error_mps",
            "left_correction",
            "right_correction",

            "odom_x",
            "odom_y",
            "odom_yaw",
            "odom_linear_x",
            "odom_angular_z",

            "imu_yaw_rate_radps",
            "imu_yaw_rate_dps",
        ]

        all_fields = set()
        for sample in samples:
            all_fields.update(sample.keys())

        fieldnames = []
        for field in preferred_fields:
            if field in all_fields:
                fieldnames.append(field)

        for field in sorted(all_fields):
            if field not in fieldnames:
                fieldnames.append(field)

        writer = csv.DictWriter(output, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(samples)

        return output.getvalue()

    def get_debug_log_filename(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return f"charlie_debug_log_{timestamp}.csv"

    def camera_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            success, encoded = cv2.imencode(
                ".jpg",
                cv_image,
                [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
            )

            if not success:
                self.get_logger().warn("Failed to encode camera frame as JPEG")
                return

            with self.lock:
                self.latest_jpeg = encoded.tobytes()
                self.last_camera_time = time.monotonic()

        except Exception as exc:
            self.get_logger().warn(f"Camera conversion failed: {exc}")

    def get_latest_jpeg(self):
        with self.lock:
            return self.latest_jpeg
    
    def send_tuning_command(
        self,
        kp=None,
        ki=None,
        wheel_radius_m=None,
        wheel_separation_m=None,
        reset_integral=False,
    ):
        command = {}

        if kp is not None:
            command["kp"] = float(kp)

        if ki is not None:
            command["ki"] = float(ki)

        if wheel_radius_m is not None:
            command["wheel_radius_m"] = float(wheel_radius_m)

        if wheel_separation_m is not None:
            command["wheel_separation_m"] = float(wheel_separation_m)

        if reset_integral:
            command["reset_integral"] = True

        msg = String()
        msg.data = json.dumps(command)

        self.tuning_pub.publish(msg)

        return {
            "ok": True,
            "command": command,
            "topic": self.tuning_command_topic,
        }

    def update_robot_pose_from_tf(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map",
                "base_link",
                rclpy.time.Time(),
            )

            t = transform.transform.translation
            q = transform.transform.rotation

            yaw = self.quaternion_to_yaw(q.x, q.y, q.z, q.w)

            with self.lock:
                self.last_robot_pose_time = time.monotonic()
                self.robot_pose_state = {
                    "received": True,
                    "frame_id": "map",
                    "child_frame_id": "base_link",
                    "x": t.x,
                    "y": t.y,
                    "yaw": yaw,
                    "yaw_deg": math.degrees(yaw),
                    "last_update_age_s": 0.0,
                    "error": "",
                }

        except TransformException as exc:
            with self.lock:
                self.robot_pose_state["received"] = False
                self.robot_pose_state["error"] = str(exc)

    def wait_for_service(self, client, service_name: str, timeout_s: float = 2.0) -> bool:
        start = time.monotonic()

        while time.monotonic() - start < timeout_s:
            if client.service_is_ready():
                return True

            client.wait_for_service(timeout_sec=0.1)

        self.get_logger().warn(f"Service not available: {service_name}")
        return False


    def wait_for_future(self, future, timeout_s: float = 10.0):
        start = time.monotonic()

        while time.monotonic() - start < timeout_s:
            if future.done():
                return future.result()

            time.sleep(0.05)

        raise TimeoutError("Service call timed out")


    def get_latest_checkpoint_dir(self):
        if not self.checkpoint_root.exists():
            return None

        checkpoint_dirs = [
            path for path in self.checkpoint_root.iterdir()
            if path.is_dir() and path.name.startswith("checkpoint_")
        ]

        if not checkpoint_dirs:
            return None

        return sorted(checkpoint_dirs, key=lambda p: p.name)[-1]


    def get_checkpoint_list(self):
        if not self.checkpoint_root.exists():
            return []

        checkpoints = []

        for path in sorted(self.checkpoint_root.iterdir(), key=lambda p: p.name):
            if not path.is_dir() or not path.name.startswith("checkpoint_"):
                continue

            metadata_path = path / f"{path.name}.json"

            item = {
                "name": path.name,
                "directory": str(path),
                "metadata_path": str(metadata_path),
                "has_metadata": metadata_path.exists(),
            }

            if metadata_path.exists():
                try:
                    item["metadata"] = json.loads(metadata_path.read_text())
                except Exception as exc:
                    item["metadata_error"] = str(exc)

            checkpoints.append(item)

        return checkpoints


    def save_mapping_checkpoint(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        checkpoint_name = f"checkpoint_{timestamp}"

        checkpoint_dir = self.checkpoint_root / checkpoint_name
        checkpoint_dir.mkdir(parents=True, exist_ok=True)

        posegraph_base_path = checkpoint_dir / checkpoint_name
        metadata_path = checkpoint_dir / f"{checkpoint_name}.json"

        with self.lock:
            robot_pose = dict(self.robot_pose_state)
            odom_pose = dict(self.odom_state)

        if not robot_pose.get("received", False):
            result = {
                "ok": False,
                "message": "Cannot save checkpoint because map->base_link pose is not available.",
                "checkpoint": checkpoint_name,
            }
            self.last_checkpoint_status = {
                "ok": False,
                "message": result["message"],
                "latest_checkpoint": None,
                "checkpoint_root": str(self.checkpoint_root),
            }
            return result

        if not self.wait_for_service(
            self.serialize_map_client,
            "/slam_toolbox/serialize_map",
            timeout_s=3.0,
        ):
            result = {
                "ok": False,
                "message": "slam_toolbox serialize_map service is not available.",
                "checkpoint": checkpoint_name,
            }
            self.last_checkpoint_status = {
                "ok": False,
                "message": result["message"],
                "latest_checkpoint": None,
                "checkpoint_root": str(self.checkpoint_root),
            }
            return result

        request = SerializePoseGraph.Request()
        request.filename = str(posegraph_base_path)

        try:
            future = self.serialize_map_client.call_async(request)
            response = self.wait_for_future(future, timeout_s=15.0)
        except Exception as exc:
            result = {
                "ok": False,
                "message": f"serialize_map failed: {exc}",
                "checkpoint": checkpoint_name,
            }
            self.last_checkpoint_status = {
                "ok": False,
                "message": result["message"],
                "latest_checkpoint": None,
                "checkpoint_root": str(self.checkpoint_root),
            }
            return result

        # SerializePoseGraph result code:
        # 0 = success, 255 = failed to write file
        serialize_result = getattr(response, "result", None)

        if serialize_result != SerializePoseGraph.Response.RESULT_SUCCESS:
            result = {
                "ok": False,
                "message": f"serialize_map returned non-success result: {serialize_result}",
                "checkpoint": checkpoint_name,
                "posegraph_base_path": str(posegraph_base_path),
            }
            self.last_checkpoint_status = {
                "ok": False,
                "message": result["message"],
                "latest_checkpoint": None,
                "checkpoint_root": str(self.checkpoint_root),
            }
            return result

        metadata = {
            "name": checkpoint_name,
            "created_at": datetime.now().isoformat(),
            "checkpoint_dir": str(checkpoint_dir),
            "posegraph_base_path": str(posegraph_base_path),
            "metadata_path": str(metadata_path),
            "saved_map_pose": {
                "frame_id": "map",
                "child_frame_id": "base_link",
                "x": robot_pose.get("x", 0.0),
                "y": robot_pose.get("y", 0.0),
                "yaw": robot_pose.get("yaw", 0.0),
                "yaw_deg": robot_pose.get("yaw_deg", 0.0),
            },
            "saved_odom_pose": {
                "frame_id": "odom",
                "child_frame_id": "base_link",
                "x": odom_pose.get("x", 0.0),
                "y": odom_pose.get("y", 0.0),
                "yaw": odom_pose.get("yaw", 0.0),
                "linear_x": odom_pose.get("linear_x", 0.0),
                "angular_z": odom_pose.get("angular_z", 0.0),
            },
            "note": "Load this checkpoint only if Charlie has not physically moved since saving.",
        }

        metadata_path.write_text(json.dumps(metadata, indent=2))

        result = {
            "ok": True,
            "message": "Mapping checkpoint saved.",
            "checkpoint": checkpoint_name,
            "checkpoint_dir": str(checkpoint_dir),
            "posegraph_base_path": str(posegraph_base_path),
            "metadata_path": str(metadata_path),
            "serialize_result": serialize_result,
            "metadata": metadata,
        }

        self.last_checkpoint_status = {
            "ok": True,
            "message": result["message"],
            "latest_checkpoint": checkpoint_name,
            "checkpoint_root": str(self.checkpoint_root),
        }

        return result


    def load_latest_mapping_checkpoint(self):
        checkpoint_dir = self.get_latest_checkpoint_dir()

        if checkpoint_dir is None:
            result = {
                "ok": False,
                "message": "No mapping checkpoints found.",
                "checkpoint_root": str(self.checkpoint_root),
            }
            self.last_checkpoint_status = {
                "ok": False,
                "message": result["message"],
                "latest_checkpoint": None,
                "checkpoint_root": str(self.checkpoint_root),
            }
            return result

        return self.load_mapping_checkpoint(checkpoint_dir.name)


    def load_mapping_checkpoint(self, checkpoint_name: str):
        checkpoint_dir = self.checkpoint_root / checkpoint_name
        metadata_path = checkpoint_dir / f"{checkpoint_name}.json"

        if not metadata_path.exists():
            result = {
                "ok": False,
                "message": f"Checkpoint metadata not found: {metadata_path}",
                "checkpoint": checkpoint_name,
            }
            self.last_checkpoint_status = {
                "ok": False,
                "message": result["message"],
                "latest_checkpoint": checkpoint_name,
                "checkpoint_root": str(self.checkpoint_root),
            }
            return result

        try:
            metadata = json.loads(metadata_path.read_text())
        except Exception as exc:
            result = {
                "ok": False,
                "message": f"Failed to read checkpoint metadata: {exc}",
                "checkpoint": checkpoint_name,
            }
            self.last_checkpoint_status = {
                "ok": False,
                "message": result["message"],
                "latest_checkpoint": checkpoint_name,
                "checkpoint_root": str(self.checkpoint_root),
            }
            return result

        posegraph_base_path = metadata["posegraph_base_path"]
        saved_pose = metadata["saved_map_pose"]

        if not self.wait_for_service(
            self.deserialize_map_client,
            "/slam_toolbox/deserialize_map",
            timeout_s=3.0,
        ):
            result = {
                "ok": False,
                "message": "slam_toolbox deserialize_map service is not available.",
                "checkpoint": checkpoint_name,
            }
            self.last_checkpoint_status = {
                "ok": False,
                "message": result["message"],
                "latest_checkpoint": checkpoint_name,
                "checkpoint_root": str(self.checkpoint_root),
            }
            return result

        initial_pose = Pose2D()
        initial_pose.x = float(saved_pose["x"])
        initial_pose.y = float(saved_pose["y"])
        initial_pose.theta = float(saved_pose["yaw"])

        request = DeserializePoseGraph.Request()
        request.filename = posegraph_base_path

        # slam_toolbox/srv/DeserializePoseGraph:
        # START_AT_GIVEN_POSE = 2
        request.match_type = 2
        request.initial_pose = initial_pose

        try:
            future = self.deserialize_map_client.call_async(request)
            response = self.wait_for_future(future, timeout_s=15.0)
        except Exception as exc:
            result = {
                "ok": False,
                "message": f"deserialize_map failed: {exc}",
                "checkpoint": checkpoint_name,
            }
            self.last_checkpoint_status = {
                "ok": False,
                "message": result["message"],
                "latest_checkpoint": checkpoint_name,
                "checkpoint_root": str(self.checkpoint_root),
            }
            return result

        deserialize_result = getattr(response, "result", None)

        # Some versions define result codes; keep this tolerant.
        success = deserialize_result in (None, 0)

        result = {
            "ok": success,
            "message": "Mapping checkpoint loaded." if success else f"deserialize_map returned result: {deserialize_result}",
            "checkpoint": checkpoint_name,
            "checkpoint_dir": str(checkpoint_dir),
            "posegraph_base_path": posegraph_base_path,
            "metadata_path": str(metadata_path),
            "deserialize_result": deserialize_result,
            "initial_pose": {
                "x": initial_pose.x,
                "y": initial_pose.y,
                "theta": initial_pose.theta,
            },
            "metadata": metadata,
        }

        self.last_checkpoint_status = {
            "ok": success,
            "message": result["message"],
            "latest_checkpoint": checkpoint_name,
            "checkpoint_root": str(self.checkpoint_root),
        }

        return result

    def get_status(self):
        now = time.monotonic()

        with self.lock:
            command_age = now - self.last_command_time

            debug_age = None
            if self.last_debug_time is not None:
                debug_age = now - self.last_debug_time

            odom = dict(self.odom_state)
            if self.last_odom_time is not None:
                odom["last_update_age_s"] = now - self.last_odom_time

            imu = dict(self.imu_state)
            if self.last_imu_time is not None:
                imu["last_update_age_s"] = now - self.last_imu_time

            battery = dict(self.battery_state)
            if self.last_battery_time is not None:
                battery["last_update_age_s"] = now - self.last_battery_time

            camera_age = None
            camera_received = self.latest_jpeg is not None

            if self.last_camera_time is not None:
                camera_age = now - self.last_camera_time
            
            map_metadata = dict(self.map_metadata)

            if self.last_map_time is not None:
                map_metadata["last_update_age_s"] = now - self.last_map_time

            debug_log_duration_s = None
            if self.debug_log_start_time is not None:
                debug_log_duration_s = now - self.debug_log_start_time
            
            robot_pose = dict(self.robot_pose_state)

            if self.last_robot_pose_time is not None:
                robot_pose["last_update_age_s"] = now - self.last_robot_pose_time

            status = {
                "command": {
                    "linear_x": self.target_linear_x,
                    "angular_z": self.target_angular_z,
                    "age_s": command_age,
                },
                "limits": {
                    "max_linear_mps": self.max_linear_mps,
                    "max_angular_radps": self.max_angular_radps,
                },
                "odom": odom,
                "imu": imu,
                "battery": battery,
                "debug": {
                    "received": self.last_debug_time is not None,
                    "topic": self.debug_topic,
                    "last_update_age_s": debug_age,
                    "data": dict(self.debug_data),
                },
                "debug_log": {
                    "enabled": self.debug_log_enabled,
                    "sample_count": len(self.debug_log_samples),
                    "duration_s": debug_log_duration_s,
                    "max_samples": self.max_debug_log_samples,
                },
                "camera": {
                    "received": camera_received,
                    "topic": self.camera_topic,
                    "last_update_age_s": camera_age,
                },
                "map": map_metadata,
                "robot_pose": robot_pose,
                "checkpoint": dict(self.last_checkpoint_status),
            }

            return status

    @staticmethod
    def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def clamp(value: float, low: float, high: float) -> float:
        return max(low, min(high, value))