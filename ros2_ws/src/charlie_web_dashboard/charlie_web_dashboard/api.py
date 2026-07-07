from datetime import datetime
from pathlib import Path
import time

import cv2
import numpy as np
from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, Response
from fastapi.staticfiles import StaticFiles
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from pydantic import BaseModel


class CmdVelRequest(BaseModel):
    linear_x: float = 0.0
    angular_z: float = 0.0


class TuningRequest(BaseModel):
    kp: float | None = None
    ki: float | None = None
    wheel_radius_m: float | None = None
    wheel_separation_m: float | None = None
    reset_integral: bool = False


class CheckpointLoadRequest(BaseModel):
    name: str | None = None


class ControlModeRequest(BaseModel):
    mode: str


class WaypointRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0


class InitialPoseRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0
    covariance_xy: float = 0.25
    covariance_yaw: float = 0.0685


def occupancy_grid_to_pgm_bytes(msg, occupied_thresh: float = 0.65, free_thresh: float = 0.25) -> bytes:
    width = int(msg.info.width)
    height = int(msg.info.height)

    if width <= 0 or height <= 0:
        raise ValueError("Map width or height is zero")

    occupied_cutoff = int(occupied_thresh * 100.0)
    free_cutoff = int(free_thresh * 100.0)
    data = list(msg.data)

    # ROS occupancy grids are stored from the map origin upward. PGM images are
    # viewed from top-left, so write rows in reverse order like nav2 map_saver.
    pixels = bytearray()
    for y in reversed(range(height)):
        row_start = y * width
        for x in range(width):
            occ = int(data[row_start + x])

            if occ < 0:
                pixel = 205  # unknown
            elif occ >= occupied_cutoff:
                pixel = 0    # occupied
            elif occ <= free_cutoff:
                pixel = 254  # free
            else:
                pixel = 205  # unknown / uncertain

            pixels.append(pixel)

    header = f"P5\n# CREATOR: Charlie web dashboard\n{width} {height}\n255\n".encode("ascii")
    return header + bytes(pixels)


def costmap_grid_to_overlay_png(msg: OccupancyGrid) -> bytes:
    """Convert a Nav2 costmap OccupancyGrid into a transparent overlay PNG.

    This intentionally differs from the normal map image. Free cells are fully
    transparent so the saved/static map remains visible. Unknown cells are faint
    gray, inflated/high-cost cells are orange, and lethal cells are red.
    """
    width = int(msg.info.width)
    height = int(msg.info.height)

    if width <= 0 or height <= 0:
        raise ValueError("Costmap width or height is zero")

    grid = np.array(msg.data, dtype=np.int16).reshape((height, width))

    # OpenCV encodes four-channel PNGs as BGRA.
    overlay = np.zeros((height, width, 4), dtype=np.uint8)

    unknown_mask = grid < 0
    inflated_mask = (grid > 0) & (grid < 100)
    lethal_mask = grid >= 100

    overlay[unknown_mask] = [128, 128, 128, 70]

    if np.any(inflated_mask):
        costs = grid[inflated_mask].astype(np.float32)
        alpha = np.clip(45.0 + costs * 1.35, 50.0, 180.0).astype(np.uint8)
        overlay[inflated_mask, 0] = 0      # B
        overlay[inflated_mask, 1] = 165    # G
        overlay[inflated_mask, 2] = 255    # R
        overlay[inflated_mask, 3] = alpha  # A

    overlay[lethal_mask] = [0, 0, 255, 220]

    # ROS map origin is bottom-left-ish; image origin is top-left.
    overlay = np.flipud(overlay)

    success, encoded = cv2.imencode(".png", overlay)

    if not success:
        raise RuntimeError("cv2.imencode failed for global costmap overlay PNG")

    return encoded.tobytes()


def ensure_global_costmap_subscription(ros_interface):
    topic = "/global_costmap/costmap"

    if getattr(ros_interface, "global_costmap_sub", None) is not None:
        return

    with ros_interface.lock:
        ros_interface.latest_global_costmap_png = None
        ros_interface.last_global_costmap_time = None
        ros_interface.global_costmap_metadata = {
            "received": False,
            "topic": topic,
            "frame_id": "",
            "width": 0,
            "height": 0,
            "resolution": 0.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "last_update_age_s": None,
        }

    def global_costmap_callback(msg: OccupancyGrid):
        now = time.monotonic()

        try:
            png_bytes = costmap_grid_to_overlay_png(msg)
        except Exception as exc:
            ros_interface.get_logger().warn(f"Global costmap PNG conversion failed: {exc}")
            return

        with ros_interface.lock:
            ros_interface.latest_global_costmap_png = png_bytes
            ros_interface.last_global_costmap_time = now
            ros_interface.global_costmap_metadata = {
                "received": True,
                "topic": topic,
                "frame_id": msg.header.frame_id,
                "width": int(msg.info.width),
                "height": int(msg.info.height),
                "resolution": float(msg.info.resolution),
                "origin_x": float(msg.info.origin.position.x),
                "origin_y": float(msg.info.origin.position.y),
                "last_update_age_s": 0.0,
            }

    ros_interface.global_costmap_sub = ros_interface.create_subscription(
        OccupancyGrid,
        topic,
        global_costmap_callback,
        10,
    )
    ros_interface.get_logger().info(f"Subscribing Nav2 global costmap on {topic}")


def get_global_costmap_png(ros_interface):
    with ros_interface.lock:
        return getattr(ros_interface, "latest_global_costmap_png", None)


def get_global_costmap_status(ros_interface):
    now = time.monotonic()

    with ros_interface.lock:
        metadata = dict(getattr(ros_interface, "global_costmap_metadata", {
            "received": False,
            "topic": "/global_costmap/costmap",
            "frame_id": "",
            "width": 0,
            "height": 0,
            "resolution": 0.0,
            "origin_x": 0.0,
            "origin_y": 0.0,
            "last_update_age_s": None,
        }))
        last_time = getattr(ros_interface, "last_global_costmap_time", None)

    if last_time is not None:
        metadata["last_update_age_s"] = now - last_time

    return metadata


def save_latest_nav_map(ros_interface):
    with ros_interface.lock:
        msg = ros_interface.latest_map_msg

    if msg is None:
        return {
            "ok": False,
            "message": "No /map has been received yet. Start mapping or Nav2 map_server first.",
        }

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    map_name = f"nav_map_{timestamp}"
    map_dir = Path.home() / "Slambot_Charlie" / "runtime" / "maps"
    map_dir.mkdir(parents=True, exist_ok=True)

    pgm_path = map_dir / f"{map_name}.pgm"
    yaml_path = map_dir / f"{map_name}.yaml"

    occupied_thresh = 0.65
    free_thresh = 0.25

    try:
        pgm_path.write_bytes(
            occupancy_grid_to_pgm_bytes(
                msg,
                occupied_thresh=occupied_thresh,
                free_thresh=free_thresh,
            )
        )

        origin = msg.info.origin
        yaw = ros_interface.quaternion_to_yaw(
            origin.orientation.x,
            origin.orientation.y,
            origin.orientation.z,
            origin.orientation.w,
        )

        yaml_text = "\n".join([
            f"image: {pgm_path.name}",
            "mode: trinary",
            f"resolution: {float(msg.info.resolution):.12g}",
            f"origin: [{float(origin.position.x):.12g}, {float(origin.position.y):.12g}, {float(yaw):.12g}]",
            "negate: 0",
            f"occupied_thresh: {occupied_thresh}",
            f"free_thresh: {free_thresh}",
            "",
        ])
        yaml_path.write_text(yaml_text)

    except Exception as exc:
        return {
            "ok": False,
            "message": f"Failed to save Nav2 map: {exc}",
            "directory": str(map_dir),
        }

    return {
        "ok": True,
        "message": "Saved Nav2 occupancy map.",
        "map_name": map_name,
        "directory": str(map_dir),
        "yaml_path": str(yaml_path),
        "image_path": str(pgm_path),
        "yaml_filename": yaml_path.name,
        "image_filename": pgm_path.name,
    }


def publish_initial_pose(ros_interface, req: InitialPoseRequest):
    if not hasattr(ros_interface, "initial_pose_pub"):
        ros_interface.initial_pose_pub = ros_interface.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            10,
        )

    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = ros_interface.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(req.x)
    msg.pose.pose.position.y = float(req.y)
    msg.pose.pose.position.z = 0.0

    qx, qy, qz, qw = ros_interface.yaw_to_quaternion(float(req.yaw))
    msg.pose.pose.orientation.x = qx
    msg.pose.pose.orientation.y = qy
    msg.pose.pose.orientation.z = qz
    msg.pose.pose.orientation.w = qw

    covariance = [0.0] * 36
    covariance[0] = float(req.covariance_xy)
    covariance[7] = float(req.covariance_xy)
    covariance[35] = float(req.covariance_yaw)
    msg.pose.covariance = covariance

    # Publish a few times to give AMCL/RViz-style subscribers a solid chance to
    # receive the one-shot initial pose even if discovery has just completed.
    for _ in range(3):
        ros_interface.initial_pose_pub.publish(msg)
        time.sleep(0.05)

    return {
        "ok": True,
        "message": "Published AMCL initial pose.",
        "topic": "/initialpose",
        "pose": {
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "yaw": float(req.yaw),
        },
    }


def create_app(ros_interface, package_share_dir: Path) -> FastAPI:
    ensure_global_costmap_subscription(ros_interface)

    app = FastAPI(title="Charlie Web Dashboard")

    static_dir = package_share_dir / "static"
    templates_dir = package_share_dir / "templates"

    if static_dir.exists():
        app.mount("/static", StaticFiles(directory=str(static_dir)), name="static")

    @app.get("/", response_class=HTMLResponse)
    def index():
        index_path = templates_dir / "index.html"

        if not index_path.exists():
            return HTMLResponse(
                "<h1>Charlie Web Dashboard</h1><p>index.html not found.</p>",
                status_code=200,
            )

        return HTMLResponse(index_path.read_text())

    @app.post("/api/cmd_vel")
    def set_cmd_vel(cmd: CmdVelRequest):
        return ros_interface.set_manual_command(cmd.linear_x, cmd.angular_z)

    @app.post("/api/control_mode")
    def set_control_mode(req: ControlModeRequest):
        return ros_interface.set_control_mode(req.mode)

    @app.post("/api/stop")
    def stop():
        ros_interface.stop()
        return {"ok": True, "stopped": True, "mode": "manual"}

    @app.get("/api/status")
    def status():
        return ros_interface.get_status()

    @app.get("/api/health")
    def health():
        return {"ok": True, "name": "charlie_web_dashboard"}

    @app.get("/api/video_feed")
    def video_feed():
        def frame_generator():
            boundary = b"--frame\r\n"

            while True:
                frame = ros_interface.get_latest_jpeg()

                if frame is not None:
                    yield (
                        boundary
                        + b"Content-Type: image/jpeg\r\n"
                        + b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n"
                        + frame
                        + b"\r\n"
                    )

                time.sleep(0.05)

        return StreamingResponse(
            frame_generator(),
            media_type="multipart/x-mixed-replace; boundary=frame",
        )

    @app.post("/api/debug_log/start")
    def start_debug_log():
        return ros_interface.start_debug_log()

    @app.post("/api/debug_log/stop")
    def stop_debug_log():
        return ros_interface.stop_debug_log()

    @app.post("/api/debug_log/clear")
    def clear_debug_log():
        return ros_interface.clear_debug_log()

    @app.get("/api/debug_log/download")
    def download_debug_log():
        csv_text = ros_interface.get_debug_log_csv()
        filename = ros_interface.get_debug_log_filename()

        return Response(
            content=csv_text,
            media_type="text/csv",
            headers={
                "Content-Disposition": f'attachment; filename="{filename}"'
            },
        )

    @app.get("/api/map/image")
    def map_image():
        png_bytes = ros_interface.get_latest_map_png()

        if png_bytes is None:
            return Response(
                content="No map has been received yet.",
                status_code=404,
                media_type="text/plain",
            )

        return Response(
            content=png_bytes,
            media_type="image/png",
        )

    @app.get("/api/costmap/global/image")
    def global_costmap_image():
        png_bytes = get_global_costmap_png(ros_interface)

        if png_bytes is None:
            return Response(
                content="No /global_costmap/costmap has been received yet.",
                status_code=404,
                media_type="text/plain",
            )

        return Response(
            content=png_bytes,
            media_type="image/png",
        )

    @app.get("/api/costmap/global/status")
    def global_costmap_status():
        return get_global_costmap_status(ros_interface)

    @app.get("/api/map/download")
    def download_map_png():
        png_bytes = ros_interface.get_latest_map_png()

        if png_bytes is None:
            return Response(
                content="No map has been received yet.",
                status_code=404,
                media_type="text/plain",
            )

        filename = ros_interface.get_map_png_filename()

        return Response(
            content=png_bytes,
            media_type="image/png",
            headers={
                "Content-Disposition": f'attachment; filename="{filename}"'
            },
        )

    @app.post("/api/map/save_nav")
    def save_nav_map():
        return save_latest_nav_map(ros_interface)

    @app.post("/api/tuning")
    def set_tuning(req: TuningRequest):
        return ros_interface.send_tuning_command(
            kp=req.kp,
            ki=req.ki,
            wheel_radius_m=req.wheel_radius_m,
            wheel_separation_m=req.wheel_separation_m,
            reset_integral=req.reset_integral,
        )

    @app.post("/api/checkpoint/save")
    def save_checkpoint():
        return ros_interface.save_mapping_checkpoint()

    @app.post("/api/checkpoint/load_latest")
    def load_latest_checkpoint():
        return ros_interface.load_latest_mapping_checkpoint()

    @app.post("/api/checkpoint/load")
    def load_checkpoint(req: CheckpointLoadRequest):
        if req.name is None or req.name.strip() == "":
            return ros_interface.load_latest_mapping_checkpoint()

        return ros_interface.load_mapping_checkpoint(req.name.strip())

    @app.get("/api/checkpoint/list")
    def list_checkpoints():
        return {
            "ok": True,
            "checkpoints": ros_interface.get_checkpoint_list(),
        }

    @app.post("/api/nav/initial_pose")
    def set_initial_pose(req: InitialPoseRequest):
        return publish_initial_pose(ros_interface, req)

    @app.post("/api/nav/waypoints")
    def add_waypoint(req: WaypointRequest):
        return ros_interface.add_waypoint(req.x, req.y, req.yaw)

    @app.post("/api/nav/waypoints/clear")
    def clear_waypoints():
        return ros_interface.clear_waypoints()

    @app.post("/api/nav/compute_path")
    def compute_path():
        return ros_interface.compute_waypoint_path()

    @app.post("/api/nav/follow_path")
    def follow_path():
        return ros_interface.follow_waypoints()

    @app.post("/api/nav/cancel")
    def cancel_nav():
        return ros_interface.cancel_navigation(set_manual_mode=True)

    return app
