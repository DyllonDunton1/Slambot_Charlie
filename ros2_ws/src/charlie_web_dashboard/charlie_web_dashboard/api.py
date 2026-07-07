from pathlib import Path
import time

from fastapi import FastAPI
from fastapi.responses import HTMLResponse, StreamingResponse, Response
from fastapi.staticfiles import StaticFiles
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


def create_app(ros_interface, package_share_dir: Path) -> FastAPI:
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
