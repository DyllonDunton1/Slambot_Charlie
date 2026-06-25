# Slambot Charlie

Charlie is a Raspberry Pi + Teensy ROS 2 SLAM robot built as a hands-on robotics engineering project. It combines differential-drive motion, closed-loop stepper control, wheel odometry, LiDAR mapping, a live camera feed, and a custom web dashboard for field testing.

The goal of this project is simple: build a real robot that can drive, map, report what it is doing, and eventually navigate on its own. It is still a prototype, but the architecture is intentionally kept clean so each subsystem can be tested, explained, and improved without turning the whole robot into a mystery box.

## Current status

Working now:

- ROS 2 Humble workspace on a Raspberry Pi 4B
- Teensy 4.1 firmware for low-level stepper motor control and encoder feedback
- Closed-loop wheel speed control with runtime tuning
- Differential-drive wheel odometry
- ROBOTIS serial LiDAR on `/scan`
- Logitech C270 USB camera on `/camera/image_raw`
- `slam_toolbox` mapping with pose graph checkpoint save/load
- Custom FastAPI dashboard for teleoperation, camera, map display, tuning, debug logs, and checkpoints
- URDF/Xacro model with `base_link`, wheel frames, `laser_frame`, `camera_link`, and `imu_link`

Still in progress:

- SparkFun ICM-20948 IMU integration
- `robot_localization` EKF fusion
- Switching wheel odometry from `/odom` to `/wheel/odom` once the EKF owns `odom -> base_link`
- Long hallway / loop-closure mapping tests
- Autonomous navigation with Nav2
- Final demo videos and portfolio writeup

One practical note: the dashboard pose marker now tracks well during normal use. It can occasionally blip slightly off and come back, but that is being treated as a minor visualization/timing issue rather than a core mapping blocker.

## System overview

```text
Browser dashboard
  -> FastAPI dashboard node
  -> ROS 2 topics/services

Dashboard
  publishes /cmd_vel
  publishes /base_tuning_command
  subscribes /odom, /base_debug, /camera/image_raw, /map
  looks up map -> base_link
  calls slam_toolbox checkpoint services

Raspberry Pi base driver
  subscribes /cmd_vel
  publishes /odom
  publishes /base_debug
  publishes odom -> base_link TF
  sends serial commands to Teensy

Teensy firmware
  receives wheel speed targets
  controls stepper drivers
  reads wheel encoders
  sends odometry/debug packets back to the Pi

LiDAR
  publishes /scan

slam_toolbox
  consumes /scan and TF
  publishes /map
  publishes map -> odom
```

Current TF tree:

```text
map
└── odom
    └── base_link
        ├── left_wheel_link
        ├── right_wheel_link
        ├── laser_frame
        ├── camera_link
        └── imu_link
```

Future localization target:

```text
base_driver_node -> /wheel/odom, no odom TF
imu_node         -> /imu/data
robot_localization -> /odometry/filtered and odom -> base_link
slam_toolbox     -> uses fused odom/TF
```

## Architecture diagrams

These simplified diagrams are intended for portfolio presentation and quick repository orientation.

### Hardware Architecture

![Hardware Architecture](architecture/hardware_architecture.png)

### Software Architecture

![Software Architecture](architecture/software_architecture.png)

## Repository layout

```text
Slambot_Charlie/
├── README.md
├── architecture/
│   ├── hardware_architecture.md
│   ├── hardware_architecture.png
│   ├── software_architecture.md
│   └── software_architecture.png
├── ros2_ws/
│   └── src/
│       ├── charlie_bringup/
│       │   └── launch/bringup.launch.py
│       ├── charlie_base_driver/
│       │   ├── include/charlie_base_driver/
│       │   └── src/
│       ├── charlie_web_dashboard/
│       │   ├── charlie_web_dashboard/
│       │   │   ├── api.py
│       │   │   ├── ros_interface.py
│       │   │   ├── web_dashboard_node.py
│       │   │   ├── static/
│       │   │   └── templates/
│       │   └── launch/web_dashboard_camera.launch.py
│       ├── charlie_description/
│       │   ├── launch/description.launch.py
│       │   └── urdf/charlie.urdf.xacro
│       └── charlie_navigation/
│           ├── config/slam_toolbox.yaml
│           └── launch/mapping.launch.py
└── teensy_firmware/
    ├── platformio.ini
    ├── include/
    └── src/
```

## Main packages

### `charlie_bringup`

Top-level launch package. The main launch file starts the base driver, LiDAR, camera/dashboard, robot description, and mapping stack.

```bash
ros2 launch charlie_bringup bringup.launch.py
```

Mapping is enabled by default, but can be controlled with:

```bash
ros2 launch charlie_bringup bringup.launch.py mapping:=true
```

### `charlie_base_driver`

C++ ROS 2 node that bridges the Raspberry Pi and Teensy. It converts `/cmd_vel` into left/right wheel speed targets, sends those targets over serial, parses odometry/debug packets from the Teensy, publishes `/odom`, publishes `/base_debug`, and currently broadcasts `odom -> base_link`.

Current serial protocol:

```text
Pi -> Teensy:
V <left_mps> <right_mps>
C KP <value>
C KI <value>
C WHEEL_RADIUS <value>
C RESET_I

Teensy -> Pi:
O <left_total_m> <right_total_m> <left_speed_mps> <right_speed_mps> <status>
D <debug fields...>
```

### `charlie_web_dashboard`

Python/FastAPI dashboard used as the field operator console. It provides:

- browser teleoperation
- live camera feed
- live map image
- robot pose overlay
- runtime tuning controls
- debug log recording/download
- SLAM Toolbox checkpoint save/load

The dashboard runs on port `8000`.

```text
http://<robot-ip>:8000
```

### `charlie_description`

Xacro/URDF model for the robot frames and basic geometry. The model intentionally uses simple shapes instead of detailed meshes so TF and sensor placement stay easy to inspect.

Coordinate convention:

```text
x = forward
y = left
z = up
```

Important frames:

```text
base_link
left_wheel_link
right_wheel_link
laser_frame
camera_link
imu_link
```

### `charlie_navigation`

Mapping and navigation configuration. It currently contains the `slam_toolbox` mapping launch and YAML config. Scan matching is currently disabled because odometry-only mapping has behaved better in the current flat-wall test environment.

Future EKF/Nav2 configuration should live here as the navigation stack grows.

### `teensy_firmware`

PlatformIO firmware for the Teensy 4.1. It handles stepper output, AS5600 encoder reads, PI wheel speed control, serial command parsing, odometry packets, and debug packets.

Known firmware pin map:

| Signal | Teensy pin |
|---|---:|
| Left STEP | 29 |
| Left DIR | 31 |
| Left ENABLE | 32 |
| Right STEP | 1 |
| Right DIR | 2 |
| Right ENABLE | 3 |
| Default I2C SDA | 18 |
| Default I2C SCL | 19 |

The left AS5600 encoder uses the default `Wire` bus and the right encoder uses `Wire1`. Both use I2C address `0x36` in the current firmware.

## Hardware summary

| Subsystem | Current hardware |
|---|---|
| Main computer | Raspberry Pi 4B |
| Microcontroller | Teensy 4.1 |
| Drive | Differential-drive stepper motors |
| Motor drivers | TMC stepper drivers, exact model still to be documented |
| Encoders | AS5600-style magnetic encoders |
| LiDAR | ROBOTIS serial LiDAR via `hls_lfcd_lds_driver` |
| Camera | Logitech C270 USB webcam |
| IMU | SparkFun ICM-20948, planned but not integrated yet |
| Battery | 3S 18650 lithium-ion pack |
| Prototype wiring | Solderable protoboard / practical hobbyist wiring |

## Build and run

Source ROS 2 and build the workspace:

```bash
source /opt/ros/humble/setup.bash
cd ~/Slambot_Charlie/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Launch the robot:

```bash
ros2 launch charlie_bringup bringup.launch.py
```

Useful checks:

```bash
ros2 node list
ros2 topic list
ros2 topic hz /odom
ros2 topic hz /scan
ros2 topic hz /map
ros2 run tf2_ros tf2_echo map base_link
ros2 service list | grep slam_toolbox
```

## Firmware build / upload

The Teensy firmware is a PlatformIO project.

```bash
cd ~/Slambot_Charlie/teensy_firmware
pio run
pio run -t upload
```

Before uploading, stop anything that may be holding the Teensy serial port:

```bash
sudo fuser -k /dev/ttyACM0
```

The base driver and any serial monitor should be stopped before flashing.

## Mapping workflow

1. Start the robot bringup launch.
2. Open the dashboard in a browser.
3. Drive slowly while watching the map and camera feed.
4. Save a checkpoint before stopping for battery charging.
5. Only reload a checkpoint if the robot has not physically moved since the save.

Checkpointing uses SLAM Toolbox pose graph serialization, not just a saved map image. That matters because it preserves the graph state needed to keep mapping from the same physical pose.

## Roadmap

Near-term work:

- Add ICM-20948 IMU node publishing `/imu/data` with `frame_id: imu_link`
- Validate gyro sign conventions before fusion
- Add `robot_localization` EKF config
- Change base driver output from `/odom` to `/wheel/odom` when EKF is enabled
- Let the EKF publish `odom -> base_link`
- Retest hallway mapping with fused odometry

Longer-term work:

- Revisit SLAM scan matching after IMU/EKF improves yaw stability
- Add Nav2 for autonomous navigation
- Improve README with screenshots and demo videos
- Record a clean portfolio demo showing driving, mapping, camera, tuning, and checkpointing

## Project notes

This robot is intentionally built with practical parts and build methods: a Raspberry Pi, a Teensy, ROS 2, protoboard wiring, 3D-printed structure, and off-the-shelf sensors. The point is not to hide the messiness of a real prototype. The point is to make the mess understandable, testable, and gradually more reliable.
