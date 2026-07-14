#!/usr/bin/env bash
set -uo pipefail

REPO_ROOT="${HOME}/Slambot_Charlie"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SAMPLE_SECONDS=12
USE_UNICAST=false
DDS_PROFILE="${REPO_ROOT}/config/dds/fastdds_unicast_discovery.xml"

show_usage() {
    cat <<EOF
Usage: $0 [-unicast] [sample_seconds]

Options:
  -unicast      Use Charlie's Fast DDS localhost unicast discovery profile.
  -h, --help    Show this help message.
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -unicast)
            USE_UNICAST=true
            ;;
        -h|--help)
            show_usage
            exit 0
            ;;
        *)
            if [[ "$1" =~ ^[0-9]+$ ]]; then
                SAMPLE_SECONDS="$1"
            else
                echo "Unknown argument: $1" >&2
                show_usage >&2
                exit 2
            fi
            ;;
    esac
    shift
done

if ! [[ "${SAMPLE_SECONDS}" =~ ^[0-9]+$ ]] || (( SAMPLE_SECONDS < 3 )); then
    echo "Sampling duration must be an integer of at least 3 seconds." >&2
    exit 2
fi

set +u
source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${REPO_ROOT}/ros2_ws/install/setup.bash"
set -u

if [[ "${USE_UNICAST}" == "true" ]]; then
    if [[ ! -f "${DDS_PROFILE}" ]]; then
        echo "Fast DDS profile not found: ${DDS_PROFILE}" >&2
        exit 1
    fi
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}"
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_ROOT="${REPO_ROOT}/runtime/diagnostics/topic_rates_${STAMP}"
mkdir -p "${OUTPUT_ROOT}"

export CHARLIE_RATE_SAMPLE_SECONDS="${SAMPLE_SECONDS}"
export CHARLIE_RATE_OUTPUT_ROOT="${OUTPUT_ROOT}"
export CHARLIE_RATE_USE_UNICAST="${USE_UNICAST}"

python3 - <<'PY'
import csv
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message

SPECS = [
    ("/scan", 4.0, 6.5, True, "LiDAR scan; expected around 5 Hz"),
    ("/odom", 8.0, 60.0, True, "Base or active odometry stream"),
    ("/imu/data", 15.0, 200.0, True, "ICM-20948 gyro data"),
    ("/base_debug", 5.0, 60.0, True, "Base-driver debug telemetry"),
    ("/battery/state", 0.5, 20.0, False, "Battery telemetry"),
    ("/map", 0.3, 3.0, False, "SLAM map updates; mapping mode only"),
    ("/wheel/odom", 8.0, 60.0, False, "Wheel odometry; active EKF mode only"),
    ("/odometry/filtered", 15.0, 50.0, False, "robot_localization EKF output"),
    ("/cmd_vel", 5.0, 30.0, False, "Final velocity command; measure while moving"),
    ("/cmd_vel_nav", 5.0, 20.0, False, "Nav2 controller output; active navigation only"),
    ("/plan", 0.1, 10.0, False, "Nav2 global plan; only updates when planning/replanning"),
    ("/global_costmap/costmap", 0.5, 2.0, False, "Nav2 global costmap publication"),
    ("/local_costmap/costmap", 1.0, 6.5, False, "Nav2 local costmap publication"),
]

sample_seconds = float(os.environ["CHARLIE_RATE_SAMPLE_SECONDS"])
out_dir = Path(os.environ["CHARLIE_RATE_OUTPUT_ROOT"])
summary_txt = out_dir / "summary.txt"
summary_csv = out_dir / "summary.csv"
visible_topics_file = out_dir / "visible_topics.txt"
env_file = out_dir / "ros_environment.txt"
raw_counts_file = out_dir / "message_counts.csv"

rclpy.init()
node = Node("charlie_topic_rate_checker")

# Give discovery a moment to populate.
discovery_deadline = time.monotonic() + 3.0
while time.monotonic() < discovery_deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

topic_types = {name: types for name, types in node.get_topic_names_and_types()}
visible_topics_file.write_text("\n".join(sorted(topic_types)) + "\n")

env_file.write_text(
    f"ROS_DISTRO={os.environ.get('ROS_DISTRO', 'unknown')}\n"
    f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '0 (default)')}\n"
    f"RMW_IMPLEMENTATION={os.environ.get('RMW_IMPLEMENTATION', 'default')}\n"
    f"FASTDDS_DEFAULT_PROFILES_FILE={os.environ.get('FASTDDS_DEFAULT_PROFILES_FILE', 'unset')}\n"
    f"USE_UNICAST={os.environ.get('CHARLIE_RATE_USE_UNICAST', 'false')}\n"
)

counts = {topic: 0 for topic, *_ in SPECS}
first_time = {topic: None for topic, *_ in SPECS}
last_time = {topic: None for topic, *_ in SPECS}
subscriptions = []
subscription_errors = {}

qos = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=50,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

for topic, *_ in SPECS:
    types = topic_types.get(topic, [])
    if not types:
        continue
    try:
        msg_type = get_message(types[0])
    except Exception as exc:
        subscription_errors[topic] = f"type import failed: {exc}"
        continue

    def make_callback(topic_name):
        def callback(_msg):
            now = time.monotonic()
            counts[topic_name] += 1
            if first_time[topic_name] is None:
                first_time[topic_name] = now
            last_time[topic_name] = now
        return callback

    try:
        subscriptions.append(node.create_subscription(msg_type, topic, make_callback(topic), qos))
    except Exception as exc:
        subscription_errors[topic] = f"subscription failed: {exc}"

print(f"Sampling {len(subscriptions)} visible topics for {sample_seconds:.0f}s...")
start = time.monotonic()
while time.monotonic() - start < sample_seconds:
    rclpy.spin_once(node, timeout_sec=0.1)

rows = []
fail_count = warning_count = pass_count = inactive_count = 0
overall_exit = 0

for topic, min_hz, max_hz, required, description in SPECS:
    count = counts[topic]
    if count >= 2 and first_time[topic] is not None and last_time[topic] is not None:
        elapsed = last_time[topic] - first_time[topic]
        rate = (count - 1) / elapsed if elapsed > 0 else 0.0
        observed = f"{rate:.2f}"
        if min_hz <= rate <= max_hz:
            status = "PASS"
            pass_count += 1
        elif required:
            status = "FAIL-LOW" if rate < min_hz else "FAIL-HIGH"
            fail_count += 1
            overall_exit = 1
        else:
            status = "WARN-LOW" if rate < min_hz else "WARN-HIGH"
            warning_count += 1
    else:
        observed = "--"
        if required:
            status = "FAIL"
            fail_count += 1
            overall_exit = 1
        else:
            status = "INACTIVE"
            inactive_count += 1

    note = description
    if topic in subscription_errors:
        note += f" | {subscription_errors[topic]}"
    elif topic not in topic_types:
        note += " | topic not present"
    elif count == 1:
        note += " | only one message received"

    rows.append((topic, observed, min_hz, max_hz, "required" if required else "optional", status, note, count))

with raw_counts_file.open("w", newline="") as handle:
    writer = csv.writer(handle)
    writer.writerow(["topic", "message_count", "first_time", "last_time"])
    for topic, *_ in SPECS:
        writer.writerow([topic, counts[topic], first_time[topic], last_time[topic]])

with summary_csv.open("w", newline="") as handle:
    writer = csv.writer(handle)
    writer.writerow(["topic", "observed_hz", "min_hz", "max_hz", "required", "status", "description", "message_count"])
    writer.writerows(rows)

lines = [
    "Charlie ROS 2 Topic Frequency Report",
    f"Generated: {time.strftime('%Y-%m-%dT%H:%M:%S%z')}",
    f"Sample duration: {sample_seconds:.0f} seconds",
    f"Visible ROS topics: {len(topic_types)}",
    f"Unicast profile: {os.environ.get('CHARLIE_RATE_USE_UNICAST', 'false')}",
    f"Output directory: {out_dir}",
    "",
    f"{'TOPIC':34} {'OBSERVED':10} {'EXPECTED':17} {'STATUS':12} NOTES",
    f"{'-'*34} {'-'*10} {'-'*17} {'-'*12} {'-'*30}",
]
for topic, observed, min_hz, max_hz, _required, status, note, _count in rows:
    lines.append(f"{topic:34} {observed:10} {f'{min_hz}-{max_hz} Hz':17} {status:12} {note}")

lines.extend([
    "",
    f"Totals: {pass_count} pass, {warning_count} warning, {fail_count} fail, {inactive_count} inactive optional",
    "",
    f"Visible topic list: {visible_topics_file}",
    f"ROS environment snapshot: {env_file}",
    f"Message counts: {raw_counts_file}",
    f"CSV summary: {summary_csv}",
])
summary_txt.write_text("\n".join(lines) + "\n")
print(summary_txt.read_text(), end="")

node.destroy_node()
rclpy.shutdown()
sys.exit(overall_exit)
PY
