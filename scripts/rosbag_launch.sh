#!/usr/bin/env bash
set -e

REPO_ROOT="${HOME}/Slambot_Charlie"
ROS_DISTRO="${ROS_DISTRO:-humble}"

source "/opt/ros/${ROS_DISTRO}/setup.bash"
source "${REPO_ROOT}/ros2_ws/install/setup.bash"

BAG_DIR="${REPO_ROOT}/runtime/bags"
mkdir -p "${BAG_DIR}"

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_NAME="${BAG_DIR}/new_building_loop_${STAMP}"

echo "Recording rosbag to:"
echo "  ${BAG_NAME}"
echo
echo "Press Ctrl+C to stop recording cleanly."
echo

ros2 bag record -o "${BAG_NAME}" \
  /scan \
  /tf \
  /tf_static \
  /map \
  /odom \
  /wheel/odom \
  /odometry/filtered \
  /cmd_vel \
  /base_debug \
  /battery/state \
  /imu/data