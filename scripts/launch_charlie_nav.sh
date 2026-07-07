#!/usr/bin/env bash
# Launch Charlie's normal robot bringup without SLAM, then start Nav2 on a saved map.
#
# Defaults are intentionally conservative:
#   - manual dashboard control is the default dashboard mode
#   - slam_toolbox mapping is disabled so AMCL owns map -> odom
#   - EKF is disabled unless requested, so the base driver owns odom -> base_link

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
DDS_PROFILE="${REPO_ROOT}/config/dds/fastdds_unicast_discovery.xml"

USE_UNICAST=false
USE_EKF=false
MAP_YAML="${HOME}/Slambot_Charlie/runtime/maps/test_map.yaml"
BRINGUP_ARGS=()

show_usage() {
  cat <<EOF
Usage: $0 [-unicast] [-ekf] [-map /path/to/map.yaml] [additional bringup launch args]

Options:
  -unicast        Use the Fast DDS unicast discovery profile.
  -ekf            Launch bringup with ekf:=true.
  -map PATH       Saved Nav2 map YAML to load. Default: ${MAP_YAML}
  -h, --help      Show this help message.

Examples:
  $0 -map ~/Slambot_Charlie/runtime/maps/test_map.yaml
  $0 -unicast -map ~/Slambot_Charlie/runtime/maps/new_building_loop.yaml
  $0 -ekf -map ~/Slambot_Charlie/runtime/maps/test_map.yaml
  $0 -map ~/Slambot_Charlie/runtime/maps/test_map.yaml wheel_separation:=0.2066
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -unicast)
      USE_UNICAST=true
      ;;
    -ekf)
      USE_EKF=true
      ;;
    -map|--map)
      shift
      if [[ $# -eq 0 ]]; then
        echo "Missing value for -map" >&2
        exit 1
      fi
      MAP_YAML="$1"
      ;;
    -h|--help)
      show_usage
      exit 0
      ;;
    *)
      BRINGUP_ARGS+=("$1")
      ;;
  esac
  shift
done

if [[ "${USE_EKF}" == "true" ]]; then
  BRINGUP_ARGS+=("ekf:=true")
fi

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup file: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "Missing workspace setup file: ${WORKSPACE_SETUP}" >&2
  echo "Build the workspace first: cd ${REPO_ROOT}/ros2_ws && colcon build --symlink-install" >&2
  exit 1
fi

if [[ ! -f "${MAP_YAML}" ]]; then
  echo "Missing map YAML: ${MAP_YAML}" >&2
  echo "Save a map first, or pass -map /path/to/map.yaml" >&2
  exit 1
fi

if [[ "${USE_UNICAST}" == "true" && ! -f "${DDS_PROFILE}" ]]; then
  echo "Missing Fast DDS profile: ${DDS_PROFILE}" >&2
  exit 1
fi

# Keep nounset disabled while sourcing ROS setup files. Some ament setup hooks
# read optional environment variables that may not be defined yet.
# shellcheck source=/dev/null
source "${ROS_SETUP}"
# shellcheck source=/dev/null
source "${WORKSPACE_SETUP}"

set -u

if [[ "${USE_UNICAST}" == "true" ]]; then
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTDDS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}"
  echo "Using RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
  echo "Using FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE}"
else
  echo "Using default ROS 2 middleware discovery"
fi

echo "Launching Charlie bringup for Nav2: mapping:=false"
echo "Launching Nav2 with map: ${MAP_YAML}"

cleanup() {
  echo "Stopping Charlie Nav launch processes..."
  if [[ -n "${NAV_PID:-}" ]]; then
    kill "${NAV_PID}" 2>/dev/null || true
  fi
  if [[ -n "${BRINGUP_PID:-}" ]]; then
    kill "${BRINGUP_PID}" 2>/dev/null || true
  fi
  wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

ros2 launch charlie_bringup bringup.launch.py mapping:=false "${BRINGUP_ARGS[@]}" &
BRINGUP_PID=$!

# Give robot_state_publisher, LiDAR, odom, and the dashboard a moment to come up
# before Nav2 lifecycle nodes start checking transforms.
sleep 4

ros2 launch charlie_navigation navigation.launch.py map:="${MAP_YAML}" &
NAV_PID=$!

wait -n "${BRINGUP_PID}" "${NAV_PID}"
