#!/usr/bin/env bash
# Launch Charlie with optional Fast DDS unicast discovery, EKF mode, and
# fastforward mode.
#
# This is intentionally kept as a thin startup wrapper so the normal ROS launch
# files remain middleware-agnostic and this can later move into a systemd unit.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
DDS_PROFILE="${REPO_ROOT}/config/dds/fastdds_unicast_discovery.xml"

USE_UNICAST=false
USE_EKF=false
USE_FASTFORWARD=false
LAUNCH_ARGS=()

show_usage() {
  cat <<EOF
Usage: $0 [-unicast] [-ekf] [-fastforward] [additional bringup launch args]

Options:
  -unicast      Use the Fast DDS unicast discovery profile.
  -ekf          Launch bringup with ekf:=true.
  -fastforward  Launch bringup with fastforward:=true.
  -h, --help    Show this help message.

Examples:
  $0
  $0 -unicast
  $0 -ekf
  $0 -fastforward
  $0 -unicast -ekf -fastforward
  $0 -unicast mapping:=false
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
    -fastforward)
      USE_FASTFORWARD=true
      ;;
    -h|--help)
      show_usage
      exit 0
      ;;
    *)
      LAUNCH_ARGS+=("$1")
      ;;
  esac
  shift
done

if [[ "${USE_EKF}" == "true" ]]; then
  LAUNCH_ARGS+=("ekf:=true")
fi

if [[ "${USE_FASTFORWARD}" == "true" ]]; then
  LAUNCH_ARGS+=("fastforward:=true")
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

if [[ "${USE_EKF}" == "true" ]]; then
  echo "Launching with EKF enabled: ekf:=true"
fi

if [[ "${USE_FASTFORWARD}" == "true" ]]; then
  echo "Launching with fastforward enabled: fastforward:=true"
fi

exec ros2 launch charlie_bringup bringup.launch.py "${LAUNCH_ARGS[@]}"
