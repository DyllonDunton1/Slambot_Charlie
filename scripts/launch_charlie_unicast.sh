#!/usr/bin/env bash
# Launch Charlie with Fast DDS unicast discovery.
#
# This is intentionally kept as a thin startup wrapper so the normal ROS launch
# files remain middleware-agnostic and this can later move into a systemd unit.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
DDS_PROFILE="${REPO_ROOT}/config/dds/fastdds_unicast_discovery.xml"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup file: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "Missing workspace setup file: ${WORKSPACE_SETUP}" >&2
  echo "Build the workspace first: cd ${REPO_ROOT}/ros2_ws && colcon build --symlink-install" >&2
  exit 1
fi

if [[ ! -f "${DDS_PROFILE}" ]]; then
  echo "Missing Fast DDS profile: ${DDS_PROFILE}" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "${ROS_SETUP}"
# shellcheck source=/dev/null
source "${WORKSPACE_SETUP}"

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTDDS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}"

echo "Using RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "Using FASTDDS_DEFAULT_PROFILES_FILE=${FASTDDS_DEFAULT_PROFILES_FILE}"

exec ros2 launch charlie_bringup bringup.launch.py "$@"
