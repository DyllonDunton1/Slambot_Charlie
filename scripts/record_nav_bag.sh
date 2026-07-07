#!/usr/bin/env bash
# Record the ROS 2 topics needed to debug Charlie Nav2 field tests.
#
# Usage:
#   bash scripts/record_nav_bag.sh
#   bash scripts/record_nav_bag.sh --lite
#   bash scripts/record_nav_bag.sh --prefix nav_loop_test
#   bash scripts/record_nav_bag.sh --output-dir ~/Slambot_Charlie/runtime/bags
#
# Notes:
#   - Run this in a separate terminal after launching Nav2.
#   - Use --lite if the Raspberry Pi starts dropping messages or feels overloaded.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
BAG_ROOT="${HOME}/Slambot_Charlie/runtime/bags"
BAG_PREFIX="nav_loop"
MODE="full"

show_usage() {
  cat <<EOF
Usage: $0 [--lite] [--full] [--prefix NAME] [--output-dir DIR]

Options:
  --lite            Record only the most important, lower-bandwidth debug topics.
  --full            Record the full Nav2 field-test topic set. Default.
  --prefix NAME     Bag name prefix. Default: ${BAG_PREFIX}
  --output-dir DIR  Output directory. Default: ${BAG_ROOT}
  -h, --help        Show this help message.

Examples:
  bash $0
  bash $0 --lite
  bash $0 --prefix nav_loop_building_a
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --lite)
      MODE="lite"
      ;;
    --full)
      MODE="full"
      ;;
    --prefix)
      shift
      if [[ $# -eq 0 ]]; then
        echo "Missing value for --prefix" >&2
        exit 1
      fi
      BAG_PREFIX="$1"
      ;;
    --output-dir|-o)
      shift
      if [[ $# -eq 0 ]]; then
        echo "Missing value for --output-dir" >&2
        exit 1
      fi
      BAG_ROOT="$1"
      ;;
    -h|--help)
      show_usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      show_usage >&2
      exit 1
      ;;
  esac
  shift
done

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "Missing ROS setup file: ${ROS_SETUP}" >&2
  exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "Missing workspace setup file: ${WORKSPACE_SETUP}" >&2
  echo "Build the workspace first: cd ${REPO_ROOT}/ros2_ws && colcon build --symlink-install" >&2
  exit 1
fi

# Keep nounset disabled while sourcing ROS setup files. Some ament setup hooks
# read optional environment variables that may not be defined yet.
# shellcheck source=/dev/null
source "${ROS_SETUP}"
# shellcheck source=/dev/null
source "${WORKSPACE_SETUP}"

mkdir -p "${BAG_ROOT}"
BAG_PATH="${BAG_ROOT}/${BAG_PREFIX}_$(date +%Y%m%d_%H%M%S)"

CORE_TOPICS=(
  /tf
  /tf_static
  /scan
  /odom
  /cmd_vel
  /cmd_vel_nav
  /cmd_vel_smoothed
  /amcl_pose
  /plan
  /battery/state
  /base_debug
  /imu/data
)

FULL_EXTRA_TOPICS=(
  /particle_cloud
  /map
  /global_costmap/costmap
  /global_costmap/costmap_updates
  /global_costmap/published_footprint
  /local_costmap/costmap
  /local_costmap/costmap_updates
  /local_costmap/published_footprint
  /local_costmap/clearing_endpoints
)

TOPICS=("${CORE_TOPICS[@]}")
if [[ "${MODE}" == "full" ]]; then
  TOPICS+=("${FULL_EXTRA_TOPICS[@]}")
fi

echo "Recording Charlie Nav2 rosbag"
echo "Mode: ${MODE}"
echo "Output: ${BAG_PATH}"
echo "Topics:"
printf '  %s\n' "${TOPICS[@]}"
echo
echo "Press Ctrl-C to stop recording."

ros2 bag record "${TOPICS[@]}" -o "${BAG_PATH}"
