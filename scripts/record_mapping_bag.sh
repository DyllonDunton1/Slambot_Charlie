#!/usr/bin/env bash
# Record the ROS 2 topics needed to debug Charlie SLAM/mapping field tests.
#
# Usage:
#   bash scripts/record_mapping_bag.sh
#   bash scripts/record_mapping_bag.sh --full
#   bash scripts/record_mapping_bag.sh --prefix mapping_scanmatch_loop
#   bash scripts/record_mapping_bag.sh --output-dir ~/Slambot_Charlie/runtime/bags
#
# Notes:
#   - Run this in a separate terminal after launching mapping/SLAM.
#   - Lite mode is the default so long mapping runs do not explode in size.
#   - Use --full only when you specifically want map/SLAM visualization topics.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
WORKSPACE_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
BAG_ROOT="${HOME}/Slambot_Charlie/runtime/bags"
BAG_PREFIX="mapping_scanmatch"
MODE="lite"

show_usage() {
  cat <<EOF
Usage: $0 [--lite] [--full] [--prefix NAME] [--output-dir DIR]

Options:
  --lite            Record the core SLAM debug topics. Default.
  --full            Add map and SLAM visualization topics when available.
  --prefix NAME     Bag name prefix. Default: ${BAG_PREFIX}
  --output-dir DIR  Output directory. Default: ${BAG_ROOT}
  -h, --help        Show this help message.

Examples:
  bash $0
  bash $0 --full
  bash $0 --prefix mapping_scanmatch_loop_building_a
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
  /base_debug
  /imu/data
  /battery/state
)

FULL_EXTRA_TOPICS=(
  /map
  /map_metadata
  /slam_toolbox/graph_visualization
)

REQUESTED_TOPICS=("${CORE_TOPICS[@]}")
if [[ "${MODE}" == "full" ]]; then
  REQUESTED_TOPICS+=("${FULL_EXTRA_TOPICS[@]}")
fi

mapfile -t AVAILABLE_TOPICS < <(ros2 topic list)
TOPICS=()
MISSING_TOPICS=()

for topic in "${REQUESTED_TOPICS[@]}"; do
  found=false
  for available_topic in "${AVAILABLE_TOPICS[@]}"; do
    if [[ "${available_topic}" == "${topic}" ]]; then
      found=true
      break
    fi
  done

  if [[ "${found}" == "true" ]]; then
    TOPICS+=("${topic}")
  else
    MISSING_TOPICS+=("${topic}")
  fi
done

if [[ ${#TOPICS[@]} -eq 0 ]]; then
  echo "No requested topics are currently available. Launch mapping/SLAM before starting this recorder." >&2
  exit 1
fi

echo "Recording Charlie SLAM mapping rosbag"
echo "Mode: ${MODE}"
echo "Output: ${BAG_PATH}"
echo "Topics:"
printf '  %s\n' "${TOPICS[@]}"

if [[ ${#MISSING_TOPICS[@]} -gt 0 ]]; then
  echo
  echo "Skipping topics that are not currently available:"
  printf '  %s\n' "${MISSING_TOPICS[@]}"
fi

echo
echo "Press Ctrl-C to stop recording."

ros2 bag record "${TOPICS[@]}" -o "${BAG_PATH}"
