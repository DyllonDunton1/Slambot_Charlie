#!/usr/bin/env bash
set -uo pipefail

# Measure important Charlie ROS 2 topic rates, save raw output, and compare the
# observed averages against expected ranges.
#
# Usage:
#   ./scripts/check_ros2_topic_rates.sh
#   ./scripts/check_ros2_topic_rates.sh 20
#   ./scripts/check_ros2_topic_rates.sh -unicast
#   ./scripts/check_ros2_topic_rates.sh -unicast 20
#
# The optional integer is the sampling duration in seconds per topic.

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
                Use this when Charlie was launched with -unicast.
  -h, --help    Show this help message.

Examples:
  $0
  $0 30
  $0 -unicast
  $0 -unicast 30
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

STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_ROOT="${REPO_ROOT}/runtime/diagnostics/topic_rates_${STAMP}"
SUMMARY_TXT="${OUTPUT_ROOT}/summary.txt"
SUMMARY_CSV="${OUTPUT_ROOT}/summary.csv"
RAW_DIR="${OUTPUT_ROOT}/raw"
VISIBLE_TOPICS_FILE="${OUTPUT_ROOT}/visible_topics.txt"
ENV_FILE="${OUTPUT_ROOT}/ros_environment.txt"

if ! [[ "${SAMPLE_SECONDS}" =~ ^[0-9]+$ ]] || (( SAMPLE_SECONDS < 3 )); then
    echo "Sampling duration must be an integer of at least 3 seconds." >&2
    exit 2
fi

# ROS/ament setup scripts reference optional variables that may be unset.
set +u
if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
    exit 1
fi

if [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/ros2_ws/install/setup.bash"
else
    echo "Workspace setup not found: ${REPO_ROOT}/ros2_ws/install/setup.bash" >&2
    echo "Build and source the workspace before running this script." >&2
    exit 1
fi
set -u

if [[ "${USE_UNICAST}" == "true" ]]; then
    if [[ ! -f "${DDS_PROFILE}" ]]; then
        echo "Fast DDS profile not found: ${DDS_PROFILE}" >&2
        exit 1
    fi
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTDDS_DEFAULT_PROFILES_FILE="${DDS_PROFILE}"
fi

mkdir -p "${RAW_DIR}"

{
    printf 'ROS_DISTRO=%s\n' "${ROS_DISTRO}"
    printf 'ROS_DOMAIN_ID=%s\n' "${ROS_DOMAIN_ID:-0 (default)}"
    printf 'RMW_IMPLEMENTATION=%s\n' "${RMW_IMPLEMENTATION:-default}"
    printf 'FASTDDS_DEFAULT_PROFILES_FILE=%s\n' "${FASTDDS_DEFAULT_PROFILES_FILE:-unset}"
    printf 'USE_UNICAST=%s\n' "${USE_UNICAST}"
} > "${ENV_FILE}"

# Avoid stale ros2daemon discovery state before checking the graph.
ros2 daemon stop >/dev/null 2>&1 || true
sleep 1

if ! timeout 8 ros2 topic list > "${VISIBLE_TOPICS_FILE}" 2>&1; then
    echo "Unable to query the ROS 2 topic graph." >&2
    cat "${VISIBLE_TOPICS_FILE}" >&2
    exit 1
fi

visible_topic_count="$(grep -c '^/' "${VISIBLE_TOPICS_FILE}" || true)"
if (( visible_topic_count == 0 )); then
    echo "No ROS 2 topics are visible from this shell." >&2
    echo >&2
    cat "${ENV_FILE}" >&2
    echo >&2
    echo "If Charlie was launched with -unicast, run:" >&2
    echo "  $0 -unicast ${SAMPLE_SECONDS}" >&2
    echo >&2
    echo "Visible-topic output saved to: ${VISIBLE_TOPICS_FILE}" >&2
    exit 1
fi

# Format:
#   topic|min_hz|max_hz|required|description
TOPIC_SPECS=(
    "/scan|4.0|6.5|required|LiDAR scan; expected around 5 Hz"
    "/odom|8.0|60.0|required|Base or active odometry stream"
    "/imu/data|15.0|200.0|required|ICM-20948 gyro data"
    "/base_debug|5.0|60.0|required|Base-driver debug telemetry"
    "/battery/state|0.5|20.0|optional|Battery telemetry"
    "/map|0.3|3.0|optional|SLAM map updates; mapping mode only"
    "/wheel/odom|8.0|60.0|optional|Wheel odometry; active EKF mode only"
    "/odometry/filtered|15.0|50.0|optional|robot_localization EKF output"
    "/cmd_vel|5.0|30.0|optional|Final velocity command; measure while moving"
    "/cmd_vel_nav|5.0|20.0|optional|Nav2 controller output; active navigation only"
    "/plan|0.1|10.0|optional|Nav2 global plan; only updates when planning/replanning"
    "/global_costmap/costmap|0.5|2.0|optional|Nav2 global costmap publication"
    "/local_costmap/costmap|1.0|6.5|optional|Nav2 local costmap publication"
)

sanitize_topic() {
    local topic="$1"
    topic="${topic#/}"
    topic="${topic//\//__}"
    printf '%s' "${topic}"
}

extract_average_rate() {
    local file="$1"
    awk '/average rate:/ {rate=$3} END {if (rate != "") print rate}' "${file}" 2>/dev/null
}

compare_rate() {
    local rate="$1"
    local min_hz="$2"
    local max_hz="$3"

    awk -v r="${rate}" -v lo="${min_hz}" -v hi="${max_hz}" 'BEGIN {
        if (r < lo) print "LOW";
        else if (r > hi) print "HIGH";
        else print "PASS";
    }'
}

printf 'Charlie ROS 2 Topic Frequency Report\n' > "${SUMMARY_TXT}"
printf 'Generated: %s\n' "$(date --iso-8601=seconds)" >> "${SUMMARY_TXT}"
printf 'Sample duration: %s seconds per topic\n' "${SAMPLE_SECONDS}" >> "${SUMMARY_TXT}"
printf 'Visible ROS topics: %s\n' "${visible_topic_count}" >> "${SUMMARY_TXT}"
printf 'Unicast profile: %s\n' "${USE_UNICAST}" >> "${SUMMARY_TXT}"
printf 'Output directory: %s\n\n' "${OUTPUT_ROOT}" >> "${SUMMARY_TXT}"
printf '%-34s %-10s %-17s %-10s %s\n' "TOPIC" "OBSERVED" "EXPECTED" "STATUS" "NOTES" >> "${SUMMARY_TXT}"
printf '%-34s %-10s %-17s %-10s %s\n' "----------------------------------" "----------" "-----------------" "----------" "------------------------------" >> "${SUMMARY_TXT}"

printf 'topic,observed_hz,min_hz,max_hz,required,status,description,raw_file\n' > "${SUMMARY_CSV}"

pids=()
metadata_files=()

for spec in "${TOPIC_SPECS[@]}"; do
    IFS='|' read -r topic min_hz max_hz required description <<< "${spec}"
    safe_name="$(sanitize_topic "${topic}")"
    raw_file="${RAW_DIR}/${safe_name}.txt"
    metadata_file="${RAW_DIR}/${safe_name}.meta"

    printf '%s|%s|%s|%s|%s|%s\n' \
        "${topic}" "${min_hz}" "${max_hz}" "${required}" "${description}" "${raw_file}" \
        > "${metadata_file}"

    if ! grep -Fxq "${topic}" "${VISIBLE_TOPICS_FILE}"; then
        printf 'Topic is not present in ros2 topic list.\n' > "${raw_file}"
        metadata_files+=("${metadata_file}")
        continue
    fi

    echo "Sampling ${topic} for ${SAMPLE_SECONDS}s..."
    (
        # ros2 topic hz is Python-based. Force unbuffered output so its periodic
        # averages reach the file before timeout sends SIGINT.
        PYTHONUNBUFFERED=1 timeout --signal=INT --kill-after=3 "${SAMPLE_SECONDS}" \
            ros2 topic hz --window 100 "${topic}" > "${raw_file}" 2>&1
        exit 0
    ) &

    pids+=("$!")
    metadata_files+=("${metadata_file}")
done

for pid in "${pids[@]}"; do
    wait "${pid}" || true
done

overall_exit=0
pass_count=0
warning_count=0
fail_count=0
inactive_count=0

for metadata_file in "${metadata_files[@]}"; do
    IFS='|' read -r topic min_hz max_hz required description raw_file < "${metadata_file}"
    rate="$(extract_average_rate "${raw_file}")"

    if [[ -z "${rate}" ]]; then
        if [[ "${required}" == "required" ]]; then
            status="FAIL"
            observed="--"
            ((fail_count += 1))
            overall_exit=1
        else
            status="INACTIVE"
            observed="--"
            ((inactive_count += 1))
        fi
    else
        comparison="$(compare_rate "${rate}" "${min_hz}" "${max_hz}")"
        observed="$(printf '%.2f' "${rate}")"

        case "${comparison}" in
            PASS)
                status="PASS"
                ((pass_count += 1))
                ;;
            LOW|HIGH)
                if [[ "${required}" == "required" ]]; then
                    status="FAIL-${comparison}"
                    ((fail_count += 1))
                    overall_exit=1
                else
                    status="WARN-${comparison}"
                    ((warning_count += 1))
                fi
                ;;
        esac
    fi

    expected="${min_hz}-${max_hz} Hz"
    printf '%-34s %-10s %-17s %-10s %s\n' \
        "${topic}" "${observed}" "${expected}" "${status}" "${description}" \
        >> "${SUMMARY_TXT}"

    csv_description="${description//\"/\"\"}"
    printf '"%s","%s","%s","%s","%s","%s","%s","%s"\n' \
        "${topic}" "${observed}" "${min_hz}" "${max_hz}" "${required}" \
        "${status}" "${csv_description}" "${raw_file}" \
        >> "${SUMMARY_CSV}"
done

{
    printf '\nTotals: %d pass, %d warning, %d fail, %d inactive optional\n' \
        "${pass_count}" "${warning_count}" "${fail_count}" "${inactive_count}"
    printf '\nInterpretation:\n'
    printf '  PASS       observed rate is inside the expected range\n'
    printf '  FAIL-LOW   required topic is publishing too slowly\n'
    printf '  FAIL-HIGH  required topic is publishing unexpectedly fast\n'
    printf '  FAIL       required topic produced no measurable messages\n'
    printf '  WARN-*     optional topic is active but outside its expected range\n'
    printf '  INACTIVE   optional topic was not publishing in the current mode\n'
    printf '\nVisible topic list:\n  %s\n' "${VISIBLE_TOPICS_FILE}"
    printf 'ROS environment snapshot:\n  %s\n' "${ENV_FILE}"
    printf 'Raw ros2 topic hz output:\n  %s\n' "${RAW_DIR}"
} >> "${SUMMARY_TXT}"

cat "${SUMMARY_TXT}"
printf '\nCSV summary: %s\n' "${SUMMARY_CSV}"

exit "${overall_exit}"
