#!/usr/bin/env bash
set -uo pipefail

# Measure important Charlie ROS 2 topic rates, save raw output, and compare the
# observed averages against expected ranges.
#
# Usage:
#   bash scripts/check_ros2_topic_rates.sh
#   bash scripts/check_ros2_topic_rates.sh 20
#
# The optional argument is the sampling duration in seconds per topic.

REPO_ROOT="${HOME}/Slambot_Charlie"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SAMPLE_SECONDS="${1:-12}"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUTPUT_ROOT="${REPO_ROOT}/runtime/diagnostics/topic_rates_${STAMP}"
SUMMARY_TXT="${OUTPUT_ROOT}/summary.txt"
SUMMARY_CSV="${OUTPUT_ROOT}/summary.csv"
RAW_DIR="${OUTPUT_ROOT}/raw"

if ! [[ "${SAMPLE_SECONDS}" =~ ^[0-9]+$ ]] || (( SAMPLE_SECONDS < 3 )); then
    echo "Sampling duration must be an integer of at least 3 seconds." >&2
    exit 2
fi

# ROS environment setup scripts are not guaranteed to be safe under `set -u`.
# Temporarily disable nounset while sourcing them, then restore it immediately.
set +u

if [[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
else
    set -u
    echo "ROS setup not found: /opt/ros/${ROS_DISTRO}/setup.bash" >&2
    exit 1
fi

if [[ -f "${REPO_ROOT}/ros2_ws/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${REPO_ROOT}/ros2_ws/install/setup.bash"
else
    set -u
    echo "Workspace setup not found: ${REPO_ROOT}/ros2_ws/install/setup.bash" >&2
    echo "Build and source the workspace before running this script." >&2
    exit 1
fi

set -u

mkdir -p "${RAW_DIR}"

# Format:
#   topic|min_hz|max_hz|required|description
#
# Required topics should normally exist in Charlie's standard bringup.
# Optional topics depend on mapping, EKF, or Nav2 mode. An inactive optional
# topic is reported as INACTIVE rather than FAIL.
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

    echo "Sampling ${topic} for ${SAMPLE_SECONDS}s..."
    (
        timeout --signal=INT "${SAMPLE_SECONDS}" \
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
    printf '\nRaw ros2 topic hz output is stored under:\n  %s\n' "${RAW_DIR}"
} >> "${SUMMARY_TXT}"

cat "${SUMMARY_TXT}"
printf '\nCSV summary: %s\n' "${SUMMARY_CSV}"

exit "${overall_exit}"
