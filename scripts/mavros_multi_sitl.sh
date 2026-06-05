#!/usr/bin/env bash
# Start two MAVROS 2 nodes for multi-vehicle PX4 SITL.
#
# Defaults map:
#   drone_1 -> PX4 instance 1, MAV_SYS_ID 2, udp://:14541@127.0.0.1:14581
#   drone_2 -> PX4 instance 2, MAV_SYS_ID 3, udp://:14542@127.0.0.1:14582
#
# Override anytime:
#   FCU_URL_1='udp://:14541@127.0.0.1:14581' \
#   FCU_URL_2='udp://:14542@127.0.0.1:14582' \
#   # or pass comma-separated fallbacks:
#   FCU_URL_CANDIDATES_1='udp://:14541@127.0.0.1:14581,udp://:14540@127.0.0.1:14580' \
#   FCU_URL_CANDIDATES_2='udp://:14542@127.0.0.1:14582,udp://:14541@127.0.0.1:14581' \
#   TGT_SYSTEM_1=2 \
#   TGT_SYSTEM_2=3 \
#   TGT_COMPONENT_1=1 \
#   TGT_COMPONENT_2=1 \
#   ./scripts/mavros_multi_sitl.sh
set -eo pipefail

source /opt/ros/humble/setup.bash

# Explicit FCU_URL_{1,2} wins. Otherwise we probe candidates in order.
# PX4 gazebo-classic sitl_multiple_run.sh starts instances at 1, so the first
# two vehicles are MAV_SYS_ID 2 and 3 with offboard remotes 14541 and 14542.
FCU_URL_CANDIDATES_1="${FCU_URL_CANDIDATES_1:-udp://:14541@127.0.0.1:14581,udp://:14540@127.0.0.1:14580}"
FCU_URL_CANDIDATES_2="${FCU_URL_CANDIDATES_2:-udp://:14542@127.0.0.1:14582,udp://:14541@127.0.0.1:14581}"
if [[ -n "${FCU_URL_1:-}" ]]; then
  FCU_URL_CANDIDATES_1="${FCU_URL_1}"
fi
if [[ -n "${FCU_URL_2:-}" ]]; then
  FCU_URL_CANDIDATES_2="${FCU_URL_2}"
fi
TGT_SYSTEM_1="${TGT_SYSTEM_1:-2}"
TGT_SYSTEM_2="${TGT_SYSTEM_2:-3}"
TGT_COMPONENT_1="${TGT_COMPONENT_1:-1}"
TGT_COMPONENT_2="${TGT_COMPONENT_2:-1}"
CONNECT_TIMEOUT_S="${MAVROS_CONNECT_TIMEOUT_S:-10}"
MAX_RESTARTS="${MAVROS_MAX_RESTARTS:-0}"
ACTIVE_FCU_URL_1=""
ACTIVE_FCU_URL_2=""

PID_1=""
PID_2=""

cleanup_stale() {
  # This launcher owns the MAVROS bridge for the demo; clear old bridge nodes first
  # so stale processes do not keep publishing disconnected /drone_X/state topics.
  pkill -f '[m]avros_node' 2>/dev/null || true
  pkill -f '[m]avros_router' 2>/dev/null || true
  pkill -f '[m]avros_node.*__ns:=/drone_1' 2>/dev/null || true
  pkill -f '[m]avros_node.*__ns:=/drone_2' 2>/dev/null || true
  pkill -f '[m]avros_router.*__ns:=/drone_1' 2>/dev/null || true
  pkill -f '[m]avros_router.*__ns:=/drone_2' 2>/dev/null || true
  pkill -f '[m]avros.*[/]drone_1' 2>/dev/null || true
  pkill -f '[m]avros.*[/]drone_2' 2>/dev/null || true
  pkill -f '[m]avros.*[/]uas1' 2>/dev/null || true
  pkill -f '[m]avros.*[/]uas2' 2>/dev/null || true
}

start_mavros() {
  local ns="$1"
  local fcu_url="$2"
  local tgt_system="$3"
  local tgt_component="$4"
  ros2 run mavros mavros_node --ros-args \
    -r "__ns:=/${ns}" \
    -p "fcu_url:=${fcu_url}" \
    -p "tgt_system:=${tgt_system}" \
    -p "tgt_component:=${tgt_component}" \
    1>&2 &
  echo $!
}

kill_pid() {
  local pid="${1:-}"
  if [[ -n "${pid}" ]]; then
    kill "${pid}" 2>/dev/null || true
    wait "${pid}" 2>/dev/null || true
  fi
}

topic_connected() {
  local topic="$1"
  local out
  out="$(timeout 3 ros2 topic echo "${topic}" --once 2>/dev/null || true)"
  [[ "${out}" == *"connected: true"* ]]
}

topic_connected_any() {
  local ns="$1"
  # MAVROS topic naming differs across setups:
  #   /<ns>/state          or /<ns>/mavros/state
  topic_connected "/${ns}/state" && return 0
  topic_connected "/${ns}/mavros/state" && return 0
  return 1
}

ensure_connected() {
  local name="$1"
  local state_topic_label="$2"
  local fcu_url_candidates_csv="$3"
  local tgt_system="$4"
  local tgt_component="$5"
  local -a fcu_urls=()
  IFS=',' read -r -a fcu_urls <<< "${fcu_url_candidates_csv}"
  local fcu_url=""
  for fcu_url in "${fcu_urls[@]}"; do
    local attempt=0
    local pid=""
    while (( attempt <= MAX_RESTARTS )); do
      pid="$(start_mavros "${name}" "${fcu_url}" "${tgt_system}" "${tgt_component}")"
      echo "mavros_multi_sitl: waiting for ${name} on ${state_topic_label} via ${fcu_url} (attempt $((attempt + 1))/${MAX_RESTARTS}+1)" >&2
      local deadline=$((SECONDS + CONNECT_TIMEOUT_S))
      while (( SECONDS < deadline )); do
        if topic_connected_any "${name}"; then
          echo "mavros_multi_sitl: ${name} connected via ${fcu_url}" >&2
          printf '%s|%s\n' "${pid}" "${fcu_url}"
          return 0
        fi
        sleep 1
      done
      echo "mavros_multi_sitl: ${name} did not connect via ${fcu_url} within ${CONNECT_TIMEOUT_S}s, restarting" >&2
      kill_pid "${pid}"
      attempt=$((attempt + 1))
    done
  done
  return 1
}

cleanup_stale
sleep 1

RESULT_FILE_1="$(mktemp)"
RESULT_FILE_2="$(mktemp)"
ensure_connected drone_1 /drone_1/state "${FCU_URL_CANDIDATES_1}" "${TGT_SYSTEM_1}" "${TGT_COMPONENT_1}" > "${RESULT_FILE_1}" &
CONNECT_JOB_1="$!"
ensure_connected drone_2 /drone_2/state "${FCU_URL_CANDIDATES_2}" "${TGT_SYSTEM_2}" "${TGT_COMPONENT_2}" > "${RESULT_FILE_2}" &
CONNECT_JOB_2="$!"

STATUS_1=0
STATUS_2=0
wait "${CONNECT_JOB_1}" || STATUS_1="$?"
wait "${CONNECT_JOB_2}" || STATUS_2="$?"

if [[ "${STATUS_1}" -ne 0 || "${STATUS_2}" -ne 0 ]]; then
  if [[ -s "${RESULT_FILE_1}" ]]; then
    RESULT_1="$(cat "${RESULT_FILE_1}")"
    kill_pid "${RESULT_1%%|*}"
  fi
  if [[ -s "${RESULT_FILE_2}" ]]; then
    RESULT_2="$(cat "${RESULT_FILE_2}")"
    kill_pid "${RESULT_2%%|*}"
  fi
  if [[ "${STATUS_1}" -ne 0 ]]; then
    echo "mavros_multi_sitl: failed to connect drone_1 using candidates: ${FCU_URL_CANDIDATES_1}" >&2
  fi
  if [[ "${STATUS_2}" -ne 0 ]]; then
    echo "mavros_multi_sitl: failed to connect drone_2 using candidates: ${FCU_URL_CANDIDATES_2}" >&2
  fi
  echo "mavros_multi_sitl: UDP sockets for PX4/MAVROS diagnostics:" >&2
  ss -lunp 2>/dev/null | grep -E 'px4|mavros|145[0-9][0-9]|1857[0-9]|1428[0-9]' >&2 || true
  rm -f "${RESULT_FILE_1}" "${RESULT_FILE_2}"
  exit 1
fi

RESULT_1="$(cat "${RESULT_FILE_1}")"
RESULT_2="$(cat "${RESULT_FILE_2}")"
rm -f "${RESULT_FILE_1}" "${RESULT_FILE_2}"

PID_1="${RESULT_1%%|*}"
ACTIVE_FCU_URL_1="${RESULT_1#*|}"
PID_2="${RESULT_2%%|*}"
ACTIVE_FCU_URL_2="${RESULT_2#*|}"
echo "mavros_multi_sitl: active drone_1 FCU URL ${ACTIVE_FCU_URL_1}" >&2
echo "mavros_multi_sitl: active drone_2 FCU URL ${ACTIVE_FCU_URL_2}" >&2

cleanup() {
  kill_pid "${PID_1}"
  kill_pid "${PID_2}"
}
trap cleanup INT TERM

while kill -0 "${PID_1}" 2>/dev/null && kill -0 "${PID_2}" 2>/dev/null; do
  sleep 1
done

echo "mavros_multi_sitl: one MAVROS process exited, stopping bridge" >&2
cleanup
exit 1
