#!/usr/bin/env bash
# Start two PX4 gazebo-classic SITL vehicles.
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
VEHICLE_COUNT="${VEHICLE_COUNT:-2}"
HEADLESS="${HEADLESS:-1}"

cd "${PX4_DIR}"
echo "px4_multi_sitl: starting ${VEHICLE_COUNT} vehicle(s) from ${PX4_DIR}" >&2
echo "px4_multi_sitl: after startup, check MAVLink sockets with: ss -lunp | grep px4" >&2
exec env HEADLESS="${HEADLESS}" ./Tools/simulation/gazebo-classic/sitl_multiple_run.sh -n "${VEHICLE_COUNT}"
