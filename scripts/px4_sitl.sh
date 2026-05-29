#!/usr/bin/env bash
# Start PX4 SITL with the known-good lab flow.
# Override defaults if needed:
#   HEADLESS=0 PX4_TARGET=gazebo-classic ./scripts/px4_sitl.sh
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_TARGET="${PX4_TARGET:-gazebo-classic}"
HEADLESS="${HEADLESS:-1}"

cd "${PX4_DIR}"
make clean "HEADLESS=${HEADLESS}"
exec make px4_sitl "${PX4_TARGET}"
