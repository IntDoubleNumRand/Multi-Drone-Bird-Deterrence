#!/usr/bin/env bash
# Quick check that MAVROS topics exist (run while PX4 SITL + MAVROS are already up).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

echo "Checking MAVROS bridge topics..."
if ! ros2 topic list | grep -E 'mavros|local_position'; then
  echo "No mavros topics found. Start PX4 SITL + MAVROS first." >&2
  exit 1
fi

echo ""
echo "State sample (/mavros/state):"
STATE_SAMPLE="$(timeout 3 ros2 topic echo /mavros/state --once || true)"
echo "${STATE_SAMPLE}"
if ! echo "${STATE_SAMPLE}" | grep -q "connected: true"; then
  echo "MAVROS is not connected to FCU yet (connected=false)." >&2
  echo "Check FCU_URL and ensure PX4 SITL is running." >&2
  exit 2
fi

echo ""
echo "One-shot pose sample (/mavros/local_position/pose):"
if ! timeout 3 ros2 topic echo /mavros/local_position/pose --once; then
  echo "No pose sample from /mavros/local_position/pose." >&2
  exit 3
fi

echo ""
echo "Next: arm, takeoff, OFFBOARD, then ./scripts/launch.sh"
