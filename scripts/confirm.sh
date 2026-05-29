#!/usr/bin/env bash
# Quick non-hanging health check for the current stack.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

echo "=== Nodes ==="
NODES="$(ros2 node list 2>/dev/null || true)"
echo "${NODES}" | grep -E "birds_node|obstacles_node|perception_node|coordinator_node|visualization_node|mavros" || true

DUP=0
for n in birds_node obstacles_node perception_node centralized_coordinator_node coordinator_node; do
  C="$(echo "${NODES}" | grep -c "/${n}$" || true)"
  if [[ "${C}" -gt 1 ]]; then
    echo "ERROR: ${C} instances of /${n} — run ./scripts/stop_app.sh then launch once" >&2
    DUP=1
  fi
done
if [[ "${DUP}" -ne 0 ]]; then
  exit 2
fi

echo ""
echo "=== Topics ==="
ros2 topic list | grep -E "^/birds/raw$|^/birds/positions$|^/obstacles/static$|^/obstacles/positions$|^/bird_markers$|^/obstacle_markers$|^/drone_marker$|^/bird/chased$|^/coordinator/target_index$|^/mavros/state$|^/mavros/local_position/pose$|^/mavros/setpoint_position/local$"

echo ""
echo "=== MAVROS state (timeout 3s) ==="
timeout 3 ros2 topic echo /mavros/state --once || echo "NO_MESSAGE_ON_/mavros/state"

echo ""
echo "=== Bird positions (timeout 3s) ==="
timeout 3 ros2 topic echo /birds/positions --once || echo "NO_MESSAGE_ON_/birds/positions"

echo ""
echo "=== Setpoint rate (5s sample) ==="
timeout 5 ros2 topic hz /mavros/setpoint_position/local --window 10 || echo "NO_RATE_ON_/mavros/setpoint_position/local"

echo ""
echo "Done."
