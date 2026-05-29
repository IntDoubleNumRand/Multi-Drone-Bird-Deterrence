#!/usr/bin/env bash
# Why is the drone not chasing? Quick read-only checks.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

echo "=== MAVROS state ==="
timeout 2 ros2 topic echo /mavros/state --once 2>/dev/null || echo "NO /mavros/state"

echo ""
echo "=== Pose frame + position ==="
timeout 2 ros2 topic echo /mavros/local_position/pose --once 2>/dev/null | head -20 || echo "NO pose"

echo ""
echo "=== Birds (count poses) ==="
timeout 2 ros2 topic echo /birds/positions --once 2>/dev/null | head -40 || echo "NO birds"

echo ""
echo "=== Chased flag ==="
timeout 2 ros2 topic echo /bird/chased --once 2>/dev/null || echo "NO /bird/chased"

echo ""
echo "=== Setpoint sample ==="
timeout 2 ros2 topic echo /mavros/setpoint_position/local --once 2>/dev/null | head -20 || echo "NO setpoint"

echo ""
echo "Expect: connected=true, mode=OFFBOARD, armed=true, birds>=1 pose, /bird/chased true when chasing."
echo "Pose frame_id should match setpoint frame_id (usually local_origin)."
