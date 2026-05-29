#!/usr/bin/env bash
# Verify birds_node → perception_node → coordinator_node bird position updates.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "PASS: $*"; }

echo "=== Nodes (birds / perception / coordinator) ==="
NODES="$(ros2 node list 2>/dev/null || true)"
for n in birds_node perception_node centralized_coordinator_node coordinator_node; do
  if echo "${NODES}" | grep -q "/${n}"; then
    pass "${n} running"
  else
    fail "${n} not running — start T4: USE_SIM_TIME=true ./scripts/launch.sh"
  fi
done

echo ""
echo "=== /birds/raw rate ==="
RAW_HZ="$(timeout 5 ros2 topic hz /birds/raw --window 5 2>&1 || true)"
echo "${RAW_HZ}"
echo "${RAW_HZ}" | grep -q 'average rate' || fail "no publisher on /birds/raw"

echo ""
echo "=== /birds/positions rate + coordinator subscription ==="
POS_HZ="$(timeout 5 ros2 topic hz /birds/positions --window 5 2>&1 || true)"
echo "${POS_HZ}"
echo "${POS_HZ}" | grep -q 'average rate' || fail "no publisher on /birds/positions (is perception_node up?)"

INFO="$(ros2 topic info /birds/positions -v 2>&1 || true)"
echo "${INFO}" | grep -q 'coordinator_node' || fail "coordinator_node not subscribed to /birds/positions"

echo ""
echo "=== Position updates (two samples, ~1.5s apart) ==="
read_bird0_x() {
  timeout 3 ros2 topic echo /birds/positions --once 2>/dev/null \
    | awk '/^- position:/{p=1; next} p&&/x:/{print $2; exit}'
}
X1="$(read_bird0_x)" || fail "could not read /birds/positions"
sleep 1.5
X2="$(read_bird0_x)" || fail "could not read second /birds/positions sample"
echo "  bird[0].x sample1: ${X1}"
echo "  bird[0].x sample2: ${X2}"
if [[ "${X1}" != "${X2}" ]]; then
  pass "bird positions are updating"
else
  fail "bird[0].x unchanged — birds_node may be stalled (check use_sim_time / /clock)"
fi

echo ""
echo "=== Coordinator outputs ==="
TARGET="$(timeout 3 ros2 topic echo /coordinator/target_index --once 2>/dev/null || true)"
CHASED="$(timeout 3 ros2 topic echo /bird/chased --once 2>/dev/null || true)"
echo "${TARGET:-NO /coordinator/target_index}"
echo "${CHASED:-NO /bird/chased}"
echo "${TARGET}" | grep -q 'data:' || fail "no target_index from coordinator"
if echo "${CHASED}" | grep -q 'data: true'; then
  pass "coordinator sees birds (bird_seen → /bird/chased true)"
else
  echo "WARN: /bird/chased is false (no birds in last callback, or battery return mode)"
fi

echo ""
echo "=== Frame IDs (should match for chase setpoints) ==="
BFRAME="$(timeout 2 ros2 topic echo /birds/positions --once 2>/dev/null | awk '/frame_id:/{print $2; exit}' || true)"
PFRAME="$(timeout 2 ros2 topic echo /mavros/local_position/pose --once 2>/dev/null | awk '/frame_id:/{print $2; exit}' || echo 'N/A')"
echo "  birds: ${BFRAME:-unknown}"
echo "  pose:  ${PFRAME}"
if [[ -n "${BFRAME}" && -n "${PFRAME}" && "${BFRAME}" != "${PFRAME}" && "${PFRAME}" != "N/A" ]]; then
  echo "WARN: frame mismatch — coordinator copies pose frame onto setpoints"
fi

echo ""
echo "Done. Full chase check: ./scripts/diagnose_chase.sh"
