#!/usr/bin/env bash
# Multi-bird topics, chased flag, setpoint stream, target index (run after launch.sh).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

EXPECTED_BIRDS="${1:-3}"
FAIL=0

pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*" >&2; FAIL=1; }

echo "=== Verify birds (expected count: ${EXPECTED_BIRDS}) ==="

echo ""
echo "--- Nodes ---"
NODE_LIST="$(ros2 node list 2>/dev/null || true)"
if echo "${NODE_LIST}" | grep -q coordinator_node; then
  pass "coordinator_node running"
else
  fail "coordinator_node not found (start ./scripts/launch.sh first)"
fi
for n in birds_node perception_node centralized_coordinator_node coordinator_node; do
  C="$(echo "${NODE_LIST}" | grep -c "/${n}$" || true)"
  if [[ "${C}" -gt 1 ]]; then
    fail "${C} instances of /${n} (run ./scripts/stop_app.sh, then launch once)"
  fi
done

echo ""
echo "--- Bird count on /birds/positions ---"
BIRD_SAMPLE="$(timeout 5 ros2 topic echo /birds/positions --once 2>/dev/null || true)"
if [[ -z "${BIRD_SAMPLE}" ]]; then
  fail "no message on /birds/positions"
else
  COUNT="$(echo "${BIRD_SAMPLE}" | grep -c '^- position:' || true)"
  if [[ "${COUNT}" -ge "${EXPECTED_BIRDS}" ]]; then
    pass "/birds/positions has ${COUNT} pose(s)"
  else
    fail "/birds/positions has ${COUNT} pose(s), expected >= ${EXPECTED_BIRDS}"
  fi
fi

echo ""
echo "--- Chase / target selection ---"
CHASED_OK=0
if timeout 12 ros2 topic echo /bird/chased 2>/dev/null | grep -m1 -q 'data: true'; then
  CHASED_OK=1
fi
TARGET_SAMPLE="$(timeout 3 ros2 topic echo /coordinator/target_index --once 2>/dev/null || true)"
if [[ "${CHASED_OK}" -eq 1 ]]; then
  pass "/bird/chased became true (drone chasing)"
elif echo "${TARGET_SAMPLE}" | grep -qE 'data: [0-9]+'; then
  pass "target index publishing (chased may be false if birds outside limit_xy)"
else
  fail "no chase flag and no /coordinator/target_index"
fi

echo ""
echo "--- Setpoint rate ---"
HZ_OUT="$(timeout 6 ros2 topic hz /mavros/setpoint_position/local --window 10 2>&1 || true)"
if echo "${HZ_OUT}" | grep -qE 'average rate: [1-9][0-9]'; then
  pass "setpoint stream active (~10+ Hz)"
elif echo "${HZ_OUT}" | grep -q 'average rate:'; then
  fail "setpoint rate low: ${HZ_OUT}"
else
  fail "no setpoint rate on /mavros/setpoint_position/local"
fi

echo ""
if [[ "${FAIL}" -eq 0 ]]; then
  echo "Release 2 verify: ALL PASSED"
  exit 0
fi
echo "Release 2 verify: SOME CHECKS FAILED"
exit 1
