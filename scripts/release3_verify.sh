#!/usr/bin/env bash
# Obstacle + perception topic checks (run after ./scripts/launch.sh).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

EXPECTED_OBS="${1:-3}"
FAIL=0

pass() { echo "PASS: $*"; }
fail() { echo "FAIL: $*" >&2; FAIL=1; }

echo "=== Verify obstacles (expected count: ${EXPECTED_OBS}) ==="

echo ""
echo "--- Nodes ---"
NODE_LIST="$(ros2 node list 2>/dev/null || true)"
for n in obstacles_node perception_node centralized_coordinator_node coordinator_node; do
  if echo "${NODE_LIST}" | grep -q "/${n}$"; then
    pass "${n} running"
  else
    fail "${n} not found (start ./scripts/launch.sh first)"
  fi
  C="$(echo "${NODE_LIST}" | grep -c "/${n}$" || true)"
  if [[ "${C}" -gt 1 ]]; then
    fail "${C} instances of /${n} (run ./scripts/stop_app.sh, launch once)"
  fi
done

echo ""
echo "--- Obstacle topics ---"
OBS_SAMPLE="$(timeout 5 ros2 topic echo /obstacles/positions --once 2>/dev/null || true)"
if [[ -z "${OBS_SAMPLE}" ]]; then
  fail "no message on /obstacles/positions"
else
  COUNT="$(echo "${OBS_SAMPLE}" | grep -c '^- position:' || true)"
  if [[ "${COUNT}" -ge "${EXPECTED_OBS}" ]]; then
    pass "/obstacles/positions has ${COUNT} obstacle(s)"
  else
    fail "/obstacles/positions has ${COUNT} obstacle(s), expected >= ${EXPECTED_OBS}"
  fi
fi

if timeout 3 ros2 topic echo /obstacles/static --once >/dev/null 2>&1; then
  pass "/obstacles/static publishing"
else
  fail "no sample on /obstacles/static"
fi

echo ""
echo "--- Release 2 regression (birds + setpoints) ---"
if "${SCRIPT_DIR}/release2_verify.sh" 3; then
  pass "release2_verify.sh passed"
else
  fail "release2_verify.sh failed"
fi

echo ""
if [[ "${FAIL}" -eq 0 ]]; then
  echo "Release 3 verify: ALL PASSED"
  exit 0
fi
echo "Release 3 verify: SOME CHECKS FAILED"
exit 1
