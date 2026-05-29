#!/usr/bin/env bash
# Build only this package. Safe to re-run; does not wipe install/.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

cd "${DRONE_WS}"
echo "Building drone_system_pkg in ${DRONE_WS} ..."
colcon build --packages-select drone_system_pkg

# Same nounset issue as env.sh when sourcing overlay after colcon.
_DRONE_ENV_PREV_NOUNSET=0
case $- in *u*) _DRONE_ENV_PREV_NOUNSET=1 ;; esac
set +u
# shellcheck source=/dev/null
source "${DRONE_WS}/install/setup.bash"
if ((_DRONE_ENV_PREV_NOUNSET)); then
  set -u
fi
unset _DRONE_ENV_PREV_NOUNSET

echo "Done. Launch with: ${SCRIPT_DIR}/launch.sh"
