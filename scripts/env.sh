# shellcheck shell=bash
# ROS 2 + workspace overlay for this repo. Must be *sourced* so your shell keeps PATH/AMENT_PREFIX_PATH.
#
#   source /path/to/2026-04-15ROBO4850/scripts/env.sh
#
# After sourcing, DRONE_WS points at the workspace root. Other scripts in scripts/ call this automatically.

if [[ -z "${BASH_VERSION:-}" ]]; then
  echo "env.sh: use bash and: source path/to/scripts/env.sh" >&2
  return 2 2>/dev/null || exit 2
fi

# Refuse direct execution (sourcing in a subshell would not affect your terminal).
if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "env.sh: source this file, do not run it:" >&2
  echo "  source \"${BASH_SOURCE[0]}\"" >&2
  exit 1
fi

_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DRONE_WS="$(cd "${_SCRIPT_DIR}/.." && pwd)"
# Canonical Unity demo project in-repo (override with UNITY_PROJECT=...).
export UNITY_PROJECT="${UNITY_PROJECT:-${DRONE_WS}/unity/FieldDemo}"
export UNITY_SOURCE="${UNITY_SOURCE:-${UNITY_PROJECT}}"
export UNITY_TARGET="${UNITY_TARGET:-${HOME}/UnityProjects/FieldDemo}"

# ROS / colcon setup scripts reference optional env vars (e.g. AMENT_TRACE_SETUP_FILES).
# If the caller used `set -u`, sourcing would fail. Temporarily allow unset vars.
_DRONE_ENV_PREV_NOUNSET=0
case $- in *u*) _DRONE_ENV_PREV_NOUNSET=1 ;; esac
set +u

# shellcheck source=/dev/null
source /opt/ros/humble/setup.bash

# Optional: classic Gazebo paths used by some ROS-Gazebo integrations.
if [[ -f /usr/share/gazebo/setup.sh ]]; then
  # shellcheck source=/dev/null
  source /usr/share/gazebo/setup.sh
fi

if [[ -f "${DRONE_WS}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${DRONE_WS}/install/setup.bash"
else
  echo "env.sh: no install/setup.bash yet. Run scripts/build.sh first." >&2
fi

if ((_DRONE_ENV_PREV_NOUNSET)); then
  set -u
fi
unset _DRONE_ENV_PREV_NOUNSET
