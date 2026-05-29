#!/usr/bin/env bash
# Application nodes only. Start PX4 SITL + Gazebo + MAVROS in other terminals first.
#
# Examples:
#   ./scripts/launch.sh
#   ./scripts/launch.sh bird_count:=5
#   USE_SIM_TIME=true ./scripts/launch.sh        # when /clock is confirmed alive
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

# Prevent duplicate birds/coordinator stacks (causes 6 birds, fighting setpoints).
"${SCRIPT_DIR}/stop_app.sh"

cd "${DRONE_WS}"
# Default to wall time to avoid silent timer stalls when /clock is missing.
# Override with USE_SIM_TIME=true when PX4/Gazebo /clock is confirmed active.
USE_SIM_TIME="${USE_SIM_TIME:-true}"
exec ros2 launch drone_system_pkg system.launch.py "use_sim_time:=${USE_SIM_TIME}" "$@"
