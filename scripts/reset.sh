#!/usr/bin/env bash
# Hard-stop common sim/ROS processes on a stuck lab session. Kills *all* matching ros2/gazebo. Use carefully.
set +e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
"${SCRIPT_DIR}/stop_app.sh" 2>/dev/null || true

echo "Stopping ros2 / gazebo / gz ..."
pkill -9 -f '[r]os2' 2>/dev/null || true
pkill -9 -f '[m]avros' 2>/dev/null || true
pkill -9 -f '[p]x4' 2>/dev/null || true
pkill -9 -f '[g]azebo' 2>/dev/null || true
pkill -9 -f '[g]zserver' 2>/dev/null || true
pkill -9 -f '[g]zclient' 2>/dev/null || true

echo "Stopping ros2 daemon (if any) ..."
ros2 daemon stop 2>/dev/null || true

echo "Done. Open a new terminal and: source path/to/scripts/env.sh"
