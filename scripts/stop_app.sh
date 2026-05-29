#!/usr/bin/env bash
# Stop only drone_system_pkg app nodes (keep PX4 / MAVROS running).
set +e

echo "Stopping drone_system app nodes ..."
pkill -f '[d]rone_system_pkg.*birds_node' 2>/dev/null || true
pkill -f '[d]rone_system_pkg.*perception_node' 2>/dev/null || true
pkill -f '[d]rone_system_pkg.*coordinator_node' 2>/dev/null || true
pkill -f '[d]rone_system_pkg.*centralized_coordinator_node' 2>/dev/null || true
pkill -f '[d]rone_system_pkg.*visualization_node' 2>/dev/null || true
pkill -f '[d]rone_system_pkg.*obstacles_node' 2>/dev/null || true
# Fallback for ros2 run invocations without full path in cmdline.
pkill -f '[/]birds_node' 2>/dev/null || true
pkill -f '[/]perception_node' 2>/dev/null || true
pkill -f '[/]coordinator_node' 2>/dev/null || true
pkill -f '[/]centralized_coordinator_node' 2>/dev/null || true
pkill -f '[/]visualization_node' 2>/dev/null || true
pkill -f '[/]obstacles_node' 2>/dev/null || true
sleep 1
echo "Done."
