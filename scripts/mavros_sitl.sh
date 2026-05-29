#!/usr/bin/env bash
# Start MAVROS 2 against PX4 SITL with a lab-friendly default FCU URL.
#
# On this lab machine, the reliable endpoint was:
#   ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14580
#
# Override anytime:
#   FCU_URL='udp://:14540@127.0.0.1:14580' ./scripts/mavros_sitl.sh
#
set -eo pipefail

source /opt/ros/humble/setup.bash

# Default is set to the working lab endpoint. Override with FCU_URL if needed.
FCU_URL="${FCU_URL:-udp://:14540@127.0.0.1:14580}"

exec ros2 run mavros mavros_node --ros-args -p "fcu_url:=${FCU_URL}"
