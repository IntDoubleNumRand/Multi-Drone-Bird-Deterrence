#!/usr/bin/env bash
# Bird-focused demo entrypoint.
# Default layout now follows the primary field map so ranges/colors match normal launch.
# To use an alternate demo layout, set FIELD_LAYOUT_PATH explicitly.
#
#   ./scripts/launch_demo.sh
#   ./scripts/launch_demo.sh bird_count:=3
#   FIELD_LAYOUT_PATH=./config/field_layout_demo.yaml ./scripts/launch_demo.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

export FIELD_LAYOUT_PATH="${FIELD_LAYOUT_PATH:-${DRONE_WS}/config/field_layout.yaml}"
LAYOUT_SRC="${FIELD_LAYOUT_PATH}"

"${SCRIPT_DIR}/stop_app.sh"

# Keep Unity StreamingAssets aligned with the demo layout.
for dest in \
  "${DRONE_WS}/unity/FieldDemo/Assets/StreamingAssets/field_layout.yaml" \
  "${UNITY_TARGET}/Assets/StreamingAssets/field_layout.yaml"; do
  if [[ -d "$(dirname "${dest}")" ]]; then
    cp -a "${LAYOUT_SRC}" "${dest}"
  fi
done

cd "${DRONE_WS}"
USE_SIM_TIME="${USE_SIM_TIME:-true}"
echo "launch_demo: FIELD_LAYOUT_PATH=${FIELD_LAYOUT_PATH}"
exec ros2 launch drone_system_pkg demo.launch.py "use_sim_time:=${USE_SIM_TIME}" "$@"
