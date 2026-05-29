#!/usr/bin/env bash
# Copy tracked Unity demo assets from this repo into the project you open in the Editor.
#
# Default source:  $DRONE_WS/unity/FieldDemo
# Default target:  ~/UnityProjects/FieldDemo
#
# Usage:
#   ./scripts/sync_unity.sh
#   UNITY_TARGET=/path/to/FieldDemo ./scripts/sync_unity.sh
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

SRC="${UNITY_SOURCE:-${DRONE_WS}/unity/FieldDemo}"
TARGET="${UNITY_TARGET:-${HOME}/UnityProjects/FieldDemo}"

if [[ ! -d "${SRC}/Assets" ]]; then
  echo "sync_unity: missing source ${SRC}/Assets" >&2
  exit 1
fi
mkdir -p "${TARGET}/Assets/Scripts" "${TARGET}/Assets/StreamingAssets"

# ROS layout is authoritative in config/ (or FIELD_LAYOUT_PATH); mirror into Unity StreamingAssets.
LAYOUT_SRC="${FIELD_LAYOUT_PATH:-${DRONE_WS}/config/field_layout.yaml}"
cp -a "${LAYOUT_SRC}" "${SRC}/Assets/StreamingAssets/field_layout.yaml"
cp -a "${LAYOUT_SRC}" "${TARGET}/Assets/StreamingAssets/field_layout.yaml"

for f in BirdsVisual.cs DroneVisual.cs FieldLayout.cs; do
  cp -a "${SRC}/Assets/Scripts/${f}" "${TARGET}/Assets/Scripts/${f}"
done

echo "sync_unity: ${SRC} -> ${TARGET}"
echo "  Scripts: BirdsVisual.cs DroneVisual.cs FieldLayout.cs"
echo "  StreamingAssets/field_layout.yaml (from ${LAYOUT_SRC})"
