#!/usr/bin/env bash
# Open FieldDemo in Unity; syncs scripts/layout into ~/UnityProjects/FieldDemo first.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "${SCRIPT_DIR}/env.sh"

"${SCRIPT_DIR}/sync_unity.sh"

UNITY_EDITOR="${UNITY_EDITOR:-${HOME}/Unity/Hub/Editor/2022.3.62f3/Editor/Unity}"
if [[ ! -x "${UNITY_EDITOR}" ]]; then
  echo "launch_unity: set UNITY_EDITOR to your Unity binary (not found at ${UNITY_EDITOR})" >&2
  exit 1
fi

echo "launch_unity: opening ${UNITY_PROJECT}"
exec "${UNITY_EDITOR}" -projectPath "${UNITY_PROJECT}"
