#!/usr/bin/env bash
set -euo pipefail

port="${META_QUEST_PORT:-8000}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

restore_headset_sleep() {
  if [[ "$(adb get-state 2>/dev/null || true)" == "device" ]]; then
    adb shell am broadcast \
      -a com.oculus.vrpowermanager.automation_disable >/dev/null 2>&1 || true
    echo "Restored normal Quest proximity sleep behavior."
  fi
}

trap restore_headset_sleep EXIT

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cd -- "${repository_root}"
  exec python -m data_collection.meta_quest.raw_recorder "$@"
fi

command -v adb >/dev/null 2>&1 || {
  echo "Error: adb is not installed or is not on PATH." >&2
  exit 1
}

if [[ "$(adb get-state 2>/dev/null || true)" != "device" ]]; then
  echo "Error: no authorized Meta Quest is available through adb." >&2
  echo "Run 'adb devices -l' and approve USB debugging in the headset." >&2
  exit 1
fi

adb reverse "tcp:${port}" "tcp:${port}" >/dev/null
adb shell am broadcast \
  -a com.oculus.vrpowermanager.automation_disable >/dev/null

echo "ADB reverse mapping: Quest tcp:${port} -> computer tcp:${port}"
echo "Recording validated raw Quest poses only; no robot output is started."

cd -- "${repository_root}"
python -m data_collection.meta_quest.raw_recorder --port "${port}" "$@"
