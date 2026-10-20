#!/usr/bin/env bash
set -euo pipefail

port="${META_QUEST_PORT:-8000}"
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

echo "Listening for the local ALVR/SteamVR bridge on 127.0.0.1:${port}."
echo "No ADB mapping, ROS node, or robot output is started."

cd -- "${repository_root}"
exec python -m data_collection.meta_quest.monitor \
  --host 127.0.0.1 --port "${port}" --print-hz 1 "$@"
