#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

echo "Offline Quest target preview only; ROS and robot output are not started."
cd -- "${repository_root}"
python -m data_collection.meta_quest.target_preview "$@"
