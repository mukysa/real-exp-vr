#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
port="${META_QUEST_PORT:-8000}"
recorder_pid=""
bridge_pid=""
recorder_python="${META_QUEST_PYTHON:-$(command -v python || true)}"

stop_children() {
  trap '' INT TERM
  trap - EXIT
  if [[ -n "${recorder_pid}" ]] && kill -0 "${recorder_pid}" 2>/dev/null; then
    kill -INT "${recorder_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bridge_pid}" ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
    kill -TERM "${bridge_pid}" 2>/dev/null || true
  fi
  [[ -z "${recorder_pid}" ]] || wait "${recorder_pid}" 2>/dev/null || true
  [[ -z "${bridge_pid}" ]] || wait "${bridge_pid}" 2>/dev/null || true
  recorder_pid=""
  bridge_pid=""
}

trap 'stop_children; exit 130' INT
trap 'stop_children; exit 143' TERM
trap stop_children EXIT

if [[ -z "${recorder_python}" ]]; then
  echo "Error: python is not on PATH. Activate the lerobot-vr environment first." >&2
  echo "  conda activate lerobot-vr" >&2
  exit 1
fi

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  cd -- "${repository_root}"
  exec "${recorder_python}" -m data_collection.meta_quest.raw_recorder "$@"
fi

echo "Starting validated raw recorder on tcp:${port}..."
echo "ALVR/OpenVR uses Wi-Fi; no adb reverse mapping is required."
cd -- "${repository_root}"
"${recorder_python}" -m data_collection.meta_quest.raw_recorder "$@" --port "${port}" &
recorder_pid=$!

receiver_ready=false
for _ in $(seq 1 50); do
  if ! kill -0 "${recorder_pid}" 2>/dev/null; then
    wait "${recorder_pid}"
    exit $?
  fi
  if ss -ltn "sport = :${port}" 2>/dev/null | grep -q LISTEN; then
    receiver_ready=true
    break
  fi
  sleep 0.1
done

if [[ "${receiver_ready}" != true ]]; then
  echo "Error: raw recorder did not listen on tcp:${port} within 5 seconds." >&2
  exit 1
fi

echo "Starting direct ALVR/OpenVR controller bridge..."
"${script_dir}/start_meta_quest_openvr_bridge.sh" --port "${port}" &
bridge_pid=$!

set +e
wait -n "${recorder_pid}" "${bridge_pid}"
first_status=$?
set -e

if kill -0 "${recorder_pid}" 2>/dev/null; then
  echo "OpenVR bridge stopped; finalizing the partial recording safely." >&2
  kill -INT "${recorder_pid}" 2>/dev/null || true
fi
if kill -0 "${bridge_pid}" 2>/dev/null; then
  kill -TERM "${bridge_pid}" 2>/dev/null || true
fi

stop_children

exit "${first_status}"
