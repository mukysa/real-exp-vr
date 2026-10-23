#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/start_quest3_left_hardware.sh --check-only
  scripts/start_quest3_left_hardware.sh --enable-hardware

The hardware mode controls only left / 172.16.0.3. Left Y toggles:
  first press  -> capture measured pose and take over
  second press -> bounded return to that captured pose

Starting hardware mode activates the left FR3 impedance controller. Keep the
Franka Desk emergency stop available and clear the workspace first.
EOF
}

mode=""
case "${1:-}" in
  --check-only|--enable-hardware) mode="$1" ;;
  --help|-h) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
controller_install="${FRANKA_CONTROLLER_ROS_INSTALL_PREFIX:-${repository_root}/gello_software/ros2/install/client}"
quest_install="${QUEST3_ROS_INSTALL_PREFIX:-${repository_root}/quest3_ros2/install}"
controller_yaml="${repository_root}/gello_software/ros2/src/franka_fr3_arm_controllers/config/controllers.yaml"
vr_yaml="${repository_root}/quest3_ros2/src/franka_vr_adapter/config/vr_controller.yaml"

for path in \
  /opt/ros/humble/setup.bash \
  "${HOME}/franka_ros2_ws/install/local_setup.bash" \
  "${controller_install}/local_setup.bash" \
  "${quest_install}/local_setup.bash" \
  "${controller_yaml}" \
  "${vr_yaml}"; do
  [[ -r "${path}" ]] || { echo "Error: missing ${path}" >&2; exit 1; }
done

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX
unset COLCON_PYTHON_EXECUTABLE AMENT_PYTHON_EXECUTABLE
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
source "${HOME}/franka_ros2_ws/install/local_setup.bash"
source "${controller_install}/local_setup.bash"
source "${quest_install}/local_setup.bash"
set -u

command -v ros2 >/dev/null
command -v flock >/dev/null
controller_spawner="$(ros2 pkg prefix controller_manager)/lib/controller_manager/spawner"
[[ -x "${controller_spawner}" ]] || {
  echo "Error: controller_manager spawner is unavailable: ${controller_spawner}" >&2
  exit 1
}
[[ "$(ros2 pkg prefix franka_fr3_arm_controllers)" == "${controller_install}"* ]]
[[ "$(ros2 pkg prefix franka_vr_adapter)" == "${quest_install}"* ]]
bash "${repository_root}/scripts/start_meta_quest_openvr_bridge.sh" --single-left --dry-run

if [[ "${mode}" == "--check-only" ]]; then
  echo "CHECK OK: left-only configuration, Y button, strict IK source, adapter, and VR controller files are available."
  echo "No robot connection was opened."
  exit 0
fi

if pgrep -af '172\.16\.0\.3|example_fr3_config|left_quest_live_ik|franka_vr_adapter.*left' \
    | grep -v -E 'pgrep|start_quest3_left_hardware' >/dev/null; then
  echo "Error: another LEFT robot/teleoperation process is running; stop it before continuing." >&2
  pgrep -af '172\.16\.0\.3|example_fr3_config|left_quest_live_ik|franka_vr_adapter.*left' >&2 || true
  exit 1
fi
left_gello_info="$(ros2 topic info --no-daemon --spin-time 1 /left/gello/raw_joint_states 2>/dev/null || true)"
left_gello_publishers="$(awk '/Publisher count:/ {print $3}' <<<"${left_gello_info}")"
[[ "${left_gello_publishers:-0}" == "0" ]] || {
  echo "Error: left GELLO command publisher is active." >&2
  exit 1
}
if ss -ltn | awk '{print $4}' | grep -Eq ':(8000|8001)$'; then
  echo "Error: Quest TCP port 8000 or 8001 is already in use." >&2
  echo "Stop start_fr3_live_preview.sh (Ctrl+C in its terminal) before hardware mode." >&2
  exit 1
fi

runtime_dir="${XDG_RUNTIME_DIR:-/tmp}"
exec 9>"${runtime_dir}/real-exp-teleoperation-${UID}.lock"
flock -n 9 || { echo "Error: another GELLO/Quest teleoperation launcher holds the lock." >&2; exit 1; }

log_dir="${repository_root}/outputs/quest3_left_hardware/$(date +%Y%m%d-%H%M%S)"
mkdir -p "${log_dir}"
children=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${children[@]}"; do
    kill -TERM -- "-${pid}" 2>/dev/null || true
  done
  for pid in "${children[@]}"; do
    wait "${pid}" 2>/dev/null || true
  done
  echo "Stopped. The FR3 command watchdog is no longer receiving targets. Logs: ${log_dir}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
start_process() {
  local name="$1"; shift
  setsid -- "$@" >"${log_dir}/${name}.log" 2>&1 &
  children+=("$!")
}
wait_for_topic() {
  local topic="$1" deadline=$((SECONDS + 45))
  while ((SECONDS < deadline)); do
    if timeout 3s ros2 topic echo "${topic}" --once --no-daemon >/dev/null 2>&1; then return 0; fi
  done
  echo "Error: timed out waiting for ${topic}" >&2
  return 1
}
wait_for_port() {
  local port="$1" deadline=$((SECONDS + 30))
  while ((SECONDS < deadline)); do
    if ss -ltn | awk '{print $4}' | grep -Eq ":${port}$"; then return 0; fi
    sleep 0.2
  done
  echo "Error: timed out waiting for TCP ${port}" >&2
  return 1
}

echo "Starting LEFT FR3 hardware controller only (172.16.0.3)."
start_process hardware ros2 launch franka_fr3_arm_controllers franka.launch.py \
  arm_id:=fr3 arm_prefix:=left namespace:=left \
  robot_ip:=172.16.0.3 load_gripper:=false \
  use_fake_hardware:=false fake_sensor_commands:=false \
  joint_sources:=joint_states joint_state_rate:=30 deployment_mode:=false
hardware_pid="${children[${#children[@]}-1]}"
wait_for_topic /left/joint_states

start_process impedance "${controller_spawner}" joint_impedance_controller \
  --controller-manager /left/controller_manager \
  --controller-manager-timeout 30 \
  --param-file "${controller_yaml}" \
  --param-file "${vr_yaml}"
impedance_pid="${children[${#children[@]}-1]}"
if ! wait "${impedance_pid}"; then
  echo "Error: impedance controller spawner failed." >&2
  tail -20 "${log_dir}/impedance.log" >&2 || true
  exit 1
fi
deadline=$((SECONDS + 30))
while ((SECONDS < deadline)); do
  kill -0 "${hardware_pid}" 2>/dev/null || {
    echo "Error: left FR3 hardware process exited. See ${log_dir}/hardware.log" >&2
    exit 1
  }
  if timeout 3s ros2 control list_controllers -c /left/controller_manager 2>/dev/null \
      | grep -Eq 'joint_impedance_controller.*active'; then break; fi
  sleep 0.5
done
controller_status="$(
  timeout 3s ros2 control list_controllers -c /left/controller_manager 2>&1 || true
)"
grep -Eq 'joint_impedance_controller.*active' <<<"${controller_status}" || {
  echo "Error: left impedance controller did not become active." >&2
  printf '%s\n' "${controller_status}" >&2
  exit 1
}

trace_path="${log_dir}/adapter.jsonl"
start_process quest_control ros2 launch franka_vr_adapter left_live_control.launch.py \
  trace_path:="${trace_path}" position_scale:=0.5 max_tcp_offset_m:=0.20
wait_for_port 8001
wait_for_topic /left/vr/source_status
wait_for_topic /left/vr/status

echo "Hardware chain ready. Keep the arm clear. First left Y takes over; second left Y returns."
echo "Press Ctrl+C at any time to stop the chain. Logs: ${log_dir}"
start_process openvr bash "${repository_root}/scripts/start_meta_quest_openvr_bridge.sh" \
  --single-left --host 127.0.0.1 --port 8001
bridge_pid="${children[${#children[@]}-1]}"
last_status=""
last_adapter_status=""
while kill -0 "${bridge_pid}" 2>/dev/null; do
  current_status="$(
    timeout 2s ros2 topic echo /left/vr/source_status --once --field data --no-daemon \
      2>/dev/null | head -1 || true
  )"
  if [[ -n "${current_status}" && "${current_status}" != "${last_status}" ]]; then
    echo "SOURCE_STATUS ${current_status}"
    last_status="${current_status}"
  fi
  adapter_status="$(
    timeout 2s ros2 topic echo /left/vr/status --once --field data --no-daemon \
      2>/dev/null | head -1 || true
  )"
  if [[ -n "${adapter_status}" && "${adapter_status}" != "${last_adapter_status}" ]]; then
    echo "ADAPTER_STATUS ${adapter_status}"
    last_adapter_status="${adapter_status}"
  fi
  sleep 0.2
done
wait "${bridge_pid}"
