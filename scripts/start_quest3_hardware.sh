#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/start_quest3_hardware.sh --left|--right|--duo --check-only
       scripts/start_quest3_hardware.sh --left|--right|--duo --enable-hardware

Physical left Touch Y toggles takeover and return for the selected arm(s).
Both Touch controllers are tracked in every mode. Hardware mode starts only
the selected FR3 controller(s): left=172.16.0.3, right=172.16.0.2.
EOF
}

mode=""
action=""
for argument in "$@"; do
  case "${argument}" in
    --left|--right|--duo)
      [[ -z "${mode}" ]] || { usage >&2; exit 2; }
      mode="${argument#--}"
      ;;
    --check-only|--enable-hardware)
      [[ -z "${action}" ]] || { usage >&2; exit 2; }
      action="${argument}"
      ;;
    --help|-h) usage; exit 0 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ -n "${mode}" && -n "${action}" ]] || { usage >&2; exit 2; }

case "${mode}" in
  left) sides=(left) ;;
  right) sides=(right) ;;
  duo) sides=(left right) ;;
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
[[ -x "${controller_spawner}" ]] || { echo "Error: controller spawner missing" >&2; exit 1; }
[[ "$(ros2 pkg prefix franka_fr3_arm_controllers)" == "${controller_install}"* ]]
[[ "$(ros2 pkg prefix franka_vr_adapter)" == "${quest_install}"* ]]
bash "${repository_root}/scripts/start_meta_quest_openvr_bridge.sh" --global-left-y --dry-run

if [[ "${action}" == "--check-only" ]]; then
  echo "CHECK OK: mode=${mode}; both Touch inputs, left Y, selected IK and adapters configured."
  echo "No robot connection was opened."
  exit 0
fi

bash "${repository_root}/scripts/start_meta_quest_openvr_bridge.sh" --check-both-touch

for side in "${sides[@]}"; do
  ip="172.16.0.3"
  [[ "${side}" == "right" ]] && ip="172.16.0.2"
  if pgrep -af "${ip}|${side}.*franka.launch.py" | grep -v -E 'pgrep|start_quest3_hardware' >/dev/null; then
    echo "Error: another ${side} robot process is running." >&2
    exit 1
  fi
  gello_info="$(ros2 topic info --no-daemon --spin-time 1 "/${side}/gello/raw_joint_states" 2>/dev/null || true)"
  gello_publishers="$(awk '/Publisher count:/ {print $3}' <<<"${gello_info}")"
  [[ "${gello_publishers:-0}" == "0" ]] || {
    echo "Error: ${side} GELLO command publisher is active." >&2
    exit 1
  }
done
if ss -ltn | awk '{print $4}' | grep -Eq ':(8000|8001)$'; then
  echo "Error: Quest TCP port 8000 or 8001 is already in use." >&2
  exit 1
fi

runtime_dir="${XDG_RUNTIME_DIR:-/tmp}"
exec 9>"${runtime_dir}/real-exp-teleoperation-${UID}.lock"
flock -n 9 || { echo "Error: another GELLO/Quest launcher holds the lock." >&2; exit 1; }

log_dir="${repository_root}/outputs/quest3_hardware/$(date +%Y%m%d-%H%M%S)-${mode}"
mkdir -p "${log_dir}"
children=()
topic_probe_pid=""
hardware_pids=()
declare -A hardware_side_by_pid
cleanup() {
  trap - EXIT INT TERM
  if [[ -n "${topic_probe_pid}" ]]; then
    kill "${topic_probe_pid}" 2>/dev/null || true
    wait "${topic_probe_pid}" 2>/dev/null || true
  fi
  for pid in "${children[@]}"; do kill -TERM -- "-${pid}" 2>/dev/null || true; done
  for pid in "${children[@]}"; do wait "${pid}" 2>/dev/null || true; done
  echo "Stopped. FR3 command watchdogs no longer receive targets. Logs: ${log_dir}"
}
trap cleanup EXIT
trap 'exit 130' INT TERM
start_process() {
  local name="$1"; shift
  setsid -- "$@" >"${log_dir}/${name}.log" 2>&1 &
  children+=("$!")
}
require_hardware_alive() {
  local pid side
  for pid in "${hardware_pids[@]}"; do
    kill -0 "${pid}" 2>/dev/null || {
      side="${hardware_side_by_pid[$pid]}"
      echo "Error: ${side} FR3 hardware process exited. Logs: ${log_dir}" >&2
      grep -E 'FCI_DIAGNOSTIC|communication_constraints_violation|\[ERROR\]|  what\(\):' \
        "${log_dir}/hardware-${side}.log" | tail -8 >&2 || true
      return 1
    }
  done
}
wait_for_topic() {
  local topic="$1" message_type="$2"
  local probe_log="${log_dir}/wait-${topic//\//_}.log"
  # Keep one subscriber alive for discovery and reception; restarting every
  # three seconds can repeatedly discard DDS discovery progress.
  timeout 45s ros2 topic echo "${topic}" "${message_type}" \
    --once --no-daemon >"${probe_log}" 2>&1 &
  local probe_pid=$!
  topic_probe_pid="${probe_pid}"
  while kill -0 "${probe_pid}" 2>/dev/null; do
    if ! require_hardware_alive; then
      kill "${probe_pid}" 2>/dev/null || true
      wait "${probe_pid}" 2>/dev/null || true
      return 1
    fi
    sleep 0.2
  done
  if wait "${probe_pid}"; then
    topic_probe_pid=""
    return 0
  fi
  topic_probe_pid=""
  echo "Error: failed waiting for ${topic}; diagnostic: ${probe_log}" >&2
  tail -15 "${probe_log}" >&2
  return 1
}

wait_for_port() {
  local port="$1" deadline=$((SECONDS + 30))
  while ((SECONDS < deadline)); do
    require_hardware_alive
    if ss -ltn | awk '{print $4}' | grep -Eq ":${port}$"; then return 0; fi
    sleep 0.2
  done
  echo "Error: timed out waiting for TCP ${port}" >&2
  return 1
}

for side in "${sides[@]}"; do
  ip="172.16.0.3"
  [[ "${side}" == "right" ]] && ip="172.16.0.2"
  echo "Starting ${side^^} FR3 hardware controller (${ip})."
  start_process "hardware-${side}" ros2 launch franka_fr3_arm_controllers franka.launch.py \
    arm_id:=fr3 arm_prefix:="${side}" namespace:="${side}" \
    robot_ip:="${ip}" load_gripper:=false \
    use_fake_hardware:=false fake_sensor_commands:=false \
    joint_sources:=joint_states joint_state_rate:=30 deployment_mode:=false
  hardware_pids+=("${children[${#children[@]}-1]}")
  hardware_side_by_pid["${children[${#children[@]}-1]}"]="${side}"
  wait_for_topic "/${side}/joint_states" sensor_msgs/msg/JointState
done

for side in "${sides[@]}"; do
  start_process "impedance-${side}" "${controller_spawner}" joint_impedance_controller \
    --controller-manager "/${side}/controller_manager" \
    --controller-manager-timeout 30 \
    --param-file "${controller_yaml}" --param-file "${vr_yaml}"
  spawner_pid="${children[${#children[@]}-1]}"
  if ! wait "${spawner_pid}"; then
    echo "Error: ${side} impedance controller spawner failed." >&2
    tail -20 "${log_dir}/impedance-${side}.log" >&2 || true
    exit 1
  fi
  deadline=$((SECONDS + 30))
  while ((SECONDS < deadline)); do
    require_hardware_alive
    if timeout 3s ros2 control list_controllers -c "/${side}/controller_manager" 2>/dev/null \
      | grep -Eq 'joint_impedance_controller.*active'; then break; fi
    sleep 0.5
  done
  controller_status="$(timeout 3s ros2 control list_controllers -c "/${side}/controller_manager" 2>&1 || true)"
  grep -Eq 'joint_impedance_controller.*active' <<<"${controller_status}" || {
    echo "Error: ${side} impedance controller did not become active." >&2
    printf '%s\n' "${controller_status}" >&2
    exit 1
  }
done

start_process quest_control ros2 launch franka_vr_adapter quest_live_control.launch.py \
  mode:="${mode}" trace_dir:="${log_dir}" position_scale:=1.0 max_tcp_offset_m:=0.0
wait_for_port 8001
for side in "${sides[@]}"; do
  wait_for_topic "/${side}/vr/source_status" std_msgs/msg/String
  wait_for_topic "/${side}/vr/status" std_msgs/msg/String
done

echo "Hardware chain ready: ${mode}. Left Y takes over selected arm(s); second Y returns."
echo "Press Ctrl+C at any time to stop. Logs: ${log_dir}"
start_process openvr bash "${repository_root}/scripts/start_meta_quest_openvr_bridge.sh" \
  --global-left-y --host 127.0.0.1 --port 8001
bridge_pid="${children[${#children[@]}-1]}"
declare -A last_source_status last_adapter_status
while kill -0 "${bridge_pid}" 2>/dev/null; do
  require_hardware_alive
  for side in "${sides[@]}"; do
    source_status="$(timeout 2s ros2 topic echo "/${side}/vr/source_status" --once --field data --no-daemon 2>/dev/null | head -1 || true)"
    if [[ -n "${source_status}" && "${source_status}" != "${last_source_status[$side]:-}" ]]; then
      echo "${side^^}_SOURCE_STATUS ${source_status}"
      last_source_status[$side]="${source_status}"
    fi
    adapter_status="$(timeout 2s ros2 topic echo "/${side}/vr/status" --once --field data --no-daemon 2>/dev/null | head -1 || true)"
    if [[ -n "${adapter_status}" && "${adapter_status}" != "${last_adapter_status[$side]:-}" ]]; then
      echo "${side^^}_ADAPTER_STATUS ${adapter_status}"
      last_adapter_status[$side]="${adapter_status}"
    fi
  done
  sleep 0.2
done
wait "${bridge_pid}"
