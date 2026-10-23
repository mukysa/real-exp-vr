#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Usage: scripts/start_quest3_teleoperation_client.sh --side left|right [--trace PATH]

Starts only the Quest 3 adapter. It does not launch a robot controller or GELLO.
EOF
}

die() {
  echo "Error: $*" >&2
  exit 1
}

if [[ "$#" -eq 1 && "$1" == "--help" ]]; then
  usage
  exit 0
fi
side=""
trace_path=""
while (($#)); do
  case "$1" in
    --side) side="${2:-}"; shift 2 ;;
    --trace) trace_path="${2:-}"; shift 2 ;;
    *) usage >&2; exit 2 ;;
  esac
done
[[ "${side}" == "left" || "${side}" == "right" ]] || die "--side must be left or right"

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
quest_install="${QUEST3_ROS_INSTALL_PREFIX:-${repository_root}/quest3_ros2/install}"
controller_install="${FRANKA_CONTROLLER_ROS_INSTALL_PREFIX:-${repository_root}/gello_software/ros2/install/client}"

export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX
unset COLCON_PYTHON_EXECUTABLE AMENT_PYTHON_EXECUTABLE
unset AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH

setup_files=(
  "/opt/ros/humble/setup.bash"
  "${HOME}/franka_ros2_ws/install/local_setup.bash"
  "${controller_install}/local_setup.bash"
  "${quest_install}/local_setup.bash"
)
set +u
for setup_file in "${setup_files[@]}"; do
  [[ -r "${setup_file}" ]] || die "ROS setup file is missing: ${setup_file}"
  # shellcheck disable=SC1090
  source "${setup_file}"
done
set -u

command -v flock >/dev/null 2>&1 || die "required command not found: flock"
command -v ros2 >/dev/null 2>&1 || die "required command not found: ros2"

runtime_dir="${XDG_RUNTIME_DIR:-/tmp}"
launcher_lock_path="${runtime_dir}/real-exp-teleoperation-${UID}.lock"
exec 9>"${launcher_lock_path}"
flock -n 9 || die "another GELLO or Quest 3 arm teleoperation instance is already running"

gello_topic_info="$(
  ros2 topic info --no-daemon --spin-time 1 "/${side}/gello/raw_joint_states" 2>/dev/null || true
)"
gello_publishers="$(awk '/Publisher count:/ {print $3}' <<<"${gello_topic_info}")"
gello_publishers="${gello_publishers:-0}"
[[ "${gello_publishers}" =~ ^[0-9]+$ ]] || die "could not parse GELLO publisher count"
((gello_publishers == 0)) || die \
  "GELLO is already publishing /${side}/gello/raw_joint_states; stop it before Quest 3"

echo "Starting Quest 3 ${side}-arm adapter only; no controller is launched."
exec ros2 launch franka_vr_adapter adapter.launch.py side:="${side}" trace_path:="${trace_path}"
