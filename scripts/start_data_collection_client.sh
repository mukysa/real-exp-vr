#!/usr/bin/env bash
set -euo pipefail

# Start the robot-control side of a split data-collection setup.
# Cameras, the ROS bridge, and the LeRobot recorder run on the data server;
# this host runs only GELLO/Franka teleoperation and robot control.
# Usage: ./scripts/start_data_collection_client.sh --duo|--left|--right --arm|--gripper|--hand [--teleop-source gello|quest] [--check-only|--enable-hardware]

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

if [[ "$#" -eq 1 && "$1" == "--help" ]]; then
  cat <<'EOF'
Usage: ./scripts/start_data_collection_client.sh --duo|--left|--right --arm|--gripper|--hand [--teleop-source gello|quest] [--check-only|--enable-hardware]

Quest currently supports --left --arm or --left --gripper and requires --check-only or
--enable-hardware. GELLO remains the default.
Use --teleop-source quest --left --hand-arm for left pose/right B/A/trigger testing.
--hand-arm implies --arm and is incompatible with --gripper/--hand.

Starts only the control-host teleoperation stack. On the data server run:
  ./scripts/start_data_collection_server.sh --duo|--left|--right --arm|--gripper|--hand [--teleop-source gello|quest] [--record]
EOF
  exit 0
fi

arm_mode=""
data_mode=""
teleop_source="gello"
quest_action=""
hand_arm=false
while [[ "$#" -gt 0 ]]; do
  argument="$1"
  shift
  case "${argument}" in
    --duo|--left|--right|--single)
      [[ -z "${arm_mode}" ]] || { echo "choose only one arm mode" >&2; exit 2; }
      arm_mode="${argument#--}"
      [[ "${arm_mode}" != "single" ]] || arm_mode="left" ;;
    --arm|--gripper|--no-gripper|--hand)
      [[ -z "${data_mode}" ]] || { echo "choose only one data mode" >&2; exit 2; }
      data_mode="${argument#--}"
      [[ "${data_mode}" != "no-gripper" ]] || data_mode="arm" ;;
    --hand-arm) hand_arm=true ;;
    --teleop-source)
      [[ "$#" -gt 0 ]] || { echo "--teleop-source requires gello or quest" >&2; exit 2; }
      teleop_source="$1"; shift ;;
    --check-only|--enable-hardware)
      [[ -z "${quest_action}" ]] || { echo "choose only one hardware action" >&2; exit 2; }
      quest_action="${argument}" ;;
    *) echo "unknown argument: ${argument}" >&2; exit 2 ;;
  esac
done
if [[ "${hand_arm}" == true ]]; then
  [[ -n "${data_mode}" ]] || data_mode=arm
  [[ "${teleop_source}" == quest && "${arm_mode}" == left && "${data_mode}" == arm ]] || {
    echo "--hand-arm requires --teleop-source quest --left and arm-only mode" >&2; exit 2;
  }
fi
[[ -n "${arm_mode}" && -n "${data_mode}" ]] || { echo "both arm and data modes are required" >&2; exit 2; }

case "${teleop_source}" in
  gello|quest) ;;
  *) echo "unsupported teleop source: ${teleop_source}" >&2; exit 2 ;;
esac
if [[ "${teleop_source}" == "quest" ]]; then
  [[ "${arm_mode}" == "left" && ( "${data_mode}" == "arm" || "${data_mode}" == "gripper" ) ]] || {
    echo "Quest collection currently supports only --left --arm or --left --gripper" >&2; exit 2;
  }
  [[ -n "${quest_action}" ]] || {
    echo "Quest requires --check-only or explicit --enable-hardware" >&2; exit 2;
  }
  repository_root="$(cd -- "${script_dir}/.." && pwd)"
  dds_config="${repository_root}/config/data_collection/cyclonedds-client.xml"
  [[ -r "${dds_config}" ]] || {
    echo "Quest collection DDS config is missing: ${dds_config}" >&2; exit 1;
  }
  export CYCLONEDDS_URI="file://${dds_config}"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
  export ROS_LOCALHOST_ONLY=0
  export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
  unset ROS_STATIC_PEERS
  echo "Quest collection DDS: wired client to server via ${dds_config}"
  gripper_option=()
  [[ "${data_mode}" == "gripper" ]] && gripper_option=(--gripper)
  [[ "${hand_arm}" == false ]] || gripper_option=(--hand-arm)
  exec bash "${script_dir}/start_quest3_hardware.sh" --left "${gripper_option[@]}" "${quest_action}"
fi
[[ -z "${quest_action}" ]] || { echo "hardware action flags require --teleop-source quest" >&2; exit 2; }

echo "Starting control-host teleoperation only."
echo "Run scripts/start_data_collection_server.sh on 192.168.52.160 for cameras and recording."
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
arm_side="--${arm_mode}"
teleop_launcher="${script_dir}/start_teleoperation.sh"

if [[ "${data_mode}" != "hand" ]]; then
  # Use this checkout's client-only build, not copied install hooks that may
  # still reference the original workspace. ROS runs with system Python.
  repository_root="$(cd -- "${script_dir}/.." && pwd)"
  export REAL_EXP_ROS_INSTALL_PREFIX="${repository_root}/gello_software/ros2/install/client"
  [[ -r "${REAL_EXP_ROS_INSTALL_PREFIX}/local_setup.bash" ]] || {
    echo "Client ROS build is missing. Run scripts/build_data_collection_client.sh first." >&2
    exit 1
  }
  export LEROBOT_CONDA_ENV=lerobot-vr
  export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
  unset PYTHONHOME CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX
  unset COLCON_PYTHON_EXECUTABLE AMENT_PYTHON_EXECUTABLE
fi

if [[ "${data_mode}" == "gripper" ]]; then
  exec "${teleop_launcher}" --gripper "${arm_side}"
elif [[ "${data_mode}" == "arm" ]]; then
  exec "${teleop_launcher}" --arm "${arm_side}"
fi

server_host="${DATA_COLLECTION_SERVER_IP:-192.168.52.160}"
telemetry_port="${HAND_TELEMETRY_PORT:-5558}"
exec "${teleop_launcher}" --hand "${arm_side}" \
  --telemetry-host "${server_host}" --telemetry-port "${telemetry_port}"
