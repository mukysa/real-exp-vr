#!/usr/bin/env bash
set -euo pipefail

[[ "$#" -eq 1 && "$1" == "--fake" ]] || {
  echo "Usage: scripts/start_quest3_left_fake.sh --fake" >&2
  exit 2
}
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
root="$(cd -- "${script_dir}/.." && pwd)"
quest_install="${QUEST3_ROS_INSTALL_PREFIX:-${root}/quest3_ros2/install}"
controller_install="${FRANKA_CONTROLLER_ROS_INSTALL_PREFIX:-${root}/gello_software/ros2/install/client}"
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME CMAKE_PREFIX_PATH COLCON_CURRENT_PREFIX AMENT_CURRENT_PREFIX
unset COLCON_PYTHON_EXECUTABLE AMENT_PYTHON_EXECUTABLE AMENT_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
set +u
source /opt/ros/humble/setup.bash
source "${HOME}/franka_ros2_ws/install/local_setup.bash"
source "${controller_install}/local_setup.bash"
source "${quest_install}/local_setup.bash"
set -u
exec ros2 launch franka_vr_adapter left_fake.launch.py enable_synthetic_input:=false
