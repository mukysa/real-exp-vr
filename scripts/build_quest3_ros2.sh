#!/usr/bin/env bash
set -euo pipefail

# Build Quest 3 into its own build/install/log directories.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
quest_workspace="${repository_root}/quest3_ros2"
controller_install="${FRANKA_CONTROLLER_ROS_INSTALL_PREFIX:-${repository_root}/gello_software/ros2/install/client}"

[[ -r "${controller_install}/local_setup.bash" ]] || {
  echo "Error: FR3 controller install is missing: ${controller_install}" >&2
  echo "Build the existing FR3 controller workspace first." >&2
  exit 1
}

exec env -i HOME="${HOME}" USER="${USER:-landau}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  PYTHONDONTWRITEBYTECODE=1 \
  FRANKA_CONTROLLER_ROS_INSTALL_PREFIX="${controller_install}" \
  bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source "${HOME}/franka_ros2_ws/install/local_setup.bash"
source "${FRANKA_CONTROLLER_ROS_INSTALL_PREFIX}/local_setup.bash"
set -u
cd -- "$1"
exec /usr/bin/colcon --log-base log build \
  --base-paths src \
  --build-base build \
  --install-base install \
  --symlink-install \
  --packages-select franka_vr_adapter \
  --executor sequential \
  --event-handlers console_direct+
' _ "${quest_workspace}" </dev/null
