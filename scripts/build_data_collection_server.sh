#!/usr/bin/env bash
set -euo pipefail

# Build only the server packages, independently of the client and old overlays.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"
ros_distro="${DATA_SERVER_ROS_DISTRO:-${ROS_DISTRO:-}}"
if [[ -z "${ros_distro}" ]]; then
  shopt -s nullglob
  setups=(/opt/ros/*/setup.bash)
  [[ "${#setups[@]}" -eq 1 ]] || {
    echo "Set DATA_SERVER_ROS_DISTRO to the installed ROS 2 distribution." >&2
    exit 1
  }
  ros_distro="$(basename -- "$(dirname -- "${setups[0]}")")"
fi

exec env -i HOME="${HOME}" USER="${USER:-pair1}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  PYTHONDONTWRITEBYTECODE=1 \
  bash --noprofile --norc -c '
set -eo pipefail
source "/opt/ros/$2/setup.bash"
source "${HOME}/franka_ros2_ws/install/local_setup.bash"
set -u
cd -- "$1/gello_software/ros2"
exec /usr/bin/colcon --log-base log/server build \
  --base-paths src \
  --build-base build/server \
  --install-base install/server \
  --symlink-install \
  --packages-select franka_realsense_camera_publisher franka_lerobot_data_bridge \
  --executor sequential \
  --event-handlers console_direct+
' _ "${repository_root}" "${ros_distro}" </dev/null
