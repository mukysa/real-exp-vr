#!/usr/bin/env bash
set -euo pipefail

# Build only the GELLO client packages without inheriting Conda or ROS overlays.
# Keep the copied build/install trees intact; their hooks contain old paths.
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repository_root="$(cd -- "${script_dir}/.." && pwd)"

exec env -i HOME="${HOME}" USER="${USER:-landau}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  PYTHONDONTWRITEBYTECODE=1 \
  bash --noprofile --norc -c '
set -eo pipefail
source /opt/ros/humble/setup.bash
source "${HOME}/franka_ros2_ws/install/local_setup.bash"
set -u
cd -- "$1/gello_software/ros2"
exec /usr/bin/colcon --log-base log/client build \
  --base-paths src \
  --build-base build/client \
  --install-base install/client \
  --symlink-install \
  --packages-select franka_gello_state_publisher franka_fr3_arm_controllers franka_gripper_manager \
  --executor sequential \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3 \
  --event-handlers console_direct+
' _ "${repository_root}" </dev/null
