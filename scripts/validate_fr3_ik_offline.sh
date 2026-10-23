#!/usr/bin/env bash
set -eo pipefail
repository_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unset PYTHONPATH PYTHONHOME AMENT_PREFIX_PATH COLCON_PREFIX_PATH CMAKE_PREFIX_PATH
unset LD_LIBRARY_PATH ROS_PACKAGE_PATH
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source /home/landau/franka_ros2_ws/install/local_setup.bash
cd "${repository_root}"
exec /usr/bin/python3 -m data_collection.fr3_ik.offline_validator "$@"
