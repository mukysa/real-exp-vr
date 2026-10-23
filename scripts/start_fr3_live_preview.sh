#!/usr/bin/env bash
set -eo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
unset PYTHONPATH PYTHONHOME
export PYTHONDONTWRITEBYTECODE=1
source /opt/ros/humble/setup.bash
source /home/landau/franka_ros2_ws/install/local_setup.bash
exec /usr/bin/python3 -m data_collection.fr3_ik.live.server "$@"
