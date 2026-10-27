#!/usr/bin/env bash
# Requires ROS and the intended bridge overlay to have been sourced.
set -euo pipefail
[[ "${ROS_DOMAIN_ID:-}" == 173 ]] || { echo 'Set ROS_DOMAIN_ID=173 for isolated M2 testing.' >&2; exit 2; }
repository_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
exec ros2 run franka_lerobot_data_bridge lerobot_data_bridge --ros-args \
  --params-file "${repository_root}/config/data_collection/quest_left.yaml" \
  -p publish_host:=127.0.0.1 -p publish_port:=15555 \
  -p camera_cache_host:=127.0.0.1 -p camera_cache_port:=15557 \
  -p deployment_mode:=false \
  -r /left/joint_states:=/m2/left/joint_states \
  -r /left/vr/accepted_joint_states:=/m2/left/vr/accepted_joint_states \
  -r /left/franka_robot_state_broadcaster/robot_state:=/m2/left/franka_robot_state_broadcaster/robot_state \
  -r /cameras/cam_left/image_raw:=/m2/cameras/cam_left/image_raw
