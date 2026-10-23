#!/usr/bin/env bash
set -eo pipefail

# Safe step-2 smoke test: start only the Quest 3 adapter with its default
# synthetic input disabled.  No controller, fake hardware, or robot is started.
workspace_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
validation_dir="${workspace_root}/validation"
mkdir -p "${validation_dir}"

source /opt/ros/humble/setup.bash
source "${workspace_root}/install/local_setup.bash"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-93}"
export ROS_LOCALHOST_ONLY=1

node_log="${validation_dir}/adapter_node.log"
status_log="${validation_dir}/status_wait_robot.txt"
status_after_state_log="${validation_dir}/status_after_robot_state.txt"
raw_log="${validation_dir}/raw_once.txt"
node_info_log="${validation_dir}/node_info.txt"
topic_list_log="${validation_dir}/topic_list.txt"

adapter_executable="${workspace_root}/install/franka_vr_adapter/lib/franka_vr_adapter/adapter_node"
[[ -x "${adapter_executable}" ]]
"${adapter_executable}" >"${node_log}" 2>&1 &
node_pid=$!

cleanup() {
  if kill -0 "${node_pid}" 2>/dev/null; then
    kill -s INT "${node_pid}" 2>/dev/null || true
    wait "${node_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT

sleep 1
ros2 topic list --no-daemon --spin-time 2 --show-types >"${topic_list_log}"
timeout 8 ros2 topic echo --no-daemon --spin-time 2 --full-length --once \
  /right/vr/status std_msgs/msg/String >"${status_log}"

status_type="std_msgs/msg/String"
raw_type="sensor_msgs/msg/JointState"
synthetic_setting="$(ros2 param get --no-daemon --spin-time 2 /franka_vr_adapter enable_synthetic_input)"
ros2 node info --no-daemon --spin-time 2 /franka_vr_adapter >"${node_info_log}"

ros2 topic pub --once /right/joint_states sensor_msgs/msg/JointState \
  "{name: [right_fr3_joint1, right_fr3_joint2, right_fr3_joint3, right_fr3_joint4, right_fr3_joint5, right_fr3_joint6, right_fr3_joint7], position: [0.0, -0.4, 0.0, -1.8, 0.0, 1.5, 0.7]}" \
  >/dev/null
timeout 8 ros2 topic echo --no-daemon --spin-time 2 --full-length --once \
  /right/vr/status std_msgs/msg/String >"${status_after_state_log}"

set +e
timeout 3 ros2 topic echo --no-daemon --spin-time 1 --once \
  /right/vr/raw_joint_states sensor_msgs/msg/JointState >"${raw_log}"
raw_echo_exit=$?
set -e

[[ "${status_type}" == "std_msgs/msg/String" ]]
[[ "${raw_type}" == "sensor_msgs/msg/JointState" ]]
[[ "${synthetic_setting}" == "Boolean value is: False" ]]
[[ "${raw_echo_exit}" -eq 124 ]]
grep -q '/right/vr/status \[std_msgs/msg/String\]' "${topic_list_log}"
grep -q '/right/vr/raw_joint_states \[sensor_msgs/msg/JointState\]' "${topic_list_log}"
grep -q '"state":"WAIT_ROBOT"' "${status_log}"
grep -q '"reason":"robot_state_missing"' "${status_log}"
grep -q '"raw_publish_count":0' "${status_log}"
grep -q '/right/joint_states: sensor_msgs/msg/JointState' "${node_info_log}"
grep -q '/right/vr/accepted_joint_states: sensor_msgs/msg/JointState' "${node_info_log}"
grep -q '"state":"HOLD"' "${status_after_state_log}"
grep -q '"reason":"synthetic_input_disabled"' "${status_after_state_log}"
grep -q '"have_robot_state":true' "${status_after_state_log}"
grep -q '"raw_publish_count":0' "${status_after_state_log}"

echo "STATUS_TYPE=${status_type}"
echo "RAW_TYPE=${raw_type}"
echo "${synthetic_setting}"
echo "RAW_ECHO_EXIT=${raw_echo_exit} (124 means no message arrived before timeout)"
cat "${status_log}"
cat "${status_after_state_log}"
echo "STEP2_RUNTIME_VALIDATION=PASS"
