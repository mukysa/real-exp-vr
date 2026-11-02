"""Start selected strict-IK sources and adapters; no robot controller here."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode not in {"left", "right", "duo"}:
        raise ValueError("mode must be left, right or duo")
    sides = ("left", "right") if mode == "duo" else (mode,)
    trace_dir = Path(LaunchConfiguration("trace_dir").perform(context))
    position_scale = float(LaunchConfiguration("position_scale").perform(context))
    max_tcp_offset_m = float(LaunchConfiguration("max_tcp_offset_m").perform(context))
    left_gripper_enabled = LaunchConfiguration("left_gripper_enabled").perform(context).lower() == "true"
    hand_arm = LaunchConfiguration("hand_arm").perform(context).lower() == "true"
    if hand_arm and (mode != "left" or left_gripper_enabled):
        raise ValueError("hand_arm requires left arm without gripper")
    nodes = []
    for side in sides:
        nodes.append(Node(
            package="franka_vr_adapter",
            executable="adapter_node",
            name=f"{side}_vr_adapter",
            parameters=[{
                "side": side,
                # With Franka Hand enabled, /left/joint_states also contains
                # two finger joints. The strict adapter needs the arm-only
                # broadcaster, while the recorder reads gripper state apart.
                "measured_topic": (
                    f"/{side}/franka/joint_states"
                    if side == "left" and left_gripper_enabled
                    else f"/{side}/joint_states"
                ),
                "trace_path": str(trace_dir / f"{side}-adapter.jsonl"),
                "expected_sequence_step": 0,
                "robot_state_timeout_s": 1.0,
                "input_timeout_s": 2.0,
                "ik_timeout_s": 1.0,
                "max_joint_step_rad": 6.0,
                "velocity_scale": 1000.0,
                "initial_match_tolerance_rad": 0.05,
            }],
            output="screen",
        ))
    nodes.append(Node(
        package="franka_vr_adapter",
        executable="multi_live_ik_node",
        additional_env={
            "OPENBLAS_NUM_THREADS": "1",
            "OMP_NUM_THREADS": "1",
            "MKL_NUM_THREADS": "1",
        },
        parameters=[{
            "mode": mode,
            "left_gripper_enabled": left_gripper_enabled,
            "hand_arm": hand_arm,
            "trigger_press": float(LaunchConfiguration("trigger_press").perform(context)),
            "trigger_release": float(LaunchConfiguration("trigger_release").perform(context)),
            "position_scale": position_scale,
            "max_tcp_offset_m": max_tcp_offset_m,
            "measured_timeout_s": 1.0,
            "input_timeout_s": 2.0,
        }],
        output="screen",
    ))
    return nodes


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="left"),
        DeclareLaunchArgument("hand_arm", default_value="false"),
        DeclareLaunchArgument("trigger_press", default_value="0.75"),
        DeclareLaunchArgument("trigger_release", default_value="0.25"),
        DeclareLaunchArgument("left_gripper_enabled", default_value="false"),
        DeclareLaunchArgument("trace_dir", default_value="/tmp/quest3-vr"),
        DeclareLaunchArgument("position_scale", default_value="1.0"),
        DeclareLaunchArgument("max_tcp_offset_m", default_value="0.0"),
        OpaqueFunction(function=_nodes),
    ])
