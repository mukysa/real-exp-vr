"""Start the left Quest strict-IK source and gated ROS adapter only."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    trace_path = LaunchConfiguration("trace_path")
    position_scale = LaunchConfiguration("position_scale")
    max_tcp_offset_m = LaunchConfiguration("max_tcp_offset_m")
    return LaunchDescription([
        DeclareLaunchArgument("trace_path", default_value=""),
        DeclareLaunchArgument("position_scale", default_value="0.03"),
        DeclareLaunchArgument("max_tcp_offset_m", default_value="0.005"),
        Node(
            package="franka_vr_adapter",
            executable="adapter_node",
            parameters=[{
                "side": "left",
                "trace_path": ParameterValue(trace_path, value_type=str),
                # Receiver sequences continue advancing while the source is
                # READY and emits no IKResult.  Require monotonicity, but do
                # not treat that intentional inter-episode gap as packet loss.
                "expected_sequence_step": 0,
                # Continuity-first live profile.  The hardware controller's
                # 0.5 s watchdog and 1 kHz bounded reference remain active.
                "robot_state_timeout_s": 1.0,
                "input_timeout_s": 2.0,
                "ik_timeout_s": 1.0,
                "max_joint_step_rad": 6.0,
                "velocity_scale": 1000.0,
                "initial_match_tolerance_rad": 0.05,
            }],
            output="screen",
        ),
        Node(
            package="franka_vr_adapter",
            executable="live_ik_node",
            parameters=[{
                "position_scale": ParameterValue(position_scale, value_type=float),
                "max_tcp_offset_m": ParameterValue(max_tcp_offset_m, value_type=float),
                "measured_timeout_s": 1.0,
                "input_timeout_s": 2.0,
                "relaxed_motion_gates": True,
                "max_source_translation_step_m": 2.0,
                "max_source_rotation_step_rad": 3.141592653589793,
            }],
            output="screen",
        ),
    ])
