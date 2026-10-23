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
            }],
            output="screen",
        ),
        Node(
            package="franka_vr_adapter",
            executable="live_ik_node",
            parameters=[{
                "position_scale": ParameterValue(position_scale, value_type=float),
                "max_tcp_offset_m": ParameterValue(max_tcp_offset_m, value_type=float),
            }],
            output="screen",
        ),
    ])
