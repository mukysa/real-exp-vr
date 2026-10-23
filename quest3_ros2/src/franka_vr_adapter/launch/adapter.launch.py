"""Start only the Quest-to-FR3 adapter; controllers are launched separately."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    side = LaunchConfiguration("side")
    trace_path = LaunchConfiguration("trace_path")
    return LaunchDescription([
        DeclareLaunchArgument("side", default_value="right", choices=("left", "right")),
        DeclareLaunchArgument("trace_path", default_value=""),
        Node(
            package="franka_vr_adapter",
            executable="adapter_node",
            parameters=[{
                "side": ParameterValue(side, value_type=str),
                "trace_path": ParameterValue(trace_path, value_type=str),
            }],
            output="screen",
        ),
    ])
