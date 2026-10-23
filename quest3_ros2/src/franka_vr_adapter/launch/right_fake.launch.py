"""Start the right FR3 fake-hardware stack with the Quest 3 adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    adapter_share = Path(get_package_share_directory("franka_vr_adapter"))
    controller_share = Path(
        get_package_share_directory("franka_fr3_arm_controllers")
    )
    core_launch = controller_share / "launch" / "franka.launch.py"
    controller_defaults = controller_share / "config" / "controllers.yaml"
    vr_override = adapter_share / "config" / "vr_controller.yaml"

    enable_synthetic_input = LaunchConfiguration("enable_synthetic_input")
    enable_fault_injection = LaunchConfiguration("enable_fault_injection")
    trace_path = LaunchConfiguration("trace_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_synthetic_input",
                default_value="false",
                description=(
                    "Generate a small test target. Keep false until fake-hardware "
                    "command validation is explicitly requested."
                ),
            ),
            DeclareLaunchArgument(
                "enable_fault_injection",
                default_value="false",
                description="Expose fake-only fault modes for validation.",
            ),
            DeclareLaunchArgument(
                "trace_path",
                default_value="",
                description="Optional JSONL trace output path.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(core_launch)),
                launch_arguments={
                    "arm_id": "fr3",
                    "arm_prefix": "right",
                    "namespace": "right",
                    "urdf_file": "fr3/fr3.urdf.xacro",
                    # This is deliberately not a robot address. The xacro
                    # interface requires a string even with fake hardware.
                    "robot_ip": "fake-hardware.invalid",
                    "load_gripper": "false",
                    "use_fake_hardware": "true",
                    "fake_sensor_commands": "false",
                    "joint_sources": "joint_states",
                    "joint_state_rate": "30",
                    "deployment_mode": "false",
                }.items(),
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                namespace="right",
                arguments=[
                    "joint_impedance_controller",
                    "--controller-manager-timeout",
                    "30",
                    "--param-file",
                    str(controller_defaults),
                    "--param-file",
                    str(vr_override),
                ],
                output="screen",
            ),
            Node(
                package="franka_vr_adapter",
                executable="adapter_node",
                parameters=[
                    {
                        "enable_synthetic_input": ParameterValue(
                            enable_synthetic_input, value_type=bool
                        ),
                        "enable_fault_injection": ParameterValue(
                            enable_fault_injection, value_type=bool
                        ),
                        "trace_path": ParameterValue(trace_path, value_type=str),
                    }
                ],
                output="screen",
            ),
        ]
    )
