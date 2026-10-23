"""Start the left FR3 fake-hardware Quest adapter; never addresses a robot."""

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
    controller_share = Path(get_package_share_directory("franka_fr3_arm_controllers"))
    core_launch = controller_share / "launch" / "franka.launch.py"
    controller_defaults = controller_share / "config" / "controllers.yaml"
    vr_override = adapter_share / "config" / "vr_controller.yaml"
    synthetic = LaunchConfiguration("enable_synthetic_input")
    faults = LaunchConfiguration("enable_fault_injection")
    trace = LaunchConfiguration("trace_path")
    return LaunchDescription([
        DeclareLaunchArgument("enable_synthetic_input", default_value="false"),
        DeclareLaunchArgument("enable_fault_injection", default_value="false"),
        DeclareLaunchArgument("trace_path", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(core_launch)),
            launch_arguments={
                "arm_id": "fr3", "arm_prefix": "left", "namespace": "left",
                "urdf_file": "fr3/fr3.urdf.xacro",
                "robot_ip": "fake-hardware.invalid", "load_gripper": "false",
                "use_fake_hardware": "true", "fake_sensor_commands": "false",
                "joint_sources": "joint_states", "joint_state_rate": "30",
                "deployment_mode": "false",
            }.items(),
        ),
        Node(package="controller_manager", executable="spawner", namespace="left",
             arguments=["joint_impedance_controller", "--controller-manager-timeout", "30",
                        "--param-file", str(controller_defaults), "--param-file", str(vr_override)], output="screen"),
        Node(package="franka_vr_adapter", executable="adapter_node",
             parameters=[{"enable_synthetic_input": ParameterValue(synthetic, value_type=bool),
                          "enable_fault_injection": ParameterValue(faults, value_type=bool),
                          "trace_path": ParameterValue(trace, value_type=str),
                          "measured_topic": "/left/joint_states",
                          "accepted_topic": "/left/vr/accepted_joint_states",
                          "raw_topic": "/left/vr/raw_joint_states",
                          "status_topic": "/left/vr/status",
                          "gello_topic": "/left/gello/raw_joint_states"}], output="screen"),
    ])
