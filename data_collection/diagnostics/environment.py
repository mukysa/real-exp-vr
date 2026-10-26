"""Run with the ROS system Python after sourcing the intended overlays."""
import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import platform
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def command(args):
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=8)
        return {"exit_code": result.returncode, "stdout": result.stdout.strip(),
                "stderr": result.stderr.strip()}
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {"error": str(exc)}


def report():
    messages = {}
    for name in ("sensor_msgs/msg/JointState", "franka_msgs/msg/FrankaRobotState", "std_msgs/msg/String"):
        result = command(["ros2", "interface", "show", name])
        if result.get("exit_code") == 0:
            result["definition_sha256"] = hashlib.sha256(result["stdout"].encode()).hexdigest()
        messages[name] = result
    return {
        "hostname": platform.node(), "utc_unix_s": time.time(), "python": sys.executable,
        "repository": str(ROOT),
        "environment": {key: os.environ.get(key) for key in (
            "ROS_DISTRO", "ROS_DOMAIN_ID", "RMW_IMPLEMENTATION", "ROS_LOCALHOST_ONLY",
            "ROS_AUTOMATIC_DISCOVERY_RANGE", "CYCLONEDDS_URI", "AMENT_PREFIX_PATH")},
        "git": command(["git", "-C", str(ROOT), "rev-parse", "HEAD"]),
        "submodules": command(["git", "-C", str(ROOT), "submodule", "status", "gello_software", "lerobot"]),
        "ros_installations": [str(p) for p in Path('/opt/ros').glob('*/setup.bash')],
        "server_overlay_exists": (ROOT / 'gello_software/ros2/install/server/local_setup.bash').exists(),
        "packages": {name: command(["ros2", "pkg", "prefix", name]) for name in (
            "franka_msgs", "franka_lerobot_data_bridge", "franka_realsense_camera_publisher")},
        "modules": {name: importlib.util.find_spec(name) is not None for name in (
            "rclpy", "numpy", "zmq", "pyrealsense2")},
        "message_definitions": messages,
        "addresses": command(["ip", "-br", "addr"]),
        "clock_sync": command(["timedatectl", "show", "-p", "NTPSynchronized", "-p", "NTP"]),
        "note": "Clock synchronization status is not a measurement of cross-host clock offset."
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    data = report()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(data, indent=2) + '\n')
    print(f'Environment report: {args.output}')


if __name__ == '__main__':
    main()
