"""Relay ALVR/SteamVR Touch-controller input into the stage-2 pose contract.

This is a robot-disconnected input adapter.  It reuses the same SAPIEN
``RenderVRDisplay`` path as DexArena, converts SAPIEN's right-handed world
coordinates back into the Unity-like source frame expected by
``TeleoperationStateMachine``, and sends compact controller records to a local
``PoseReceiver`` TCP listener.

SAPIEN is imported lazily so the regular ``lerobot-vr`` environment can run
all protocol, replay, and safety tests without carrying the simulation stack.
The bridge itself is launched with ``scripts/start_meta_quest_steamvr_bridge.sh``
using the existing DexArena/SAPIEN Python installation.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import socket
import time
from typing import Any, Sequence

from .calibration import RigidPose
from .coordinates import (
    UNITY_TO_ROBOT_BASIS,
    change_rotation_basis,
    transform_vector,
)


# Matches the legacy OpenVR input masks used by DexArena's vr_native/display.py.
BUTTON_TRIGGER = 1 << 33
BUTTON_GRIP = 1 << 2
BUTTON_A = 1 << 7
BUTTON_B = 1 << 1

# DexArena's validated Quest/ALVR rig exposes these controller indices.
CONTROLLER_LEFT = 1
CONTROLLER_RIGHT = 2
CONTROLLERS = {"left": CONTROLLER_LEFT, "right": CONTROLLER_RIGHT}

ROBOT_TO_UNITY_BASIS = tuple(
    tuple(UNITY_TO_ROBOT_BASIS[column][row] for column in range(3))
    for row in range(3)
)


def sapien_pose_to_unity(
    position: Sequence[float],
    quaternion_wxyz: Sequence[float],
) -> RigidPose:
    """Convert a SAPIEN/OpenVR pose into the existing Unity source contract.

    DexArena exposes controller poses in SAPIEN's right-handed
    ``x-forward, y-left, z-up`` frame.  Stage 2 deliberately accepts raw
    Unity-style ``x-right, y-up, z-forward`` samples and performs its own
    explicit Unity-to-robot basis change.  Applying the inverse basis here
    means the existing safety/calibration code recovers the original SAPIEN
    pose exactly once, without adding an unlabelled second convention.
    """

    if len(quaternion_wxyz) != 4:
        raise ValueError("quaternion_wxyz must contain four values")
    w, x, y, z = (float(value) for value in quaternion_wxyz)
    position_unity = transform_vector(position, ROBOT_TO_UNITY_BASIS)
    quaternion_unity = change_rotation_basis(
        (x, y, z, w),
        ROBOT_TO_UNITY_BASIS,
    )
    return RigidPose(position_unity, quaternion_unity)


def compact_controller_line(
    *,
    side: str,
    pose: RigidPose,
    grasp: float,
    clutch: bool,
    record: bool,
    frame_id: int,
    timestamp_ns: int,
) -> str:
    """Serialize one sample in the already-tested 10-float controller layout."""

    if side not in CONTROLLERS:
        raise ValueError("side must be left or right")
    values = (
        *pose.position,
        *pose.quaternion_xyzw,
        float(grasp),
        int(bool(clutch)),
        int(bool(record)),
    )
    payload = ", ".join(f"{value:.9g}" for value in values)
    return (
        f"{side.title()} controller | f = {int(frame_id)} | "
        f"t = {int(timestamp_ns)}:, {payload}"
    )


def alvr_is_streaming(session_path: str | Path) -> bool:
    """Return whether any client in ALVR's session file is streaming."""

    try:
        document = json.loads(Path(session_path).read_text(encoding="utf-8"))
        connections = document["client_connections"]
        entries = connections.values() if isinstance(connections, dict) else connections
        return any(entry.get("connection_state") == "Streaming" for entry in entries)
    except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
        return False


def _load_sapien() -> Any:
    try:
        import sapien
        import sapien.render
    except ImportError as exc:
        raise RuntimeError(
            "SAPIEN is unavailable. Start this module through "
            "scripts/start_meta_quest_steamvr_bridge.sh or set "
            "STEAMVR_PYTHON and SAPIEN_SITE_PACKAGES."
        ) from exc
    if not hasattr(sapien.render, "RenderVRDisplay"):
        raise RuntimeError("installed SAPIEN has no RenderVRDisplay support")
    return sapien


def _wait_for_alvr(session_path: Path) -> None:
    last_report = 0.0
    while not alvr_is_streaming(session_path):
        now = time.monotonic()
        if now - last_report >= 5.0:
            print(
                "Waiting for ALVR Streaming: open ALVR on Quest, wear/wake the "
                "headset, and confirm the desktop dashboard says Streaming.",
                flush=True,
            )
            last_report = now
        time.sleep(0.5)


def _trigger_value(display: Any, controller_id: int, mask: int) -> float:
    try:
        state = display.get_controller_axis_state(controller_id, 1)
        value = float(state[0])
        return min(1.0, max(0.0, value))
    except Exception:
        return 1.0 if mask & BUTTON_TRIGGER else 0.0


def run_bridge(args: argparse.Namespace) -> None:
    sapien = _load_sapien()
    if args.dry_run:
        print(
            "SteamVR bridge dry-run OK: SAPIEN RenderVRDisplay is available; "
            "robot_output_enabled=false"
        )
        return

    session_path = Path(args.alvr_session).expanduser()
    if not args.no_alvr_wait:
        _wait_for_alvr(session_path)

    # Keep OpenVR legacy controller input active. Registering an action
    # manifest suppresses the button-mask API used by SAPIEN/DexArena.
    shader_dir = os.path.join(
        os.path.dirname(sapien.__file__),
        "vulkan_shader",
        "vr_default",
    )
    sapien.render.set_viewer_shader_dir(shader_dir)
    sapien.render.enable_vr()

    scene = sapien.Scene()
    display = sapien.render.RenderVRDisplay()
    display.root_pose = sapien.Pose()
    display.set_scene(scene)

    selected = (
        CONTROLLERS.items()
        if args.side == "both"
        else ((args.side, CONTROLLERS[args.side]),)
    )
    print(
        f"Connecting SteamVR input to PoseReceiver at {args.host}:{args.port}; "
        "robot_output_enabled=false",
        flush=True,
    )
    with socket.create_connection((args.host, args.port), timeout=3.0) as output:
        output.settimeout(None)
        frame_id = 0
        sent = 0
        last_ids: tuple[int, ...] = ()
        report_start = time.monotonic()
        try:
            while True:
                scene.update_render()
                display.update_render()
                display.render()
                frame_id += 1
                ids = tuple(int(value) for value in display.get_controller_ids())
                if ids != last_ids:
                    print(
                        f"SteamVR controller ids: {last_ids} -> {ids} "
                        f"(expected left={CONTROLLER_LEFT}, right={CONTROLLER_RIGHT})",
                        flush=True,
                    )
                    last_ids = ids
                timestamp_ns = time.monotonic_ns()
                for side, controller_id in selected:
                    if controller_id not in ids:
                        continue
                    source_pose = display.get_controller_pose(controller_id)
                    pose = sapien_pose_to_unity(source_pose.p, source_pose.q)
                    mask = int(display.get_controller_button_pressed(controller_id))
                    line = compact_controller_line(
                        side=side,
                        pose=pose,
                        grasp=_trigger_value(display, controller_id, mask),
                        clutch=bool(mask & BUTTON_B),
                        record=bool(mask & BUTTON_A),
                        frame_id=frame_id,
                        timestamp_ns=timestamp_ns,
                    )
                    output.sendall(line.encode("utf-8") + b"\n")
                    sent += 1
                now = time.monotonic()
                if now - report_start >= args.report_interval_s:
                    elapsed = now - report_start
                    print(
                        f"SteamVR frames={frame_id} sent={sent} "
                        f"rate={sent / elapsed:.1f} pose/s ids={ids}",
                        flush=True,
                    )
                    sent = 0
                    report_start = now
        except KeyboardInterrupt:
            print("Stopping SteamVR pose bridge cleanly.", flush=True)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Relay ALVR/SteamVR controllers into the robot-disconnected "
            "Meta Quest PoseReceiver contract."
        )
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--side", choices=("both", "left", "right"), default="both")
    parser.add_argument(
        "--alvr-session",
        default="~/.config/alvr/session.json",
        help="ALVR session.json used to wait for Streaming state.",
    )
    parser.add_argument("--no-alvr-wait", action="store_true")
    parser.add_argument("--report-interval-s", type=float, default=5.0)
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate SAPIEN support without touching SteamVR or opening a socket.",
    )
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.report_interval_s <= 0.0:
        parser.error("--report-interval-s must be positive")
    return args


def main(argv: list[str] | None = None) -> None:
    run_bridge(parse_args(argv))


if __name__ == "__main__":
    main()
