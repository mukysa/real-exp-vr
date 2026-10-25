"""Relay live ALVR/SteamVR controller input into the stage-2 TCP contract.

Unlike :mod:`steamvr_bridge`, this adapter reads OpenVR directly and does not
create a SAPIEN render display.  That distinction matters on the lab client:
the installed SAPIEN 3.0.0b1 build caches controller input unless it submits
VR frames, while a background OpenVR application receives continuously fresh
poses without rendering anything into the headset.

This module only produces controller input records.  It has no ROS imports,
publisher, robot backend, or robot output.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import socket
import time
from typing import Any, Sequence

from .calibration import RigidPose
from .coordinates import change_rotation_basis, transform_vector
from .power import QuestKeepAwake
from .steamvr_bridge import (
    BUTTON_A,
    BUTTON_B,
    BUTTON_TRIGGER,
    alvr_is_streaming,
    compact_controller_line,
)

# SteamVR Touch legacy bindings map left X and right A to the A button bit.
BUTTON_X = BUTTON_A
# The legacy B-button bit is reported for left Y and right B.
BUTTON_Y = BUTTON_B


# OpenVR tracking space is right-handed: x right, y up, -z forward.  The
# existing raw-input contract is Unity-like: x right, y up, +z forward.
OPENVR_TO_UNITY_BASIS = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, -1.0),
)


def _rotation_matrix_to_quaternion(
    matrix: Sequence[Sequence[float]],
) -> tuple[float, float, float, float]:
    """Convert a slightly noisy 3x3 OpenVR matrix to normalized xyzw.

    OpenVR matrices measured on the rig differ from exact orthonormality by
    normal floating-point error.  The project's strict basis validator is
    intended for declared coordinate bases, so controller samples use this
    numerically tolerant conversion and let :class:`RigidPose` normalize the
    result.
    """

    rows = tuple(tuple(float(value) for value in row) for row in matrix)
    if len(rows) != 3 or any(len(row) != 3 for row in rows):
        raise ValueError("OpenVR rotation must be a 3x3 matrix")
    if not all(math.isfinite(value) for row in rows for value in row):
        raise ValueError("OpenVR rotation must contain finite values")

    trace = rows[0][0] + rows[1][1] + rows[2][2]
    if trace > 0.0:
        scale = math.sqrt(max(0.0, trace + 1.0)) * 2.0
        if scale < 1e-12:
            raise ValueError("OpenVR rotation matrix is degenerate")
        result = (
            (rows[2][1] - rows[1][2]) / scale,
            (rows[0][2] - rows[2][0]) / scale,
            (rows[1][0] - rows[0][1]) / scale,
            0.25 * scale,
        )
    elif rows[0][0] > rows[1][1] and rows[0][0] > rows[2][2]:
        scale = math.sqrt(max(0.0, 1.0 + rows[0][0] - rows[1][1] - rows[2][2])) * 2.0
        if scale < 1e-12:
            raise ValueError("OpenVR rotation matrix is degenerate")
        result = (
            0.25 * scale,
            (rows[0][1] + rows[1][0]) / scale,
            (rows[0][2] + rows[2][0]) / scale,
            (rows[2][1] - rows[1][2]) / scale,
        )
    elif rows[1][1] > rows[2][2]:
        scale = math.sqrt(max(0.0, 1.0 + rows[1][1] - rows[0][0] - rows[2][2])) * 2.0
        if scale < 1e-12:
            raise ValueError("OpenVR rotation matrix is degenerate")
        result = (
            (rows[0][1] + rows[1][0]) / scale,
            0.25 * scale,
            (rows[1][2] + rows[2][1]) / scale,
            (rows[0][2] - rows[2][0]) / scale,
        )
    else:
        scale = math.sqrt(max(0.0, 1.0 + rows[2][2] - rows[0][0] - rows[1][1])) * 2.0
        if scale < 1e-12:
            raise ValueError("OpenVR rotation matrix is degenerate")
        result = (
            (rows[0][2] + rows[2][0]) / scale,
            (rows[1][2] + rows[2][1]) / scale,
            0.25 * scale,
            (rows[1][0] - rows[0][1]) / scale,
        )
    return RigidPose((0.0, 0.0, 0.0), result).quaternion_xyzw


def openvr_matrix_to_unity_pose(matrix: Sequence[Sequence[float]]) -> RigidPose:
    """Convert an OpenVR 3x4 tracking matrix to the raw Unity-like contract."""

    rows = tuple(tuple(float(value) for value in row) for row in matrix)
    if len(rows) != 3 or any(len(row) != 4 for row in rows):
        raise ValueError("OpenVR pose must be a 3x4 matrix")
    if not all(math.isfinite(value) for row in rows for value in row):
        raise ValueError("OpenVR pose must contain finite values")
    position = transform_vector(
        (rows[0][3], rows[1][3], rows[2][3]),
        OPENVR_TO_UNITY_BASIS,
    )
    quaternion = _rotation_matrix_to_quaternion(tuple(row[:3] for row in rows))
    return RigidPose(
        position,
        change_rotation_basis(quaternion, OPENVR_TO_UNITY_BASIS),
    )


def _load_openvr() -> Any:
    try:
        import openvr
    except ImportError as exc:
        raise RuntimeError(
            "Python OpenVR bindings are unavailable. Run this module through "
            "scripts/start_meta_quest_openvr_bridge.sh and install openvr into "
            "OPENVR_SITE_PACKAGES."
        ) from exc
    return openvr


def _controller_controls(system: Any, controller_id: int, *, single_left: bool = False) -> tuple[float, bool, bool]:
    ok, state = system.getControllerState(controller_id)
    if not ok:
        return 0.0, False, False
    mask = int(state.ulButtonPressed)
    try:
        trigger = float(state.rAxis[1].x)
    except (AttributeError, IndexError, TypeError):
        trigger = 1.0 if mask & BUTTON_TRIGGER else 0.0
    trigger = min(1.0, max(0.0, trigger))
    return trigger, bool(mask & (BUTTON_Y if single_left else BUTTON_B)), bool(mask & BUTTON_A)


def _physical_controller_id(system: Any, openvr: Any, side: str) -> int:
    """Find the ALVR Touch controller, even if a virtual hand owns its role."""

    expected_serial_suffix = f"_Controller_{side.title()}"
    role = (
        openvr.TrackedControllerRole_LeftHand
        if side == "left"
        else openvr.TrackedControllerRole_RightHand
    )
    role_id = int(system.getTrackedDeviceIndexForControllerRole(role))

    def matches(device_id: int) -> bool:
        try:
            return (
                system.getTrackedDeviceClass(device_id)
                == openvr.TrackedDeviceClass_Controller
                and system.getStringTrackedDeviceProperty(
                    device_id, openvr.Prop_SerialNumber_String
                ).endswith(expected_serial_suffix)
            )
        except Exception:
            return False

    if role_id != openvr.k_unTrackedDeviceIndexInvalid and matches(role_id):
        return role_id
    for device_id in range(openvr.k_unMaxTrackedDeviceCount):
        if device_id != role_id and matches(device_id):
            return device_id
    return openvr.k_unTrackedDeviceIndexInvalid


def _wait_for_alvr(session_path: Path, power: QuestKeepAwake) -> None:
    last_report = 0.0
    while not alvr_is_streaming(session_path):
        power.update(True)
        now = time.monotonic()
        if now - last_report >= 5.0:
            print(
                "Waiting for ALVR Streaming: wear/wake the Quest and keep the "
                "ALVR client in the foreground.",
                flush=True,
            )
            last_report = now
        time.sleep(0.5)


def run_bridge(args: argparse.Namespace) -> None:
    openvr = _load_openvr()
    if args.dry_run:
        print("OpenVR bridge dry-run OK; robot_output_enabled=false")
        return

    session_path = Path(args.alvr_session).expanduser()
    if args.check_left_touch:
        if not alvr_is_streaming(session_path):
            raise SystemExit("ALVR is not Streaming; physical left Touch check failed.")
        openvr.init(openvr.VRApplication_Background)
        try:
            system = openvr.VRSystem()
            valid = 0
            controller_id = openvr.k_unTrackedDeviceIndexInvalid
            for _ in range(30):
                controller_id = _physical_controller_id(system, openvr, "left")
                poses = system.getDeviceToAbsoluteTrackingPose(
                    openvr.TrackingUniverseStanding,
                    0.0,
                    openvr.k_unMaxTrackedDeviceCount,
                )
                if controller_id != openvr.k_unTrackedDeviceIndexInvalid:
                    tracked = poses[controller_id]
                    valid += int(tracked.bDeviceIsConnected and tracked.bPoseIsValid)
                time.sleep(0.1)
            print(f"Physical left Touch: {valid}/30 valid OpenVR poses; id={controller_id}")
            if valid != 30:
                raise SystemExit("Physical left Touch tracking is not ready.")
        finally:
            openvr.shutdown()
        return

    power = QuestKeepAwake()
    openvr_initialized = False
    try:
        power.update(True)
        if not args.no_alvr_wait:
            _wait_for_alvr(session_path, power)

        openvr.init(openvr.VRApplication_Background)
        openvr_initialized = True
        system = openvr.VRSystem()
        selected = ("left", "right") if args.side == "both" else (args.side,)
        print(
            f"Connecting direct OpenVR input to PoseReceiver at {args.host}:{args.port}; "
            "robot_output_enabled=false",
            flush=True,
        )
        with socket.create_connection((args.host, args.port), timeout=3.0) as output:
            output.settimeout(None)
            frame_id = 0
            sent = 0
            report_start = time.monotonic()
            next_poll = report_start
            last_ids: dict[str, int] = {}
            last_clutch: dict[str, bool] = {}
            while True:
                power.update(True)
                if not args.no_alvr_wait and not alvr_is_streaming(session_path):
                    print(
                        "ALVR left Streaming state; stopping instead of emitting cached poses.",
                        flush=True,
                    )
                    return
                poses = system.getDeviceToAbsoluteTrackingPose(
                    openvr.TrackingUniverseStanding,
                    0.0,
                    openvr.k_unMaxTrackedDeviceCount,
                )
                frame_id += 1
                timestamp_ns = time.monotonic_ns()
                current_ids: dict[str, int] = {}
                for side in selected:
                    controller_id = _physical_controller_id(system, openvr, side)
                    if controller_id == openvr.k_unTrackedDeviceIndexInvalid:
                        continue
                    current_ids[side] = controller_id
                    tracked = poses[controller_id]
                    if not tracked.bDeviceIsConnected or not tracked.bPoseIsValid:
                        continue
                    pose = openvr_matrix_to_unity_pose(tracked.mDeviceToAbsoluteTracking.m)
                    if args.single_left and side != "left":
                        continue
                    grasp, clutch, record = _controller_controls(
                        system, controller_id, single_left=args.single_left
                    )
                    if args.single_left and last_clutch.get(side) != clutch:
                        print(f"Left Y clutch={'pressed' if clutch else 'released'}", flush=True)
                        last_clutch[side] = clutch
                    line = compact_controller_line(
                        side=side,
                        pose=pose,
                        grasp=grasp,
                        clutch=clutch,
                        record=record,
                        frame_id=frame_id,
                        timestamp_ns=timestamp_ns,
                    )
                    output.sendall(line.encode("utf-8") + b"\n")
                    sent += 1
                if current_ids != last_ids:
                    print(f"OpenVR controller ids: {last_ids} -> {current_ids}", flush=True)
                    last_ids = current_ids
                now = time.monotonic()
                if now - report_start >= args.report_interval_s:
                    elapsed = now - report_start
                    print(
                        f"OpenVR frames={frame_id} sent={sent} "
                        f"rate={sent / elapsed:.1f} pose/s ids={current_ids}",
                        flush=True,
                    )
                    sent = 0
                    report_start = now
                next_poll += 1.0 / args.poll_hz
                time.sleep(max(0.0, next_poll - time.monotonic()))
    except (BrokenPipeError, ConnectionResetError):
        print("PoseReceiver closed the connection; stopping OpenVR bridge.", flush=True)
    except KeyboardInterrupt:
        print("Stopping OpenVR pose bridge cleanly.", flush=True)
    finally:
        if openvr_initialized:
            openvr.shutdown()
        power.close()


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Relay live ALVR controllers through OpenVR without robot output."
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8000)
    parser.add_argument("--side", choices=("both", "left", "right"), default="both")
    parser.add_argument("--single-left", action="store_true", help="left controller only; use Y as clutch")
    parser.add_argument("--poll-hz", type=float, default=30.0)
    parser.add_argument("--report-interval-s", type=float, default=5.0)
    parser.add_argument("--alvr-session", default="~/.config/alvr/session.json")
    parser.add_argument("--no-alvr-wait", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--check-left-touch", action="store_true")
    args = parser.parse_args(argv)
    if args.single_left:
        args.side = "left"
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.poll_hz <= 0.0:
        parser.error("--poll-hz must be positive")
    if args.report_interval_s <= 0.0:
        parser.error("--report-interval-s must be positive")
    return args


def main(argv: list[str] | None = None) -> None:
    run_bridge(parse_args(argv))


if __name__ == "__main__":
    main()
