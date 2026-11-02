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
import json
import math
from pathlib import Path
import socket
import threading
import time
from typing import Any, Sequence

from .calibration import RigidPose
from .coordinates import change_rotation_basis, transform_vector
from .power import QuestKeepAwake
from .tracking_reference import POSE_REFERENCES, TrackingStartReference, pose_dict
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


def openvr_matrix_to_pose(matrix: Sequence[Sequence[float]]) -> RigidPose:
    """Read a finite tracking matrix in the original OpenVR standing axes."""
    rows = tuple(tuple(float(value) for value in row) for row in matrix)
    if len(rows) != 3 or any(len(row) != 4 for row in rows):
        raise ValueError("OpenVR pose must be a 3x4 matrix")
    if not all(math.isfinite(value) for row in rows for value in row):
        raise ValueError("OpenVR pose must contain finite values")
    quaternion = _rotation_matrix_to_quaternion(tuple(row[:3] for row in rows))
    return RigidPose(tuple(row[3] for row in rows), quaternion)


def _openvr_pose_to_unity(pose: RigidPose) -> RigidPose:
    return RigidPose(
        transform_vector(pose.position, OPENVR_TO_UNITY_BASIS),
        change_rotation_basis(pose.quaternion_xyzw, OPENVR_TO_UNITY_BASIS),
    )


def openvr_matrix_to_unity_pose(matrix: Sequence[Sequence[float]]) -> RigidPose:
    """Convert an OpenVR 3x4 tracking matrix to the Unity-like contract."""
    return _openvr_pose_to_unity(openvr_matrix_to_pose(matrix))


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
        raise RuntimeError("OpenVR controller state unavailable; refusing neutral button fallback")
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


def _quest_headset_id(system: Any, openvr: Any, serial: str | None = None) -> int:
    """Find the physical Quest even when index zero is SteamVR's null HMD."""
    serials = {serial} if serial else set()
    if not serial:
        for side in ("left", "right"):
            controller_id = _physical_controller_id(system, openvr, side)
            if controller_id != openvr.k_unTrackedDeviceIndexInvalid:
                value = system.getStringTrackedDeviceProperty(
                    controller_id, openvr.Prop_SerialNumber_String)
                serials.add(value.removesuffix(f"_Controller_{side.title()}"))
    matches = []
    for device_id in range(openvr.k_unMaxTrackedDeviceCount):
        try:
            device_serial = system.getStringTrackedDeviceProperty(
                device_id, openvr.Prop_SerialNumber_String)
            # An explicit serial override must not select SteamVR's null HMD.
            if device_serial == "Null Serial Number":
                continue
            if (system.getTrackedDeviceClass(device_id) in
                    (openvr.TrackedDeviceClass_HMD, openvr.TrackedDeviceClass_TrackingReference)
                    and device_serial in serials):
                matches.append(device_id)
        except Exception:
            continue
    return matches[0] if len(matches) == 1 else openvr.k_unTrackedDeviceIndexInvalid


def _tracked_pose(poses: Any, device_id: int, openvr: Any) -> RigidPose | None:
    if device_id == openvr.k_unTrackedDeviceIndexInvalid:
        return None
    tracked = poses[device_id]
    if not tracked.bDeviceIsConnected or not tracked.bPoseIsValid:
        return None
    try:
        return openvr_matrix_to_pose(tracked.mDeviceToAbsoluteTracking.m)
    except (ValueError, TypeError, IndexError):
        return None


def _reference_reset_event(system: Any, openvr: Any) -> int | None:
    """Drain notifications without accessing VRCompositor."""
    resets = {openvr.VREvent_StandingZeroPoseReset,
              openvr.VREvent_ChaperoneUniverseHasChanged,
              openvr.VREvent_ChaperoneRoomSetupCommitted}
    event = openvr.VREvent_t()
    while system.pollNextEvent(event):
        if int(event.eventType) in resets:
            return int(event.eventType)
    return None


def _write_reference(path: str | None, reference: TrackingStartReference,
                     system: Any, openvr: Any, headset_id: int) -> None:
    assert reference.headset_start is not None
    metadata = {
        "pose_reference": reference.mode,
        "tracking_universe": "OpenVR Standing",
        "source_axes": "right-handed: x right, y up, -z forward",
        "output_axes": "Unity-like: x right, y up, +z forward",
        "captured_monotonic_ns": reference.captured_monotonic_ns,
        "headset_id": headset_id,
        "headset_serial": system.getStringTrackedDeviceProperty(
            headset_id, openvr.Prop_SerialNumber_String),
        "standing_from_headset_start": pose_dict(reference.headset_start),
        "reference_from_standing": pose_dict(reference.reference_from_standing),
        "frozen_until_process_exit": True,
    }
    print("HEADSET_START_REFERENCE " + json.dumps(metadata, sort_keys=True), flush=True)
    if path:
        target = Path(path).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("x", encoding="utf-8") as output:
            json.dump(metadata, output, indent=2, sort_keys=True)
            output.write("\n")


class _AsyncKeepAwake:
    """Keep ADB startup and retries off the OpenVR pose polling thread."""

    def __init__(self, power: QuestKeepAwake, retry_interval_s: float = 30.0) -> None:
        self._power = power
        self._retry_interval_s = retry_interval_s
        self._last_attempt_s = float("-inf")
        self._thread: threading.Thread | None = None

    def update(self) -> None:
        if self._power.active or (self._thread is not None and self._thread.is_alive()):
            return
        now = time.monotonic()
        if now - self._last_attempt_s < self._retry_interval_s:
            return
        self._last_attempt_s = now
        self._thread = threading.Thread(
            target=self._power.update, args=(True,), daemon=True, name="quest-keep-awake"
        )
        self._thread.start()

    def close(self) -> None:
        if self._thread is not None:
            self._thread.join(timeout=4.0)
        self._power.close()


def _wait_for_alvr(session_path: Path, power: _AsyncKeepAwake) -> None:
    last_report = 0.0
    while not alvr_is_streaming(session_path):
        power.update()
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
    if args.check_left_touch or args.check_both_touch:
        deadline = time.monotonic() + args.touch_wait_s
        while not alvr_is_streaming(session_path):
            if time.monotonic() >= deadline:
                raise SystemExit("ALVR is not Streaming; physical Touch check failed.")
            print("Waiting for ALVR Streaming before Touch check...", flush=True)
            time.sleep(min(1.0, max(0.0, deadline - time.monotonic())))
        system = openvr.init(openvr.VRApplication_Background)
        try:
            sides = ("left", "right") if args.check_both_touch else ("left",)
            reference = TrackingStartReference(args.pose_reference, stable_s=args.reference_stable_s)
            while True:
                valid = {side: 0 for side in sides}
                headset_valid = 0
                controller_ids = {side: openvr.k_unTrackedDeviceIndexInvalid for side in sides}
                for _ in range(30):
                    poses = system.getDeviceToAbsoluteTrackingPose(
                        openvr.TrackingUniverseStanding,
                        0.0,
                        openvr.k_unMaxTrackedDeviceCount,
                    )
                    if args.pose_reference != "standing":
                        headset_id = _quest_headset_id(system, openvr, args.headset_serial)
                        head_pose = _tracked_pose(poses, headset_id, openvr)
                        headset_valid += int(head_pose is not None)
                        reference.observe(head_pose, time.monotonic_ns())
                    for side in sides:
                        controller_id = _physical_controller_id(system, openvr, side)
                        controller_ids[side] = controller_id
                        if controller_id != openvr.k_unTrackedDeviceIndexInvalid:
                            tracked = poses[controller_id]
                            if args.hand_arm and side == "left":
                                valid[side] += int(tracked.bDeviceIsConnected and tracked.bPoseIsValid)
                                continue
                            if args.hand_arm and side == "right" and tracked.bDeviceIsConnected:
                                try:
                                    _controller_controls(system, controller_id)
                                except RuntimeError:
                                    continue
                                valid[side] += int(bool(system.isInputAvailable()))
                                continue
                            if tracked.bDeviceIsConnected and tracked.bPoseIsValid:
                                try:
                                    _controller_controls(system, controller_id)
                                except RuntimeError:
                                    continue
                                valid[side] += int(bool(system.isInputAvailable()))
                    time.sleep(0.1)
                for side in sides:
                    print(
                        f"Physical {side} Touch: {valid[side]}/30 valid "
                        f"{'button reads' if args.hand_arm and side == 'right' else 'poses' if args.hand_arm else 'poses and button reads'}; "
                        f"id={controller_ids[side]}",
                        flush=True,
                    )
                if args.pose_reference != "standing":
                    print(f"Physical Quest headset: {headset_valid}/30 valid poses; "
                          f"stable_start_reference={reference.ready}", flush=True)
                if (all(valid[side] == 30 for side in sides) and reference.ready
                        and (args.pose_reference == "standing" or headset_valid == 30)):
                    if args.pose_reference != "standing":
                        _write_reference(args.reference_file, reference, system, openvr, headset_id)
                    break
                if time.monotonic() >= deadline:
                    raise SystemExit("Physical Touch input/headset reference not ready after waiting.")
                waiting_for = " and ".join(sides)
                print(
                    f"Waiting for physical {waiting_for} Touch tracking to reach 30/30...",
                    flush=True,
                )
        finally:
            openvr.shutdown()
        return

    power = _AsyncKeepAwake(QuestKeepAwake())
    openvr_initialized = False
    try:
        power.update()
        if not args.no_alvr_wait:
            _wait_for_alvr(session_path, power)

        system = openvr.init(openvr.VRApplication_Background)
        openvr_initialized = True
        reference = TrackingStartReference(args.pose_reference, stable_s=args.reference_stable_s)
        reference_deadline = time.monotonic() + args.reference_wait_s
        reference_headset_id = None
        reference_headset_serial = None
        last_reference_report = float("-inf")
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
            hand_arm_ids: dict[str, int] = {}
            while True:
                power.update()
                if not args.no_alvr_wait and not alvr_is_streaming(session_path):
                    print(
                        "ALVR left Streaming state; stopping instead of emitting cached poses.",
                        flush=True,
                    )
                    return
                reset_event = _reference_reset_event(system, openvr)
                if reset_event is not None and frame_id:
                    raise RuntimeError(
                        f"OpenVR tracking origin changed (event {reset_event}); "
                        "restart and capture a new headset reference before control")
                poses = system.getDeviceToAbsoluteTrackingPose(
                    openvr.TrackingUniverseStanding,
                    0.0,
                    openvr.k_unMaxTrackedDeviceCount,
                )
                frame_id += 1
                timestamp_ns = time.monotonic_ns()
                if args.pose_reference != "standing":
                    if reference.ready:
                        headset_id = reference_headset_id
                        if system.getStringTrackedDeviceProperty(
                                headset_id, openvr.Prop_SerialNumber_String) != reference_headset_serial:
                            raise RuntimeError("Physical Quest headset identity changed; restart reference capture")
                    else:
                        headset_id = _quest_headset_id(system, openvr, args.headset_serial)
                    head_pose = _tracked_pose(poses, headset_id, openvr)
                    if reference.observe(head_pose, timestamp_ns):
                        reference_headset_id = headset_id
                        reference_headset_serial = system.getStringTrackedDeviceProperty(
                            headset_id, openvr.Prop_SerialNumber_String)
                        _write_reference(args.reference_file, reference, system, openvr, headset_id)
                    if not reference.ready:
                        if time.monotonic() >= reference_deadline:
                            raise RuntimeError("No stable physical Quest headset startup pose; no input sent")
                        if time.monotonic() - last_reference_report >= 5.0:
                            print("Waiting for stable physical Quest headset pose; hold head still. "
                                  "Null HMD is never used as the reference.", flush=True)
                            last_reference_report = time.monotonic()
                        time.sleep(1.0 / args.poll_hz)
                        next_poll = time.monotonic()
                        continue
                    if head_pose is None:
                        raise RuntimeError("Physical Quest headset tracking lost; frozen reference retained, stream stopped")
                if not system.isInputAvailable():
                    raise RuntimeError("OpenVR input unavailable; dismiss dashboard and verify bindings")
                current_ids: dict[str, int] = {}
                for side in selected:
                    controller_id = _physical_controller_id(system, openvr, side)
                    if controller_id == openvr.k_unTrackedDeviceIndexInvalid:
                        continue
                    current_ids[side] = controller_id
                    if args.hand_arm:
                        previous_id = hand_arm_ids.setdefault(side, controller_id)
                        if previous_id != controller_id:
                            raise RuntimeError("Hand-arm controller identity changed; restart required")
                        if side == "right":
                            if poses[controller_id].bDeviceIsConnected:
                                grasp, clutch, record = _controller_controls(system, controller_id)
                                line = (f"right buttons|f={frame_id}|t={timestamp_ns}:"
                                        f"{grasp:.9g},{int(clutch)},{int(record)}")
                                output.sendall(line.encode("utf-8") + b"\n")
                                sent += 1
                            continue
                    tracked = poses[controller_id]
                    if not tracked.bDeviceIsConnected or not tracked.bPoseIsValid:
                        continue
                    pose = _openvr_pose_to_unity(reference.apply(
                        openvr_matrix_to_pose(tracked.mDeviceToAbsoluteTracking.m)))
                    if args.single_left and side != "left":
                        continue
                    left_y_button = args.single_left or (args.global_left_y and side == "left")
                    if args.hand_arm:
                        grasp, clutch, record = 0.0, False, False
                    else:
                        grasp, clutch, record = _controller_controls(
                            system, controller_id, single_left=left_y_button
                        )
                    if left_y_button and last_clutch.get(side) != clutch:
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
    parser.add_argument("--global-left-y", action="store_true", help="relay both controllers; left Y is the control toggle")
    parser.add_argument("--hand-arm", action="store_true", help="left pose; independent right B/A/trigger")
    parser.add_argument("--poll-hz", type=float, default=30.0)
    parser.add_argument("--report-interval-s", type=float, default=5.0)
    parser.add_argument("--alvr-session", default="~/.config/alvr/session.json")
    parser.add_argument("--no-alvr-wait", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--pose-reference", choices=POSE_REFERENCES, default="standing",
                        help="standing: unchanged; headset-start-position: frozen origin, standing axes; "
                             "headset-start: frozen full pose (requires axis calibration)")
    parser.add_argument("--headset-serial", help="physical ALVR headset serial; otherwise derive from Touch serials")
    parser.add_argument("--reference-file", help="write new startup-reference JSON; refuse overwriting an existing file")
    parser.add_argument("--reference-wait-s", type=float, default=30.0)
    parser.add_argument("--reference-stable-s", type=float, default=0.5)
    parser.add_argument("--check-left-touch", action="store_true")
    parser.add_argument("--check-both-touch", action="store_true")
    parser.add_argument(
        "--touch-wait-s", type=float, default=0.0,
        help="wait up to this many seconds for a full 30/30 physical Touch window",
    )
    args = parser.parse_args(argv)
    if args.hand_arm and (args.single_left or args.global_left_y):
        parser.error("--hand-arm cannot be combined with legacy control profiles")
    if args.hand_arm:
        args.side = "both"
    if args.single_left and args.global_left_y:
        parser.error("--single-left and --global-left-y are mutually exclusive")
    if args.single_left:
        args.side = "left"
    if args.global_left_y:
        args.side = "both"
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not math.isfinite(args.poll_hz) or args.poll_hz <= 0.0:
        parser.error("--poll-hz must be positive")
    if not math.isfinite(args.report_interval_s) or args.report_interval_s <= 0.0:
        parser.error("--report-interval-s must be positive")
    if not 0.0 <= args.touch_wait_s <= 300.0:
        parser.error("--touch-wait-s must be between 0 and 300")
    if not math.isfinite(args.reference_wait_s) or args.reference_wait_s <= 0.0:
        parser.error("--reference-wait-s must be finite and positive")
    if not math.isfinite(args.reference_stable_s) or not 0.0 <= args.reference_stable_s < args.reference_wait_s:
        parser.error("--reference-stable-s must be finite, nonnegative and less than reference wait")
    if args.reference_file and args.pose_reference == "standing":
        parser.error("--reference-file requires a headset-start reference mode")
    return args


def main(argv: list[str] | None = None) -> None:
    run_bridge(parse_args(argv))


if __name__ == "__main__":
    main()
