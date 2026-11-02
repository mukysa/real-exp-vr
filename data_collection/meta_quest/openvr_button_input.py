"""Controller-only OpenVR access without creating a renderer or compositor."""

from __future__ import annotations

from contextlib import contextmanager
import math
import time
from types import SimpleNamespace
from typing import Any, Iterator

from .openvr_bridge import _load_openvr, _physical_controller_id, _rotation_matrix_to_quaternion


class OpenVRButtonInput:
    """Expose the monitor's input interface using one coherent state per poll."""

    pose_frame = "openvr"

    def __init__(self, openvr: Any, system: Any, *, check_pose: bool) -> None:
        self.openvr = openvr
        self.system = system
        self.check_pose = check_pose
        self.controllers: dict[str, int] = {}
        self.input_available = False
        self._states: dict[int, Any] = {}
        self._poses: dict[int, Any] = {}
        self._packets: dict[int, tuple[int, float]] = {}

    def prepare_frame(self) -> None:
        self._states.clear()
        self._poses.clear()
        self.input_available = bool(self.system.isInputAvailable())

    def get_controller_ids(self) -> tuple[int, ...]:
        self.controllers = {
            side: _physical_controller_id(self.system, self.openvr, side)
            for side in ("left", "right")
        }
        ids = tuple(device_id for device_id in self.controllers.values()
                    if device_id != self.openvr.k_unTrackedDeviceIndexInvalid)
        self._packets = {key: value for key, value in self._packets.items() if key in ids}
        return ids

    def get_controller_button_pressed(self, controller_id: int) -> int:
        if self.check_pose:
            ok, state, pose = self.system.getControllerStateWithPose(
                self.openvr.TrackingUniverseStanding, controller_id
            )
            self._poses[controller_id] = pose
        else:
            ok, state = self.system.getControllerState(controller_id)
        if not ok:
            raise RuntimeError("OpenVR did not return controller state; check connection and input focus")
        self._states[controller_id] = state
        packet = int(state.unPacketNum)
        if self._packets.get(controller_id, (None, 0.0))[0] != packet:
            self._packets[controller_id] = (packet, time.monotonic())
        return int(state.ulButtonPressed)

    def get_controller_axis_state(self, controller_id: int, axis: int) -> tuple[float, float]:
        state = self._states[controller_id]
        return float(state.rAxis[axis].x), float(state.rAxis[axis].y)

    def get_controller_pose(self, controller_id: int) -> Any:
        pose = self._poses[controller_id]
        if not pose.bDeviceIsConnected or not pose.bPoseIsValid:
            raise ValueError("OpenVR controller pose is disconnected or invalid")
        matrix = pose.mDeviceToAbsoluteTracking.m
        position = tuple(float(matrix[i][3]) for i in range(3))
        if not all(math.isfinite(value) for value in position):
            raise ValueError("OpenVR position must be finite")
        x, y, z, w = _rotation_matrix_to_quaternion(
            tuple(tuple(matrix[i][j] for j in range(3)) for i in range(3))
        )
        return SimpleNamespace(p=position, q=(w, x, y, z))

    def input_details(self, controller_id: int) -> dict[str, Any]:
        packet, changed_at = self._packets[controller_id]
        return {
            "packet_num": packet,
            "packet_unchanged_s": round(time.monotonic() - changed_at, 3),
            "input_available": self.input_available,
            "pose_frame": self.pose_frame,
        }


@contextmanager
def openvr_button_input(*, check_pose: bool) -> Iterator[OpenVRButtonInput]:
    openvr = _load_openvr()
    # init() returns IVRSystem. Do not initialize again or request VRCompositor.
    system = openvr.init(openvr.VRApplication_Background)
    try:
        yield OpenVRButtonInput(openvr, system, check_pose=check_pose)
    finally:
        openvr.shutdown()
