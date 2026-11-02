"""Verify shared startup transforms and the video-free bridge lifecycle."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from data_collection.meta_quest import openvr_bridge as bridge
from data_collection.meta_quest.calibration import RigidPose, compose
from data_collection.meta_quest.coordinates import quaternion_to_matrix
from data_collection.meta_quest.tracking_reference import TrackingStartReference


def pose(position=(0.0, 0.0, 0.0), yaw=0.0):
    return RigidPose(position, (0.0, math.sin(yaw / 2), 0.0, math.cos(yaw / 2)))


def test_fixed_position_reference_preserves_axes_and_shared_hand_separation():
    ref = TrackingStartReference("headset-start-position", stable_s=0)
    head = pose((1.0, 1.7, -2.0), math.pi / 2)
    assert ref.observe(head, 100)
    left = pose((0.7, 1.2, -2.4), 0.2)
    right = pose((1.3, 1.2, -2.4), -0.2)
    assert ref.apply(head).position == pytest.approx((0, 0, 0))
    assert ref.apply(left).position == pytest.approx((-0.3, -0.5, -0.4))
    assert ref.apply(left).quaternion_xyzw == pytest.approx(left.quaternion_xyzw)
    delta = tuple(b - a for a, b in zip(ref.apply(left).position, ref.apply(right).position))
    assert delta == pytest.approx((0.6, 0, 0))
    assert not ref.observe(pose((9.0, 4.0, 8.0), -1.0), 200)
    assert ref.headset_start == head
    assert ref.apply(left).position == pytest.approx((-0.3, -0.5, -0.4))


def test_full_pose_reference_applies_inverse_rigid_transform():
    ref = TrackingStartReference("headset-start", stable_s=0)
    head = pose((1.0, 1.7, -2.0), math.pi / 2)
    relative = pose((0.4, -0.3, -0.5), -0.4)
    ref.observe(head, 1)
    result = ref.apply(compose(head, relative))
    assert result.position == pytest.approx(relative.position)
    assert result.quaternion_xyzw == pytest.approx(relative.quaternion_xyzw)
    assert ref.apply(head).quaternion_xyzw == pytest.approx((0, 0, 0, 1))


def test_reference_waits_for_stability_and_resets_on_invalid_tracking():
    ref = TrackingStartReference("headset-start-position", stable_s=0.5)
    assert not ref.observe(pose(), 0)
    assert not ref.observe(pose((0.1, 0, 0)), 400_000_000)
    assert not ref.observe(None, 800_000_000)
    assert not ref.observe(pose(), 900_000_000)
    assert not ref.observe(pose(), 1_399_000_000)
    assert ref.observe(pose(), 1_400_000_000)
    assert ref.captured_monotonic_ns == 1_400_000_000


def test_position_reference_keeps_existing_robot_takeover_deltas():
    from data_collection.meta_quest.coordinates import unity_to_robot_pose
    ref = TrackingStartReference("headset-start-position", stable_s=0)
    ref.observe(pose((0.8, 1.6, -0.3), 0.4), 0)
    start, moved = pose((0.2, 1, -0.5), 0.1), pose((0.4, 0.9, -0.8), 0.5)
    original = [unity_to_robot_pose(bridge._openvr_pose_to_unity(p)) for p in (start, moved)]
    anchored = [unity_to_robot_pose(bridge._openvr_pose_to_unity(ref.apply(p)))
                for p in (start, moved)]
    assert tuple(b - a for a, b in zip(anchored[0].position, anchored[1].position)) == (
        pytest.approx(tuple(b - a for a, b in zip(original[0].position, original[1].position))))
    for old, new in zip(original, anchored):
        assert new.quaternion_xyzw == pytest.approx(old.quaternion_xyzw)


def test_reference_stability_checks_rotation_and_requires_capture():
    ref = TrackingStartReference("headset-start-position", stable_s=0.5)
    with pytest.raises(RuntimeError, match="not been captured"):
        ref.apply(pose())
    ref.observe(pose(), 0)
    assert not ref.observe(pose(yaw=0.3), 500_000_000)
    assert ref.observe(pose(yaw=0.3), 1_000_000_000)
    standing = TrackingStartReference("standing")
    assert standing.ready
    assert standing.apply(pose((1, 2, 3))) == pose((1, 2, 3))


def runtime(monkeypatch, *, failure="receiver"):
    calls, sent = [], []
    clock = SimpleNamespace(now=0.0)
    state = SimpleNamespace(ulButtonPressed=2, rAxis=[SimpleNamespace(x=0)] * 2)
    serials = {0: "Null Serial Number", 1: "QUEST",
               2: "QUEST_Controller_Left", 3: "QUEST_Controller_Right"}
    classes = {0: 1, 1: 4, 2: 2, 3: 2}
    frame = SimpleNamespace(number=0)

    def tracked(position, *, valid=True):
        return SimpleNamespace(bDeviceIsConnected=True, bPoseIsValid=valid,
            mDeviceToAbsoluteTracking=SimpleNamespace(m=(
                (1, 0, 0, position[0]), (0, 1, 0, position[1]), (0, 0, 1, position[2]))))

    def poses(origin, prediction, count):
        calls.append(("poses", origin, prediction))
        frame.number += 1
        if failure == "interrupt":
            raise KeyboardInterrupt
        head = (1, 1.7, -2) if frame.number == 1 else (5, 2, -7)
        return [tracked((0, 0, 0)), tracked(head, valid=(failure != "no_head" and
                not (failure == "lost_head" and frame.number > 1))),
                tracked((0.7, 1.2, -2.4)), tracked((1.3, 1.2, -2.4))]

    def read(device_id):
        return failure != "button", state

    def poll(event):
        if failure == "reset" and frame.number:
            event.eventType = 808
            return True
        return False

    system = SimpleNamespace(
        getTrackedDeviceIndexForControllerRole=lambda role: {1: 2, 2: 3}[role],
        getTrackedDeviceClass=lambda i: classes.get(i, 0),
        getStringTrackedDeviceProperty=lambda i, prop: serials[i],
        getDeviceToAbsoluteTrackingPose=poses, getControllerState=read,
        isInputAvailable=lambda: failure != "focus", pollNextEvent=poll)
    vr = SimpleNamespace(VRApplication_Background=3, TrackingUniverseStanding=1,
        TrackedControllerRole_LeftHand=1, TrackedControllerRole_RightHand=2,
        TrackedDeviceClass_HMD=1, TrackedDeviceClass_Controller=2,
        TrackedDeviceClass_TrackingReference=4, Prop_SerialNumber_String=100,
        k_unTrackedDeviceIndexInvalid=4294967295, k_unMaxTrackedDeviceCount=4,
        VREvent_StandingZeroPoseReset=808, VREvent_ChaperoneUniverseHasChanged=801,
        VREvent_ChaperoneRoomSetupCommitted=807, VREvent_t=lambda: SimpleNamespace(eventType=0))
    vr.init = lambda app: (calls.append(("init", app)) or system)
    vr.shutdown = lambda: calls.append(("shutdown",))

    class Connection:
        def __enter__(self):
            calls.append(("connect",))
            return self

        def __exit__(self, *args):
            calls.append(("disconnect",))

        def settimeout(self, value):
            pass

        def sendall(self, data):
            if failure == "receiver" and len(sent) >= 4:
                raise BrokenPipeError
            sent.append(data.decode())

    monkeypatch.setattr(bridge, "_load_openvr", lambda: vr)
    monkeypatch.setattr(bridge, "QuestKeepAwake", lambda: SimpleNamespace(
        active=True, close=lambda: calls.append(("power_close",))))
    monkeypatch.setattr(bridge.socket, "create_connection", lambda *a, **k: Connection())
    monkeypatch.setattr(bridge.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(bridge.time, "monotonic_ns", lambda: int(clock.now * 1e9))
    monkeypatch.setattr(bridge.time, "sleep", lambda n: setattr(clock, "now", clock.now + n))
    return vr, system, calls, sent


def test_headset_discovery_ignores_null_hmd_and_handles_tracking_reference(monkeypatch):
    vr, system, _, _ = runtime(monkeypatch)
    assert bridge._quest_headset_id(system, vr) == 1
    assert bridge._quest_headset_id(system, vr, "QUEST") == 1
    assert bridge._quest_headset_id(system, vr, "Null Serial Number") == vr.k_unTrackedDeviceIndexInvalid
    assert bridge._quest_headset_id(system, vr, "MISSING") == vr.k_unTrackedDeviceIndexInvalid
    original = system.getStringTrackedDeviceProperty
    system.getStringTrackedDeviceProperty = lambda i, p: "OTHER" if i == 1 else original(i, p)
    assert bridge._quest_headset_id(system, vr) == vr.k_unTrackedDeviceIndexInvalid


def test_video_free_bridge_freezes_origin_writes_metadata_and_cleans_up(monkeypatch, tmp_path):
    _, _, calls, sent = runtime(monkeypatch)
    target = tmp_path / "reference.json"
    args = bridge.parse_args(["--global-left-y", "--no-alvr-wait",
        "--pose-reference", "headset-start-position", "--reference-stable-s", "0",
        "--reference-file", str(target)])
    bridge.run_bridge(args)
    assert len(sent) == 4
    left = [float(v) for v in sent[0].rsplit(":", 1)[1].split(",") if v.strip()]
    assert left[:3] == pytest.approx((-0.3, -0.5, 0.4))
    assert sent[0].split(":", 1)[1] == sent[2].split(":", 1)[1]
    assert calls.count(("init", 3)) == calls.count(("shutdown",)) == 1
    assert calls[-2:] == [("shutdown",), ("power_close",)]
    metadata = json.loads(target.read_text())
    assert metadata["headset_id"] == 1
    assert metadata["headset_serial"] == "QUEST"
    assert metadata["standing_from_headset_start"]["position_m"] == [1, 1.7, -2]
    assert metadata["frozen_until_process_exit"] is True
    # The fake runtime deliberately has no compositor, renderer or Vulkan API.


@pytest.mark.parametrize("failure,message", [
    ("no_head", "No stable physical Quest"),
    ("lost_head", "tracking lost"),
    ("button", "controller state unavailable"),
    ("focus", "input unavailable"),
    ("reset", "tracking origin changed"),
])
def test_bridge_stops_on_invalid_tracking_input_or_origin_change(monkeypatch, failure, message):
    _, _, calls, sent = runtime(monkeypatch, failure=failure)
    args = bridge.parse_args(["--no-alvr-wait", "--pose-reference", "headset-start-position",
        "--reference-stable-s", "0", "--reference-wait-s", "0.1"])
    with pytest.raises(RuntimeError, match=message):
        bridge.run_bridge(args)
    assert calls.count(("shutdown",)) == 1
    assert len(sent) == (2 if failure in {"lost_head", "reset"} else 0)


def test_bridge_interrupt_balances_shutdown(monkeypatch):
    _, _, calls, _ = runtime(monkeypatch, failure="interrupt")
    bridge.run_bridge(bridge.parse_args(["--no-alvr-wait"]))
    assert calls.count(("shutdown",)) == 1


def test_existing_reference_file_is_preserved_and_shutdown_balanced(monkeypatch, tmp_path):
    _, _, calls, sent = runtime(monkeypatch)
    path = tmp_path / "reference.json"
    path.write_text("existing anchor")
    with pytest.raises(FileExistsError):
        bridge.run_bridge(bridge.parse_args(["--no-alvr-wait", "--pose-reference",
            "headset-start-position", "--reference-stable-s", "0", "--reference-file", str(path)]))
    assert not sent
    assert path.read_text() == "existing anchor"
    assert calls.count(("shutdown",)) == 1


def test_preflight_checks_headset_and_button_reads(monkeypatch, tmp_path):
    _, _, calls, sent = runtime(monkeypatch, failure="healthy")
    monkeypatch.setattr(bridge, "alvr_is_streaming", lambda _: True)
    bridge.run_bridge(bridge.parse_args(["--check-both-touch", "--pose-reference",
        "headset-start-position", "--reference-file", str(tmp_path / "check.json")]))
    assert not sent
    assert ("connect",) not in calls
    assert calls.count(("shutdown",)) == 1


def test_preflight_rejects_missing_headset_and_invalid_buttons(monkeypatch):
    for failure in ("no_head", "button"):
        _, _, calls, _ = runtime(monkeypatch, failure=failure)
        monkeypatch.setattr(bridge, "alvr_is_streaming", lambda _: True)
        with pytest.raises(SystemExit, match="not ready"):
            bridge.run_bridge(bridge.parse_args(["--check-left-touch", "--pose-reference",
                "headset-start-position", "--touch-wait-s", "0"]))
        assert calls.count(("shutdown",)) == 1


def test_headset_reference_options_reject_nonfinite_or_unsafe_values():
    for options in (["--reference-wait-s", "nan"], ["--reference-stable-s", "-1"],
                    ["--poll-hz", "inf"], ["--reference-file", "ref.json"]):
        with pytest.raises(SystemExit):
            bridge.parse_args(options)


def test_hardware_launcher_uses_same_video_free_backend_and_origin():
    script = (Path(__file__).resolve().parents[1] / "scripts/start_quest3_hardware.sh").read_text()
    assert "start_meta_quest_steamvr_bridge.sh" not in script
    assert "pose_reference=standing" in script
    assert "--pose-reference=headset-start-position" in script
    assert "--reference-file" in script
