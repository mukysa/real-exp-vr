"""Verify input polling and teardown without starting any graphics services."""

from types import SimpleNamespace

import pytest

from data_collection.meta_quest import button_check, openvr_button_input as inputs


def fake_runtime(monkeypatch, *, available=True, valid_pose=True, read_error=False):
    calls = []
    axes = [SimpleNamespace(x=0.0, y=0.0), SimpleNamespace(x=0.73, y=0.0)]
    state = SimpleNamespace(ulButtonPressed=button_check.BUTTON_B, unPacketNum=42, rAxis=axes)
    pose = SimpleNamespace(
        bDeviceIsConnected=True, bPoseIsValid=valid_pose,
        mDeviceToAbsoluteTracking=SimpleNamespace(m=((1, 0, 0, 0.2),
                                                    (0, 1, 0, 0.3),
                                                    (0, 0, 1, -0.4))),
    )

    def read(device_id):
        calls.append(("state", device_id))
        if read_error:
            return False, state
        return True, state

    def read_pose(origin, device_id):
        ok, value = read(device_id)
        return ok, value, pose

    system = SimpleNamespace(isInputAvailable=lambda: available,
                             getControllerState=read, getControllerStateWithPose=read_pose)

    def init(app):
        calls.append(("init", app))
        return system

    runtime = SimpleNamespace(VRApplication_Background=3, TrackingUniverseStanding=1,
                              k_unTrackedDeviceIndexInvalid=4294967295,
                              init=init, shutdown=lambda: calls.append(("shutdown",)))
    monkeypatch.setattr(inputs, "_load_openvr", lambda: runtime)
    monkeypatch.setattr(inputs, "_physical_controller_id",
                        lambda system, runtime, side: {"left": 4, "right": 7}[side])
    # The fake deliberately has no VRCompositor, renderer, or Vulkan API.
    return calls, state, system


def test_one_background_init_and_shutdown_with_coherent_state(monkeypatch):
    calls, state, _ = fake_runtime(monkeypatch)
    with inputs.openvr_button_input(check_pose=False) as display:
        display.prepare_frame()
        assert display.get_controller_ids() == (4, 7)
        assert display.get_controller_button_pressed(4) == button_check.BUTTON_B
        assert display.get_controller_axis_state(4, 1) == (0.73, 0.0)
        assert display.input_details(4)["packet_num"] == 42
        assert display.controllers == {"left": 4, "right": 7}
    assert calls == [("init", 3), ("state", 4), ("shutdown",)]


@pytest.mark.parametrize("failure", [KeyboardInterrupt, RuntimeError])
def test_shutdown_is_balanced_when_scope_exits_with_exception(monkeypatch, failure):
    calls, _, _ = fake_runtime(monkeypatch)
    with pytest.raises(failure):
        with inputs.openvr_button_input(check_pose=False):
            raise failure()
    assert calls == [("init", 3), ("shutdown",)]


def test_failed_init_does_not_shutdown_uninitialized_runtime(monkeypatch):
    calls, _, _ = fake_runtime(monkeypatch)
    def fail(_):
        raise RuntimeError("init failed")
    runtime = inputs._load_openvr()
    runtime.init = fail
    with pytest.raises(RuntimeError, match="init failed"):
        with inputs.openvr_button_input(check_pose=False):
            pytest.fail("scope entered after failed init")
    assert calls == []


def test_optional_pose_uses_raw_openvr_coordinates(monkeypatch):
    fake_runtime(monkeypatch)
    with inputs.openvr_button_input(check_pose=True) as display:
        display.prepare_frame()
        display.get_controller_ids()
        display.get_controller_button_pressed(4)
        pose = display.get_controller_pose(4)
        assert pose.p == pytest.approx((0.2, 0.3, -0.4))
        assert pose.q == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_invalid_tracking_is_reported(monkeypatch):
    fake_runtime(monkeypatch, valid_pose=False)
    with inputs.openvr_button_input(check_pose=True) as display:
        display.prepare_frame()
        display.get_controller_ids()
        display.get_controller_button_pressed(4)
        with pytest.raises(ValueError, match="disconnected or invalid"):
            display.get_controller_pose(4)


def test_packet_counter_age_is_not_claimed_to_be_tracking_freshness(monkeypatch):
    _, state, _ = fake_runtime(monkeypatch)
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(inputs.time, "monotonic", lambda: clock.now)
    with inputs.openvr_button_input(check_pose=False) as display:
        display.prepare_frame()
        display.get_controller_ids()
        display.get_controller_button_pressed(4)
        clock.now = 2.0
        display.prepare_frame()
        display.get_controller_button_pressed(4)
        assert display.input_details(4)["packet_unchanged_s"] == 2.0
        state.unPacketNum = 43
        display.get_controller_button_pressed(4)
        assert display.input_details(4)["packet_unchanged_s"] == 0.0


def test_default_monitor_avoids_sapien_and_balances_shutdown(monkeypatch, capsys):
    calls, _, _ = fake_runtime(monkeypatch)
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(button_check.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(button_check.time, "sleep", lambda value: setattr(clock, "now", clock.now + value))
    monkeypatch.setattr(button_check, "_load_sapien", lambda: pytest.fail("SAPIEN imported"))
    button_check._monitor(0.15, 10.0, 2.0, False, button_check._Diagnostics(0.25, 2.0))
    output = capsys.readouterr().out
    assert "left: signal=OK" in output
    assert "packet_num=42" in output
    assert "side_ids={'left': 4, 'right': 7}" in output
    assert calls[0] == ("init", 3)
    assert calls[-1] == ("shutdown",)
    assert sum(call[0] == "init" for call in calls) == 1


def test_unavailable_focus_does_not_claim_healthy_input(monkeypatch, capsys):
    fake_runtime(monkeypatch, available=False)
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(button_check.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(button_check.time, "sleep", lambda value: setattr(clock, "now", clock.now + value))
    button_check._monitor(0.05, 10.0, 2.0, False, button_check._Diagnostics(0.25, 2.0))
    output = capsys.readouterr().out
    assert "left: signal=UNAVAILABLE" in output
    assert "left: signal=OK" not in output


def test_failed_controller_read_is_visible(monkeypatch, capsys):
    fake_runtime(monkeypatch, read_error=True)
    clock = SimpleNamespace(now=0.0)
    monkeypatch.setattr(button_check.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(button_check.time, "sleep", lambda value: setattr(clock, "now", clock.now + value))
    button_check._monitor(0.05, 10.0, 2.0, False, button_check._Diagnostics(0.25, 2.0))
    assert "left: signal=ERROR" in capsys.readouterr().out


def test_ctrl_c_during_polling_shuts_down_openvr_once(monkeypatch):
    calls, _, _ = fake_runtime(monkeypatch)
    def interrupt(_):
        raise KeyboardInterrupt()
    monkeypatch.setattr(button_check.time, "sleep", interrupt)
    with pytest.raises(KeyboardInterrupt):
        button_check._monitor(0.0, 60.0, 2.0, False, button_check._Diagnostics(0.25, 2.0))
    assert calls.count(("init", 3)) == 1
    assert calls.count(("shutdown",)) == 1


def test_cli_defaults_to_openvr_and_retains_sapien_option(monkeypatch):
    from unittest.mock import Mock
    run = Mock()
    monkeypatch.setattr(button_check, "run", run)
    button_check.main([])
    assert run.call_args.kwargs["backend"] == "openvr"
    button_check.main(["--backend", "sapien"])
    assert run.call_args.kwargs["backend"] == "sapien"
