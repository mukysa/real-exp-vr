"""Exercise diagnostics with fake VR interfaces; never initialize SteamVR."""

from types import SimpleNamespace

from data_collection.meta_quest import button_check


def _monitor(monkeypatch, *, slow_render=False, axis_error=False, invalid_pose=False):
    clock = SimpleNamespace(now=0.0)
    calls = []
    monkeypatch.setattr(button_check.time, "monotonic", lambda: clock.now)
    monkeypatch.setattr(button_check.time, "sleep", lambda seconds: setattr(clock, "now", clock.now + seconds))
    diagnostics = button_check._Diagnostics(0.25, 2.0)
    frame = 0

    def scene_update():
        nonlocal frame
        frame += 1
        calls.append("scene_update")

    def render():
        calls.append("render")
        if slow_render and frame == 2:
            clock.now += 2.5
            # Inspect an active call, before the timing wrapper returns.
            diagnostics._check_stall()

    def ids():
        calls.append("ids")
        return (1, 2)

    def buttons(controller):
        calls.append(f"buttons{controller}")
        return button_check.BUTTON_TRIGGER if axis_error else 0

    def axis(controller, index):
        calls.append(f"axis{controller}:{index}")
        if axis_error:
            raise RuntimeError("simulated axis failure")
        return (0.0, 0.0)

    display = SimpleNamespace(
        set_scene=lambda scene: None,
        update_render=lambda: calls.append("display_update"),
        render=render,
        get_controller_ids=ids,
        get_controller_button_pressed=buttons,
        get_controller_axis_state=axis,
        get_controller_pose=lambda controller: SimpleNamespace(
            p=(float("nan") if invalid_pose else 0.0, 0.0, 0.0), q=(1.0, 0.0, 0.0, 0.0)
        ),
    )
    sapien = SimpleNamespace(
        Scene=lambda: SimpleNamespace(update_render=scene_update),
        Pose=lambda: None,
        render=SimpleNamespace(RenderVRDisplay=lambda: display),
    )
    monkeypatch.setattr(button_check, "_load_sapien", lambda: sapien)
    monkeypatch.setattr(button_check, "_configure_sapien_vr", lambda _: False)
    button_check._monitor(5.1, 10.0, 2.0, invalid_pose, diagnostics)
    return calls


def test_reports_unchanged_zero_inputs_periodically_and_keeps_read_order(monkeypatch, capsys):
    calls = _monitor(monkeypatch)
    output = capsys.readouterr().out
    assert calls[:8] == [
        "scene_update", "display_update", "render", "ids",
        "buttons1", "axis1:1", "buttons2", "axis2:1",
    ]
    assert output.count("left: signal=OK") == 3
    assert output.count("right: signal=OK") == 3
    assert output.count("TIMING window_s=") == 3
    assert "trigger=0.00 trigger_source=axis1 axis_error='none'" in output
    assert "poll_sleep=100.0/100.0" in output


def test_identifies_active_render_stall_and_read_gaps(monkeypatch, capsys):
    _monitor(monkeypatch, slow_render=True)
    output = capsys.readouterr().out
    assert "STALL stage=render elapsed_ms=2500.0 call_not_returned=true" in output
    assert "SLOW stage=render duration_ms=2500.0 call_returned=true" in output
    assert "READ_GAP side=left gap_s=2.600" in output
    assert "READ_GAP side=right gap_s=2.600" in output
    assert "LOOP_GAP gap_s=2.600" in output


def test_axis_errors_are_visible_even_when_digital_fallback_returns_one(monkeypatch, capsys):
    _monitor(monkeypatch, axis_error=True)
    output = capsys.readouterr().out
    assert "left: signal=DEGRADED" in output
    assert "trigger=1.00 trigger_source=button_fallback" in output
    assert "RuntimeError: simulated axis failure" in output
    assert "left: signal=OK" not in output


def test_optional_pose_probe_exposes_nonfinite_values(monkeypatch, capsys):
    _monitor(monkeypatch, invalid_pose=True)
    output = capsys.readouterr().out
    assert "left: signal=DEGRADED" in output
    assert "pose=ERROR(ValueError: pose must contain seven finite values)" in output
