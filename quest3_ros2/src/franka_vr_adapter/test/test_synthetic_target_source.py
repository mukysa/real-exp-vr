from __future__ import annotations

import math

import json

from franka_vr_adapter.adapter_node import JsonlTraceWriter, SyntheticTargetSource


Q0 = (0.0, -0.4, 0.0, -1.8, 0.0, 1.5, 0.7)


def test_first_synthetic_target_equals_measured_anchor() -> None:
    source = SyntheticTargetSource(amplitude_rad=0.02, period_s=8.0)
    source.set_anchor(Q0, 10.0)

    result = source.make_result(10.0)

    assert result is not None
    assert result.positions == Q0
    assert result.source_sequence == 1


def test_synthetic_motion_is_small_and_finite() -> None:
    source = SyntheticTargetSource(amplitude_rad=0.02, period_s=8.0)
    source.set_anchor(Q0, 10.0)
    source.make_result(10.0)

    result = source.make_result(14.0)

    assert result is not None
    assert all(math.isfinite(value) for value in result.positions)
    assert abs(result.positions[0] - Q0[0]) <= 0.02
    assert result.positions[1:] == Q0[1:]


def test_reanchor_preserves_sequence_and_starts_at_measured_state() -> None:
    source = SyntheticTargetSource(amplitude_rad=0.02, period_s=8.0)
    source.set_anchor(Q0, 10.0)
    assert source.make_result(10.0).source_sequence == 1
    moved = (0.01,) + Q0[1:]

    source.reanchor(moved, 11.0)
    recovered = source.make_result(11.0)

    assert recovered is not None
    assert recovered.source_sequence == 2
    assert recovered.positions == moved


def test_jsonl_trace_is_strict_json(tmp_path) -> None:
    path = tmp_path / "trace.jsonl"
    trace = JsonlTraceWriter(str(path))
    trace.write("requested", positions=list(Q0))
    trace.close()

    record = json.loads(path.read_text())
    assert record["event"] == "requested"
    assert record["positions"] == list(Q0)
