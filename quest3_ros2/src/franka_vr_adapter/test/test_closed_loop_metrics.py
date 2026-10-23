import math

from franka_vr_adapter.closed_loop_runner import (
    EXPECTED_JOINTS,
    _all_finite,
    _best_sequence_match,
)


def _sample(time_s, positions):
    return time_s, EXPECTED_JOINTS, tuple(positions)


def test_all_finite_requires_every_sample_to_have_seven_finite_positions():
    valid = _sample(0.0, [0.0] * 7)
    short = _sample(0.1, [0.0] * 6)
    non_finite = _sample(0.2, [0.0] * 6 + [math.nan])

    assert _all_finite([valid])
    assert not _all_finite([valid, short])
    assert not _all_finite([valid, non_finite])


def test_sequence_match_allows_one_boundary_sample_but_rejects_bad_shape():
    first = _sample(0.0, [0.0] * 7)
    second = _sample(0.1, [0.1] * 7)
    boundary = _sample(-0.1, [-0.1] * 7)

    match = _best_sequence_match([first, second], [boundary, first, second])
    assert match["matched_count"] == 2
    assert match["max_error_rad"] == 0.0

    malformed = _sample(0.1, [0.1] * 6)
    bad_match = _best_sequence_match([first, malformed], [first, second])
    assert not (
        bad_match["coverage"] >= 0.98
        and bad_match["max_error_rad"] <= 1e-12
    )
