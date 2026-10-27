from data_collection.quest_action_policy import use_measured_hold


def test_hold_only_when_control_is_not_active():
    assert use_measured_hold(None, False)
    assert not use_measured_hold(None, True)
    for state in ("READY", "WAITING"):
        assert use_measured_hold(state, False)
        assert use_measured_hold(state, True)
    for state in ("FOLLOWING", "RETURNING", "FAULT"):
        assert not use_measured_hold(state, False)
        assert not use_measured_hold(state, True)
