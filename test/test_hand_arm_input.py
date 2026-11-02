from dataclasses import replace

import pytest

from data_collection.meta_quest.hand_arm_input import HandArmInput
from data_collection.meta_quest.protocol import ProtocolError, parse_pose_line
from data_collection.meta_quest.receiver import ReceivedPose


def buttons(sequence, *, b=False, a=False, trigger=0.0, connection=1, now=None):
    sample = parse_pose_line(f'right buttons|f={sequence}|t={sequence}: {trigger},{int(b)},{int(a)}')
    return ReceivedPose(sample, sequence / 30 if now is None else now, sequence, connection)


def left(sequence, *, connection=1, pressed=True):
    sample = parse_pose_line(f'left controller|f={sequence}|t={sequence}:0,1,0,0,0,0,1,1,{int(pressed)},1')
    return ReceivedPose(sample, sequence / 30, sequence, connection)


def test_button_records_are_not_tracked_poses_and_validate_controls():
    sample = buttons(1, b=True, a=True, trigger=0.7).sample
    assert sample.source == 'buttons' and not sample.tracked
    assert sample.clutch and sample.record and sample.grasp == 0.7
    for payload in ('nan,0,0', '1.1,0,0', '0,2,0', '0,0', '0,0,0,0'):
        with pytest.raises(ProtocolError):
            parse_pose_line('right buttons|f=1:' + payload)


def test_left_buttons_and_right_poses_cannot_take_over():
    stream = HandArmInput()
    stream.offer(left(1))
    frame = stream.take()
    assert not frame.left_y
    assert frame.poses['left'].sample.clutch is False
    assert frame.poses['left'].sample.grasp is None
    right_pose = replace(left(2), sample=replace(left(2).sample, side='right'))
    stream.offer(right_pose)
    assert stream.take() is None
    assert not stream.control_fresh(0.1)


def test_right_b_short_press_survives_coalescing_and_never_changes_left_pose():
    stream = HandArmInput()
    stream.offer(buttons(1))
    stream.offer(left(2))
    assert not stream.take().left_y
    stream.offer(buttons(3, b=True))
    stream.offer(buttons(4, b=False))
    stream.offer(left(5))
    frame = stream.take()
    assert frame.left_y and frame.poses['left'].sequence == 5
    stream.offer(left(6))
    assert not stream.take().left_y
    stream.offer(left(7))
    assert not stream.take().left_y


def test_a_discard_edges_independent_of_left_tracking_with_hysteresis():
    stream = HandArmInput()
    stream.offer(buttons(1))
    stream.offer(buttons(2, a=True))
    stream.offer(buttons(3, a=True))
    assert [event.operation for event in stream.pop_events(0.1)] == ['toggle']
    stream.offer(buttons(4, trigger=0.8))
    stream.offer(buttons(5, trigger=0.5))
    stream.offer(buttons(6, trigger=0.8))
    assert [event.operation for event in stream.pop_events(0.2)] == ['discard']
    stream.offer(buttons(7, trigger=0.1))
    stream.offer(buttons(8, a=True, trigger=0.9))
    assert [event.operation for event in stream.pop_events(0.3)] == ['discard']
    stream.offer(buttons(9, a=True, trigger=0.9))
    assert stream.pop_events(0.35) == []


@pytest.mark.parametrize('restart', ['connection', 'gap'])
def test_held_buttons_require_release_after_start_or_reconnect(restart):
    stream = HandArmInput()
    stream.offer(buttons(1, b=True, a=True, trigger=1))
    stream.offer(left(2))
    assert not stream.take().left_y
    assert not stream.pop_events(0.1)
    stream.offer(buttons(3))
    stream.offer(buttons(4, b=True, a=True, trigger=1))
    generation = stream.generation
    stream.offer(buttons(5, b=True, a=True, trigger=1,
                         connection=2 if restart == 'connection' else 1,
                         now=4 if restart == 'gap' else None))
    assert stream.generation == generation + 1
    stream.offer(left(6, connection=2 if restart == 'connection' else 1))
    assert not stream.take().left_y
    assert not stream.pop_events(4)


def test_expired_events_and_duplicate_records_cannot_replay():
    stream = HandArmInput()
    stream.offer(buttons(1))
    stream.offer(buttons(2, a=True))
    stream.offer(buttons(1))
    stream.offer(buttons(2, a=True))
    assert len(stream.pop_events(0.1)) == 1
    stream.offer(buttons(3))
    stream.offer(buttons(4, a=True))
    assert stream.pop_events(2.0) == []
