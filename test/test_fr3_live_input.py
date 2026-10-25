from data_collection.fr3_ik.live_input import PairedQuestInput
from data_collection.meta_quest.protocol import PoseSample
from data_collection.meta_quest.receiver import ReceivedPose


def pose(side, frame, sequence, *, clutch=False, connection=1):
    sample = PoseSample(
        side, "controller", (0.0, 1.0, 0.0), (0.0, 0.0, 0.0, 1.0),
        True, clutch=clutch, frame_id=frame,
    )
    return ReceivedPose(sample, frame / 30.0, sequence, connection)


def test_duo_uses_complete_same_frame_pair_and_left_y_only():
    stream = PairedQuestInput("duo")
    stream.offer(pose("left", 1, 1))
    assert stream.take() is None
    stream.offer(pose("right", 1, 2, clutch=True))
    frame = stream.take()
    assert frame is not None and set(frame.poses) == {"left", "right"}
    assert frame.left_y is False  # right B never takes over

    stream.offer(pose("left", 2, 3, clutch=True))
    assert stream.take() is None
    stream.offer(pose("right", 2, 4))
    frame = stream.take()
    assert frame is not None and frame.left_y is True
    assert frame.poses["left"].sample.frame_id == frame.poses["right"].sample.frame_id


def test_short_left_y_press_survives_coalescing():
    stream = PairedQuestInput("right")
    stream.offer(pose("left", 1, 1))
    stream.offer(pose("right", 1, 2))
    stream.take()
    stream.offer(pose("left", 2, 3, clutch=True))
    stream.offer(pose("right", 2, 4))
    stream.offer(pose("left", 3, 5, clutch=False))
    stream.offer(pose("right", 3, 6))
    frame = stream.take()
    assert frame is not None and frame.left_y is True
    assert set(frame.poses) == {"right"}
    stream.offer(pose("left", 4, 7))
    stream.offer(pose("right", 4, 8))
    assert stream.take().left_y is False


def test_different_frames_or_connections_never_form_duo_pair():
    stream = PairedQuestInput("duo")
    stream.offer(pose("left", 1, 1))
    stream.offer(pose("right", 2, 2))
    assert stream.take() is None
    stream.offer(pose("left", 3, 3))
    stream.offer(pose("right", 3, 4, connection=2))
    assert stream.take() is None


def test_left_mode_keeps_tracking_right_b_without_using_it():
    stream = PairedQuestInput("left")
    stream.offer(pose("left", 1, 1))
    stream.offer(pose("right", 1, 2, clutch=True))
    frame = stream.take()
    assert frame is not None and frame.left_y is False
    assert stream.right_b is True
