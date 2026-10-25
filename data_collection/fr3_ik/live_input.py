"""Coalesce two Touch streams while retaining physical left-Y press edges."""

from __future__ import annotations

from dataclasses import dataclass

from data_collection.meta_quest.receiver import ReceivedPose


@dataclass(frozen=True)
class ControlFrame:
    poses: dict[str, ReceivedPose]
    left_y: bool


class PairedQuestInput:
    def __init__(self, mode: str) -> None:
        if mode not in {"left", "right", "duo"}:
            raise ValueError("mode must be left, right or duo")
        self.mode = mode
        self.latest: dict[str, ReceivedPose] = {}
        self.complete_pair: dict[str, ReceivedPose] | None = None
        self.left_ready = False
        self.pair_ready = False
        self.left_y_known = False
        self.left_y = False
        self.right_b = False
        self.pending_y_frame: int | None = None

    def offer(self, received: ReceivedPose) -> None:
        sample = received.sample
        if sample.source != "controller" or not sample.tracked or sample.side not in {"left", "right"}:
            return
        if sample.frame_id is None:
            return
        self.latest[sample.side] = received
        if sample.side == "left":
            pressed = bool(sample.clutch)
            if self.left_y_known and pressed and not self.left_y:
                self.pending_y_frame = sample.frame_id
            self.left_y = pressed
            self.left_y_known = True
            self.left_ready = True
        else:
            self.right_b = bool(sample.clutch)
            left = self.latest.get("left")
            if (
                left is not None
                and left.sample.frame_id == sample.frame_id
                and left.connection_id == received.connection_id
            ):
                self.complete_pair = {"left": left, "right": received}
                self.pair_ready = True

    def take(self) -> ControlFrame | None:
        if self.mode == "left":
            if not self.left_ready:
                return None
            self.left_ready = False
            poses = {"left": self.latest["left"]}
        else:
            if not self.pair_ready or self.complete_pair is None:
                return None
            self.pair_ready = False
            poses = self.complete_pair
        frame = poses["left"].sample.frame_id
        assert frame is not None
        pulse = self.pending_y_frame is not None and frame >= self.pending_y_frame
        if pulse:
            self.pending_y_frame = None
        selected = ("left", "right") if self.mode == "duo" else (self.mode,)
        return ControlFrame({side: poses[side] for side in selected}, self.left_y or pulse)
