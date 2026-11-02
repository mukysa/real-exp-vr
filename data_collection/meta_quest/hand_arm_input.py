"""Left pose / right buttons input, without ROS or robot dependencies."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace
import math

from .receiver import ReceivedPose


@dataclass(frozen=True)
class ButtonEvent:
    operation: str
    connection_id: int
    sequence: int
    received_monotonic_s: float


class HandArmInput:
    """Preserve button edges before pose coalescing; never use right poses.

    ``left_y`` and ControlFrame.left_y are legacy interface slots for the
    selected takeover button. In this class they exclusively represent B.
    """

    def __init__(self, *, timeout_s=2.0, trigger_press=0.75, trigger_release=0.25):
        if not 0 <= trigger_release < trigger_press <= 1:
            raise ValueError("expected 0 <= trigger_release < trigger_press <= 1")
        self.timeout_s = timeout_s
        self.trigger_press, self.trigger_release = trigger_press, trigger_release
        self.latest: dict[str, ReceivedPose] = {}
        self.right_b = False
        self.left_ready = False
        self.generation = 0
        self._right: ReceivedPose | None = None
        self._armed = {key: False for key in ("b", "a", "trigger")}
        self._down = dict(self._armed)
        self._pending_b: ButtonEvent | None = None
        self._events: deque[ButtonEvent] = deque(maxlen=32)
        self._last_output_pressed = False

    @property
    def left_y(self):
        return self.right_b

    def control_fresh(self, now):
        return self._right is not None and 0 <= now - self._right.received_monotonic_s <= self.timeout_s

    def offer(self, received: ReceivedPose):
        sample = received.sample
        if sample.side == "left" and sample.source == "controller" and sample.tracked:
            previous = self.latest.get("left")
            if previous is not None and previous.connection_id == received.connection_id and received.sequence <= previous.sequence:
                return
            self.latest["left"] = replace(received, sample=replace(sample, grasp=None, clutch=False, record=False))
            self.left_ready = True
            return
        if sample.side != "right" or sample.source != "buttons":
            return
        if sample.clutch is None or sample.record is None or sample.grasp is None or not math.isfinite(sample.grasp):
            return
        old = self._right
        if old is not None and received.connection_id == old.connection_id and received.sequence <= old.sequence:
            return
        reset = old is None or old.connection_id != received.connection_id or not 0 <= received.received_monotonic_s - old.received_monotonic_s <= self.timeout_s
        if reset:
            self.generation += 1
            self._armed = dict.fromkeys(self._armed, False)
            self._down = dict.fromkeys(self._down, False)
            self._pending_b = None
            self._events.clear()
            self._last_output_pressed = False
        self._right = received
        self.latest["right"] = received
        trigger = sample.grasp
        levels = {"b": bool(sample.clutch), "a": bool(sample.record),
                  "trigger": trigger >= (self.trigger_release if self._down["trigger"] else self.trigger_press)}
        edges = set()
        for key, down in levels.items():
            if not down:
                self._armed[key] = True
            if down and not self._down[key] and self._armed[key]:
                edges.add(key)
                self._armed[key] = False
            self._down[key] = down
        self.right_b = levels["b"]
        def event(operation):
            return ButtonEvent(operation, received.connection_id, received.sequence, received.received_monotonic_s)
        if "b" in edges:
            self._pending_b = event("takeover")
        if "trigger" in edges:
            self._events.append(event("discard"))
        elif "a" in edges and not levels["trigger"]:
            self._events.append(event("toggle"))

    def pop_events(self, now):
        events = list(self._events)
        self._events.clear()
        return [event for event in events if 0 <= now - event.received_monotonic_s <= 1.0]

    def take(self):
        from data_collection.fr3_ik.live_input import ControlFrame
        if not self.left_ready:
            return None
        self.left_ready = False
        left = self.latest["left"]
        event = self._pending_b
        pressed = False
        if event is not None:
            if left.connection_id != event.connection_id or left.received_monotonic_s - event.received_monotonic_s > self.timeout_s:
                self._pending_b = None
            elif left.received_monotonic_s >= event.received_monotonic_s and not self._last_output_pressed:
                pressed = True
                self._pending_b = None
        self._last_output_pressed = pressed
        return ControlFrame({"left": left}, pressed)
