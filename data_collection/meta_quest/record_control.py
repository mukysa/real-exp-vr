"""Local JSON control socket for the optional hand-arm recorder.

ROS stays in the server relay; the Conda recorder only imports pyzmq. Remote
events carry Unix deadlines, so client/server clocks must be synchronized.
The recorder's revision is a one-use capability: queued buttons cannot change
the next episode after a save, discard, rejection, or keyboard command.
"""
from __future__ import annotations

import math
import fcntl
import os
from pathlib import Path
import time
import uuid

import zmq


class RecorderControl:
    def __init__(self, context, endpoint="tcp://127.0.0.1:5559"):
        self._lock = None
        if endpoint.startswith("tcp://127.0.0.1:"):
            port = int(endpoint.rsplit(":", 1)[1])
            self._lock = (Path(os.environ.get("XDG_RUNTIME_DIR", "/tmp")) /
                          f"quest-hand-arm-recorder-{os.getuid()}-{port}.lock").open("a")
            try:
                fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except OSError:
                self._lock.close()
                raise RuntimeError("Another hand-arm recorder already owns this control port") from None
        self.socket = context.socket(zmq.DEALER)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.setsockopt(zmq.SNDHWM, 32)
        self.socket.setsockopt(zmq.RCVHWM, 32)
        self.socket.connect(endpoint)
        self.recorder_id = uuid.uuid4().hex
        self.revision = 0
        self.state = "idle"
        self.result = "ready"
        self.detail = ""
        self.event_id = None
        self.source = None
        self.source_at = 0.0
        self._seen = {}
        self._last_publish = float("-inf")

    def publish(self, *, force=False):
        now = time.monotonic()
        if not force and now - self._last_publish < 0.2:
            return
        payload = {"kind": "status", "recorder_id": self.recorder_id,
                   "revision": self.revision, "state": self.state,
                   "result": self.result, "detail": self.detail, "event_id": self.event_id}
        try:
            self.socket.send_json(payload, flags=zmq.NOBLOCK)
            self._last_publish = now
        except zmq.Again:
            pass

    def drain(self):
        events = []
        for _ in range(64):
            try:
                payload = self.socket.recv_json(flags=zmq.NOBLOCK)
            except zmq.Again:
                break
            except (ValueError, UnicodeError):
                continue
            if not isinstance(payload, dict):
                continue
            if payload.get("kind") == "source":
                status = payload.get("status")
                if isinstance(status, dict):
                    self.source, self.source_at = status, time.monotonic()
            elif payload.get("kind") == "event" and isinstance(payload.get("event"), dict):
                events.append(payload["event"])
        return events

    def following(self):
        return (self.source is not None and time.monotonic() - self.source_at <= 1.5
                and self._source_fresh()
                and self.source.get("hand_arm") is True
                and self.source.get("state") == "FOLLOWING"
                and self.source.get("right_control_fresh") is True)

    def _source_fresh(self):
        try:
            age = time.time() - float(self.source["generated_unix_s"])
            return -0.1 <= age <= 1.5
        except (TypeError, ValueError, KeyError, OverflowError):
            return False

    def validate(self, event):
        try:
            session, sequence = event["session"], event["sequence"]
            deadline = float(event["expires_at"])
            if not isinstance(session, str) or not session or type(sequence) is not int or sequence < 1:
                return "invalid event identity"
            if event.get("operation") not in {"toggle", "discard"}:
                return "unknown operation"
            if not math.isfinite(deadline) or not 0 < deadline - time.time() <= 1.1:
                return "expired event or unsynchronized clocks"
            if (self.source is None or time.monotonic() - self.source_at > 1.5 or not self._source_fresh()
                    or self.source.get("record_session") != session):
                return "stale or different control session"
            if event.get("recorder_id") != self.recorder_id or event.get("revision") != self.revision:
                return "stale recorder state; release and press again"
            if self.state not in {"idle", "recording"}:
                return "recorder busy or in error"
            if sequence <= self._seen.get(session, 0):
                return "duplicate event"
            if len(self._seen) >= 64:
                self._seen.clear()
            self._seen[session] = sequence
        except (KeyError, TypeError, ValueError, OverflowError):
            return "malformed event"
        return None

    def begin(self, command, event=None):
        self.revision += 1
        self.event_id = None if event is None else [event["session"], event["sequence"]]
        self.state = "saving" if command == "e" else "busy"
        self.result, self.detail = "accepted", command
        self.publish(force=True)

    def reject(self, detail, event=None):
        self.event_id = None if event is None else [event.get("session"), event.get("sequence")]
        self.finish(self.state, "rejected", detail)

    def finish(self, state, result, detail):
        self.state, self.result, self.detail = state, result, detail
        self.publish(force=True)

    def close(self):
        self.finish("offline", "stopped", "recorder stopped")
        self.socket.close(0)
        if self._lock is not None:
            self._lock.close()
