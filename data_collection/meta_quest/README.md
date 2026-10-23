# Meta Quest Touch Input

This package implements the robot-disconnected Quest Touch input, raw recording,
coordinate conversion, target preview, and offline safety layers.

## Supported live path

The only supported operator path is:

```text
Quest Touch controllers
  -> ALVR client over Wi-Fi/TCP
  -> ALVR server + SteamVR + ALVR external driver
  -> direct background OpenVR bridge
  -> local TCP 127.0.0.1:8000
  -> PoseReceiver/raw_recorder
  -> lossless JSONL
```

Follow [Quest-ALVR-OpenVR-Reproduction.md](../../Quest-ALVR-OpenVR-Reproduction.md)
for the complete startup, recording, validation, and shutdown procedure. Other
Markdown files should link to that guide instead of maintaining another set of
operator commands.

After the ALVR Dashboard reaches `Streaming`, the current recording entry is:

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
unset PYTHONPATH PYTHONHOME
./scripts/record_meta_quest_openvr.sh \
  --output recordings/quest_raw/stage8-openvr-guided-20261020.jsonl
```

Use a new stage name after that output exists. Press `Ctrl+C` once and wait for
`Saved ... poses`. The wrapper starts both the recorder and OpenVR bridge,
forwards shutdown signals, waits for the completion summary, and leaves no
`.partial` file after a clean close.

This path never imports ROS or Franka libraries and has no robot publisher.

## Runtime split

- The raw recorder runs in the existing client `lerobot-vr` environment.
- `scripts/start_meta_quest_openvr_bridge.sh` selects the verified standalone
  Python 3.11, `openvr==2.12.1401`, and SteamVR runtime already on the client.
- ALVR/SteamVR transports and exposes tracking; ALVR does not save the project
  JSONL itself.
- No `adb reverse` is used by the current path.

The bridge waits for ALVR `Streaming`. If ALVR leaves that state, it stops
instead of emitting cached poses. The Quest must remain worn, awake, and in the
ALVR foreground during a capture.

## JSONL contract

The writer creates `<output>.partial`, flushes every accepted pose, appends a
completion summary, calls `fsync`, and atomically renames the partial file to the
requested output. Existing outputs are never overwritten.

A pose record contains:

- global `sequence`, bridge `frame_id`, TCP `connection_id`;
- bridge monotonic `device_timestamp_ns` and recorder
  `received_monotonic_s`;
- `side`, `source`, `tracked` and protocol `layout`;
- position in metres and quaternion in `xyzw` order;
- `grasp`, `clutch`, and `record` controls.

Every OpenVR poll normally produces two adjacent records with the same
`frame_id` and timestamp: first `left`, then `right`. Plotting all rows as one
trajectory therefore creates an artificial zigzag between the two controller
locations. Always group by `side`; the current single-right-arm work must use
`--side right`.

Current control mapping is:

```text
analog trigger -> grasp
B/Y button mask -> clutch
A/X button mask -> record
```

These are input fields only. Final teleoperation and episode semantics still
require explicit edge/debounce logic and safety review.

## Validate a capture

The authoritative guide contains the full validation sequence. A completed
capture must satisfy all of the following:

- replay reports `schema=1, lossless`;
- `completed=true` in the last summary record;
- `accepted_connections=1` for a formal episode;
- zero rejected and subscriber-dropped records;
- expected sides contain changing poses;
- no same-name `.partial` remains.

The guided reference capture is:

```text
recordings/quest_raw/stage6-openvr-guided-20261020.jsonl
928 poses: left=464, right=464
accepted_connections=1, rejected=0, dropped=0, completed=true
SHA-256: 0c6ba8544968137151d45e580722eeec4fd138a40db2213ecf742a1046fd3ed7
```

`stage7-openvr-guided-20261020.jsonl` is valid as right-hand evidence: the
right controller has 379 distinct full poses across 452 records. Its left side
is mostly stationary/piecewise updated, so it is not a dual-hand motion episode.

## Coordinate and target contract

Raw controller values use the Unity-like tracking convention produced by the
OpenVR adapter. They are not robot-base coordinates and must never be sent
directly to a robot.

The explicit Unity-to-robot basis is:

```text
robot_x =  quest_z
robot_y = -quest_x
robot_z =  quest_y
```

Positions use the basis matrix directly. Orientations use `B R B^T`; quaternion
components are not manually permuted. A configurable controller-to-tool rigid
transform is applied after the basis change. `PoseCalibration` maps motion
relative to the captured controller anchor onto a caller-provided target anchor.

The offline state machine defines `WAIT_INPUT`, `READY`, `FOLLOWING`,
`MANUAL_HOLD`, and `FAULT_HOLD`. Safety checks cover host timeout, TCP
connection change, invalid tracking, source pose jumps, target speed, workspace,
and quaternion continuity. Entry to `FOLLOWING` captures a fresh source anchor
against the last safe target to prevent engagement jumps.

The compact protocol cannot reliably distinguish a stationary controller from
frozen device frames and has no explicit recenter event. Conservative host
watchdogs and jump checks cover observable failures, but this remains a
real-robot enable constraint.

## Episode rule

Any ALVR/TCP disconnect or `connection_id` change invalidates the formal
episode. Stop and recollect the whole episode. Never concatenate data across a
reconnect or resume following inside the same candidate episode.

The protected `stage2-sixdof-safety3.jsonl` capture intentionally includes a
real connection change and is only a coordinate/safety regression fixture, not
training data.

## Deprecated paths

The following are retained only for source/history diagnostics and must not be
used as operator startup procedures:

- Hand Tracking Streamer over USB/ADB and its `Controllers Only` UI;
- the SAPIEN `RenderVRDisplay` SteamVR bridge, which returned cached poses on
  this client without VR frame submission;
- starting raw recorder and bridge manually in two terminals;
- the monitor-only route, which stores only latest in-memory values and cannot
  replace the full-rate JSONL recorder.

History is available in Git. Current Markdown intentionally omits executable
instructions for those routes.

## Current safety boundary

The strict offline FR3 IK and pure-Python A/B gate are implemented. The
standalone Quest ROS launcher starts only the adapter; it does not start GELLO,
a robot controller, Franka, MANUS, Sharpa, cameras, or the pair1 recorder.
The next gate is the dedicated 5 mm Quest capture plus a fresh read-only left
arm snapshot, followed by the staged operator-authorized hardware checks in
`docs/AB-Offline-Pipeline-20261023.md`. Fake hardware is not a release gate.
