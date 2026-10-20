# Standalone Meta Quest Pose Input

This directory is the robot-disconnected foundation for a new Meta Quest
teleoperation stack. It uses the upstream Hand Tracking Streamer repository at
`libs/hand-tracking-streamer` for the headset application and protocol
documentation.

The current code only receives, validates, and displays poses. It does not
import Franka libraries, publish robot targets, or convert Unity coordinates
into a robot frame.

Two interchangeable controller transports are supported. The original APK
route remains available. The ALVR route reads the SteamVR controllers through
OpenVR and converts their poses back into the same raw Unity-frame contract
before any stage-2 calibration or safety processing:

```text
Quest controllers -> ALVR client --Wi-Fi/TCP--> ALVR server + SteamVR
  -> openvr_bridge -> local TCP :8000 -> PoseReceiver/raw_recorder -> JSONL
```

The bridge is input-only: it does not import ROS/Franka code or enable robot
output. It deliberately avoids registering a SteamVR action manifest because
that disables the legacy button-mask API used for B/clutch and A/record.
It also avoids the SAPIEN display adapter on this client: that path returned
cached controller poses unless a VR frame was submitted, while a background
OpenVR application was verified to receive continuously changing poses.

For the complete operator procedure, including every environment, ALVR,
SteamVR, recording, shutdown, and validation command, see
[`Quest-ALVR-OpenVR-Reproduction.md`](../../Quest-ALVR-OpenVR-Reproduction.md).

Start the existing ALVR Dashboard and SteamVR, put on the Quest, open ALVR in
the headset, and wait for the dashboard to show `Streaming`. Record with one
command from the existing `lerobot-vr` environment:

```bash
cd /home/landau/real-exp-vr
conda activate lerobot-vr
unset PYTHONPATH PYTHONHOME
./scripts/record_meta_quest_openvr.sh \
  --output recordings/quest_raw/<name>.jsonl
```

The wrapper starts the existing lossless raw recorder and the direct OpenVR
bridge, waits for the local receiver, and safely finalizes the `.partial` file
when `Ctrl+C` is pressed. Existing outputs are never overwritten. The Quest
must remain worn/awake with ALVR in the foreground; if ALVR leaves `Streaming`,
the bridge stops rather than writing cached poses. This Wi-Fi transport does
not use `adb reverse`.

For diagnosis, the two components can still be run in separate terminals:

```bash
# Terminal 1 (lerobot-vr)
python -m data_collection.meta_quest.raw_recorder \
  --output recordings/quest_raw/<name>.jsonl

# Terminal 2 (standalone OpenVR Python configured by this launcher)
./scripts/start_meta_quest_openvr_bridge.sh
```

Validate the reused OpenVR dependency path without opening a connection with:

```bash
./scripts/start_meta_quest_openvr_bridge.sh --dry-run
```

Successful live validation prints controller ids and increasing pose counts.
Replay/inspection must show `source=controller`, layout
`controller_compact_controls_v1`, changing position/quaternion values, and
B/A/trigger changes. Stop with `Ctrl+C`; do not hard-kill SteamVR.

## Current boundary

```text
Hand Tracking Streamer on Quest
  -> TCP over USB/ADB on port 8000
  -> data_collection.meta_quest.receiver
  -> validated latest wrist/controller poses in the Unity tracking frame
  -> 1 Hz terminal report plus live controller-pose visualization
```

The tested headset application is the controller-enabled
`maniskill-shadowhand` branch of
`Soo0803/VR-hand-tracking-and-controller`, commit
`a8f16e376d1090b93a5ba314e8d9afd0f3a7b7a3`. Android reports package
`com.wengmister.handtrackingstreamer`, version name `1.1.0`, version code `7`.
The archived APK is
`downloads/hand_tracking_streamer-maniskill-shadowhand.apk`, SHA-256
`e0b41ab36e0bcbbc1a1d616bd659bad6d93c02efcb332c1d87c2bee1c6e04879`.
The version name alone therefore does not distinguish this controller build
from upstream releases; use the source commit and APK digest above.

## Start a safe pose-only test

Connect and authorize the Quest, then run from the repository root:

```bash
./scripts/start_meta_quest_pose_monitor.sh
```

In the headset choose:

```text
Protocol: TCP (Wired / ADB)
IP:       127.0.0.1
Port:     8000
Mode:     Controllers Only (controller-enabled build)
```

Press **Start Streaming**. The terminal should show a live pose for each
tracked side. The configuration panel intentionally disappears while streaming,
so seeing only passthrough in the headset at that point is normal. Press the
flat **Menu** button on the left controller to stop streaming and restore the
panel. Stop the computer-side monitor with `Ctrl+C`.

The current Unity scene only auto-links `LeftHandRayInteractor` and
`RightHandRayInteractor` for menu interaction; it does not configure controller
UI rays. Enable Quest system hand tracking and use a pinch gesture to configure
the page. This is a Quest application limitation, not a controller connection
failure. The app also gates its Start button on Unity's
`Application.internetReachability`, even for ADB loopback, so Wi-Fi currently
has to be connected although the telemetry itself travels over USB. A future
APK should remove that network gate for wired mode and add controller UI input.

The launcher initially restores normal physical-proximity behavior. The monitor
requests Quest's mounted/awake state only after it receives the first controller
packet, so starting the monitor while Hand Tracking Streamer is closed does not
prevent normal headset sleep. After two seconds without a controller packet, or
when the monitor exits normally or through `Ctrl+C`, it restores physical
proximity sensing and normal headset sleep. Keep the Quest connected over USB,
and stop the monitor when it is no longer needed to avoid unnecessary display,
tracking, and battery use.

The terminal reports the latest controller poses once per second. A computer-side
window redraws whenever new controller records arrive, displays each controller
as a position plus quaternion-derived XYZ axes, and shows the measured receive
frequency for each side. The plot stays in the raw Unity tracking frame. To run
without a window (for example, over SSH), add `--no-visualization`.

The visualization now uses two distinct axis styles: pastel dashed arrows and a
center marker for the fixed Unity tracking frame, and saturated solid arrows
labelled at each controller for that controller's local orientation. This avoids
mistaking a controller-local axis for a global reference axis.

The controller-enabled APK currently installed on the lab Quest sends each
controller as `position, quaternion_xyzw, trigger, clutch, record`. The monitor
labels the front-trigger value as `grasp`; the final two flags correspond to
the B-button clutch and A-button recording controls used by the controller
build. They are displayed for input verification only and do not command a
robot.

The receiver keeps only the latest pose for each source in memory. The monitor
prints a summary at `--print-hz` (1 Hz by default); neither component persists
every incoming controller packet. Redirecting terminal output creates a
diagnostic summary log, not a raw 45--52 Hz trajectory and not a LeRobot
dataset. A dedicated raw recorder with device/host timestamps, frame identity,
tracking validity and explicit episode boundaries is required before collection.

## Full-rate raw recording and replay

Stage 2.2 adds a robot-disconnected JSONL recorder. It stores every validated
pose delivered by `PoseReceiver`, including its global sequence number, TCP
connection id and host monotonic receive time. Start it after activating the
`lerobot-vr` environment:

```bash
cd /home/landau/real-exp-vr
unset PYTHONPATH PYTHONHOME
./scripts/record_meta_quest_raw.sh \
  --output recordings/quest_raw/stage2-input.jsonl
```

Stop with `Ctrl+C`, or pass `--duration-s 120`. The writer first creates a
`.jsonl.partial` file and renames it only after a clean close and summary
record. Existing output files are never overwritten. Large captures under
`recordings/quest_raw/` are ignored by Git.

Validate without waiting for the original timing:

```bash
python -m data_collection.meta_quest.replay \
  recordings/quest_raw/stage2-input.jsonl --no-sleep
```

Use `--summary-only` to inspect only the count/loss status, or `--speed 2` for
two-times playback. A capture with a missing completion summary is rejected. A
completed capture with subscriber queue drops remains available for diagnosis
but is reported as `LOSSY` and must not become a reference fixture.

The retained real full-rate stage-2 capture is the read-only test fixture
`recordings/quest_raw/stage2-sixdof-safety3.jsonl`. It contains 3,945 lossless
poses (43 left and 3,902 right) and has SHA-256:

```text
2d84d7eb95cebb133998711684fe98038b862a47b72939a10eadf9beadbf7a66
```

The capture deliberately includes a real input gap and TCP connection change,
so it is a coordinate/safety regression fixture rather than a continuous
episode or training sample. Integration tests verify its digest before reading
and after target generation. Generated previews must always use a different
path; the preview writer rejects source/output aliasing and existing outputs.

The recorder/replayer and local TCP path have automated coverage, including the
protected real fixture above. These commands do not start ROS, GELLO, Franka,
MANUS, Sharpa or the server recorder.

## Stage 1 result (2026-10-17)

The client hardware path was validated with a Quest 3 running Android 14:

```text
Quest 3 + left/right Touch controllers
  -> Hand Tracking Streamer controller build
  -> adb reverse tcp:8000 tcp:8000
  -> PoseReceiver / monitor in lerobot-vr Python 3.12.13
```

The final snapshot reported `connections=12 poses=3659 rejected=0`. Both sides
showed independent position and quaternion changes. Left input was observed at
about 45--51 Hz and right input at about 44--52 Hz. Non-zero `grasp` occurred
on both sides; `record=1` was observed on the left and `clutch=1` on the right.
These observations validate transport and parsing, not the final robot-control
button assignment. The physical-button mapping must be deliberately frozen and
edge/debounce tested when the teleoperation state machine is implemented.

Evidence is in `logs/quest-input-stage1-20261017.log`. It contains the 1 Hz
monitor snapshots and is not sufficient for replaying the original full-rate
motion.

If port 8000 is already occupied, stop the old listener first. ADB reverse
rules are cleared whenever the ADB server or device transport restarts; the
launcher recreates the rule each time.

## Coordinate and safety contract

All reported positions and `xyzw` quaternions remain in the Unity local tracking
frame. The installed controller build obtains them with
`OVRInput.GetLocalControllerPosition()` and
`OVRInput.GetLocalControllerRotation()`, relative to the Quest tracking origin;
they are not global robot-base coordinates and must not be sent directly to a
robot. A robot teleoperation path needs an explicit transform chain:

```text
controller pose in Quest local tracking frame
  -> Quest room/tracking-origin frame (if required)
  -> calibrated Quest-to-robot-base transform
  -> clutch-relative, limited robot target
```

The next layers need explicit implementations and tests for:

1. Quest-to-robot frame calibration and origin management.
2. Clutch-relative motion so tracking origin jumps cannot move the robot.
3. Workspace, velocity, acceleration, and rotation limits.
4. Stale/frozen-pose detection and hold behavior.
5. An explicit dead-man enable and emergency-stop path.
6. A dry-run target visualizer before any FR3 backend is connected.

The calibration primitive is now available in `calibration.py`. At the start of
teleoperation, define a safe target pose, hold the controller at the desired
starting pose, and use one deliberate button rising edge to capture the Quest
anchor. Subsequent poses are mapped as motion relative to that anchor, so the
captured pose maps exactly to the predefined target. The utility does not yet
enable robot output; it is the tested foundation for that later state machine.

The source anchor and target anchor must use the same coordinate convention. The
current Quest packets are Unity left-handed tracking-frame values, while robot
poses are normally right-handed robot-base values. Apply and test the explicit
Unity-to-robot axis conversion before composing those transforms; calibration is
not a substitute for that conversion.

"Global" for teleoperation therefore means global in the robot base frame, not
an absolute coordinate supplied by Quest. Quest's tracking origin can be
recentered or drift over time, so the practical arm mapping should capture a
calibration pose and use clutch-relative deltas, with workspace and velocity
limits, before enabling robot output.

The intended future module boundary is:

```text
validated Quest poses
  -> calibrated, clutch-relative target generator
  -> dry-run target visualizer and safety supervisor
  -> explicitly enabled robot backend
```

## Stage 2.3 offline coordinate and target preview

`coordinates.py` now implements the explicit Unity-to-robot basis change:

```text
robot_x =  quest_z
robot_y = -quest_x
robot_z =  quest_y
```

Positions use this matrix directly. Orientations use `B R B^T`, preserving a
proper rotation across the handedness change; quaternion components are never
manually permuted. A configurable fixed controller-to-tool rigid transform is
applied after the basis change. The existing `PoseCalibration` then maps motion
relative to the captured controller anchor onto a caller-provided target anchor.

Create a robot-disconnected preview from the protected real capture:

```bash
./scripts/preview_meta_quest_targets.sh \
  recordings/quest_raw/stage2-sixdof-safety3.jsonl \
  --output recordings/quest_targets/stage2-sixdof-safety3-right.jsonl \
  --plot recordings/quest_targets/stage2-sixdof-safety3-right.png \
  --side right \
  --anchor first \
  --target-position 0 0 0 \
  --target-quaternion 0 0 0 1
```

`--anchor first` is for offline inspection only. Use `--anchor clutch` or
`--anchor record` when evaluating deliberate operator anchoring. A held button
at startup is not accepted until it has first been released and pressed again.
If the selected controller's TCP connection id changes, the preview enters
`HOLD` and never continues the old calibration silently.

The JSONL output records the input digest, basis, scale, tool transform, target
anchor, source sequence/connection id and every valid or held target. It is an
offline end-effector preview only: the launcher imports no ROS modules and has
no robot backend. Optional plotting is loaded lazily and requires matplotlib.

The retained capture physically checks the up, right, and forward translations
and includes two intended pitch motions. Their signs agree with the mapping
above, and all three recorded re-anchors preserve the previous target exactly.
The compact installed controller protocol still has no explicit tracking-origin
or recenter event, so a recenter cannot be distinguished reliably from physical
controller motion. This limitation does not block offline IK or fake hardware,
but must remain an explicit robot-enable constraint.

## Stage 2.4 offline clutch and safety state machine

`state_machine.py` adds a robot-disconnected state machine with distinct
`WAIT_INPUT`, `READY`, `FOLLOWING`, `MANUAL_HOLD`, and `FAULT_HOLD` states.
The configured button is treated as a debounced toggle: a deliberate rising
edge starts following, the next rising edge enters manual hold, and recovery
from a fault requires an observed release followed by a new rising edge.
Every entry to `FOLLOWING` captures a new source anchor against the last safe
target, so engagement and recovery do not jump back to an older target.

`safety.py` provides provisional offline checks for host receive timeout, TCP
reconnection, explicit invalid tracking, source position/rotation jumps,
target linear/angular speed, workspace radius, and target quaternion sign
continuity. A violation enters `FAULT_HOLD` and retains the last safe target.
These values are preview defaults derived in part from the stage-2 capture;
they are not validated FR3 limits and must be reviewed with IK and the actual
controller period before any robot backend exists.

Replay the protected real capture through the state machine:

```bash
./scripts/validate_meta_quest_safety.sh \
  recordings/quest_raw/stage2-sixdof-safety3.jsonl \
  --output recordings/quest_targets/stage2-sixdof-safety3-right-safety.jsonl \
  --plot recordings/quest_targets/stage2-sixdof-safety3-right-safety.png \
  --side right \
  --engage-button clutch \
  --target-position 0 0 0 \
  --target-quaternion 0 0 0 1
```

The output records all configured limits plus state and hold-reason counts.
The script imports no ROS packages and has no topic publisher or robot backend.
Tests inject stale timing, connection changes, invalid tracking, source jumps,
speed violations and workspace violations by copying in-memory records; the
operator-owned source JSONL is never edited.

For formal data collection, any TCP connection change invalidates the current
episode. The recorder must end or mark that candidate as interrupted, and a new
episode must start from a fresh Home/anchor; data on opposite sides of a reconnect
must never be concatenated. The state machine retains a tested recovery primitive
for diagnostics, but production episode control must not use it to resume the
same recording.

The compact controller protocol still cannot prove that changing coordinates
came from a Quest recenter rather than fast physical motion, and identical
packets cannot distinguish a genuinely stationary controller from frozen
device frames. The host-arrival watchdog and conservative jump detector cover
the failures that are observable today. Explicit frame ids, device timestamps,
tracking validity and recenter events remain protocol work before real control.
