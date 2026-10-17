# Standalone Meta Quest Pose Input

This directory is the robot-disconnected foundation for a new Meta Quest
teleoperation stack. It uses the upstream Hand Tracking Streamer repository at
`libs/hand-tracking-streamer` for the headset application and protocol
documentation.

The current code only receives, validates, and displays poses. It does not
import Franka libraries, publish robot targets, or convert Unity coordinates
into a robot frame.

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
