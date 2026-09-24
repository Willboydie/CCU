# Promo bringup plan

Promo Smart Marker bringup. G1–G17 have passed. Hardware gates H1–H10 run on the Raspberry Pi 5. The Pi and the tablet both join the phone hotspot `willys_little_chicken`. Edit on this PC, copy the tree with [scripts/deploy-pi.ps1](../scripts/deploy-pi.ps1), then run that gate's commands in an SSH session.

```
ssh -4 will@192.168.4.2
```

The login password is `kippen`. The deploy script uses the same login. `ssh` and `scp` use IPv4 (`-4`). `192.168.4.2` is the Pi ethernet address. On the phone hotspot the tablet broadcasts `SMFIND` to UDP port `8766`, and the CCU answers `SMHERE`. The tablet then opens TCP port `8765` on the address that answered. The ESP32-C3 Wi-Fi dongles stay off this link.

The CCU program follows [CMD_PROTOCOL.md](CMD_PROTOCOL.md), [POC_WORKFLOW.md](POC_WORKFLOW.md), and [MACRO_ROUTINES.md](MACRO_ROUTINES.md), with the promo pose rules in this document. `ccu/serial.cpp` is the POSIX implementation the Pi runs.

Pose tracking, tag finding, and the IMU are unfinished. This build does not implement them. The tag-finding chapters in the product docs stay as the later product. The promo rules and the byte layout live in [CMD_PROTOCOL.md](CMD_PROTOCOL.md), [POC_WORKFLOW.md](POC_WORKFLOW.md), and [MACRO_ROUTINES.md](MACRO_ROUTINES.md).

Accuracy of aiming is out of scope. `GalvoModel`, `LidarModel`, and `PathEngine` are naive mappings with named constants, each in its own file, so a later calibration can replace the function bodies.

---

## Promo pose

The device is level and stands at the plan origin. DXF `(0, 0)` is that origin (Riverside vertex V01). `PlanGeometry.marker` is ignored. The operator turns the base to bring vertices into the existing ±35° wedge, 13 m range.


| Quantity           | Promo value                                    |
| ------------------ | ---------------------------------------------- |
| Position           | `(east, north, up) = (0, 0, 0)` mm             |
| Tilt               | `(roll, pitch) = (0, 0)`                       |
| `lastKnownPose`    | `(0, 0, 0, 0, 0, 0)` on every telemetry frame  |
| `moved`            | `false` on every frame                         |
| `originSetStatus`  | `IDLE` (`0`) on every frame                    |
| `tagFindingStatus` | `POSE_KNOWN` (`0`) on every frame              |
| `effectiveYaw`     | Clockwise from due north, radians in `[0, 2π)` |


Set Origin is one press. The operator places the device on the origin, levels it, points it due north, and taps **Set Origin**. The tablet sends `BEGIN_ORIGIN_SET` with an empty payload. The CCU stores the current AS5048 stage angle as `northOffset`, sets `originIsSet = true`, and leaves the work mode idle. There is no tag walk, no laser-on for tags, and no `WAITING_FOR_TAG`.

```
effectiveYaw = wrap_2pi(northOffset - stage)
```

The AS5048 count increases anticlockwise when viewed from above (`kStageAngleInvert` is false in OrientationEngine). Subtracting the stage makes a clockwise turn increase `effectiveYaw`. At the moment of Set Origin, `stage == northOffset`, so `effectiveYaw` is 0 and the plan wedge points up (+north).

A second `BEGIN_ORIGIN_SET` while `originIsSet` is already true is dropped. `END_SESSION` clears origin, `northOffset`, survey results, and any projection path. Power-off does the same, because none of that is stored on disk.

`BEGIN_TAG_FINDING` and `TAG_ALIGNED` are dropped in every mode. The promo tablet never sends them.

---

## What the operator does

1. The four USB devices stay on the ports in `ccu_ports.cfg`. Start `ccu` on the Pi. Start the tablet app.
2. The connection dialog lists the device from `DEVICE_ID` (`displayName`, active or not).
3. **Start session** sends `BEGIN_NEW_SESSION`. **Join session** sends `JOIN_SESSION` when `sessionActive` is set.
4. Pose screen shows **Set Origin** until a telemetry frame has `originIsSet`. Then it shows **Pose Identified** and **Continue to plan**.
5. On the plan, turn the base until the chosen vertices sit in the wedge. Survey, then project.
6. **End session** is a separate control. It is the only control that clears origin.

A 5 s silence on `CONNECTED` drops the link and resumes `DEVICE_ID`. The session and the origin stay. The tablet treats itself as connected while it has seen `REGULAR_TELEMETRY` for that `deviceId` and `sessionId` in the last 5 s, and then sends `CONNECTED` at 10 Hz.

The pose screen has no tag pad, no Find Tags, no Reposition, and no Clear origin. The header chip is **Origin not set** or **Pose Known**. The plan banner for a missing origin stays. There is no "device moved" banner.

---

## Program shape

One thread. One `session_poll()` loop. Session data is file-static inside `session.cpp`. Other files call getters and setters (`session_mode()`, `session_running()`, `session_set_stage()`, `session_origin_is_set()`, and so on). No mode classes, no event bus, no per-routine threads.

```
session_poll()
  dongle_poll()                         // every pass, including session idle
  if session idle:
      device_id heartbeat at 2 Hz
      accept only BEGIN_NEW_SESSION / JOIN_SESSION
      return
  angle_poll()                          // latest 0xA0, apply stage
  if a frontend frame was read:
      if command_allowed(mode, cmd): dispatch
      else drop
  switch work mode:
      Idle:       return
      Surveying:  survey_poll()
      Projecting: project_poll()
  if 100 ms elapsed and the link is up:
      send REGULAR_TELEMETRY
```

`survey_poll` and `project_poll` return every pass. The sweep, the return, and the dwell are states inside `survey.cpp`. Lidar bytes are read only from `survey_poll`. Each result is written on the dongle as `SURVEY_RESULT`. `surveyFinished` sets the work mode back to idle. Projector bytes are read and written only from `project_poll`. Angle and the dongle are read from the session loop.

While the session is idle the loop still reads and writes the dongle, so `DEVICE_ID` goes out and `BEGIN_NEW_SESSION` can arrive. It does not read the angle, the lidar, or the projector, and it does not send telemetry.

### Accept table

`command_allowed(mode, cmd)` is a pure function in its own file. Tests call it directly. The session switch calls it and then drops or dispatches. `mode` here is the pair `(session running?, work mode, originIsSet)`.


| Command                                               | Session idle           | Running, work idle                | Surveying           | Projecting                   |
| ----------------------------------------------------- | ---------------------- | --------------------------------- | ------------------- | ---------------------------- |
| `BEGIN_NEW_SESSION`                                   | yes                    | drop                              | drop                | drop                         |
| `JOIN_SESSION`                                        | yes, if the id matches | drop                              | drop                | drop                         |
| `CONNECTED`                                           | drop                   | yes                               | yes                 | yes                          |
| `END_SESSION`                                         | drop                   | yes, if the id matches            | yes, abort then end | yes, end projection then end |
| `BEGIN_ORIGIN_SET`                                    | drop                   | yes, while `originIsSet` is false | drop                | drop                         |
| `BEGIN_TAG_FINDING`, `TAG_ALIGNED`                    | drop                   | drop                              | drop                | drop                         |
| `PERFORM_SURVEY`                                      | drop                   | yes, while `originIsSet`          | drop                | drop                         |
| `ABORT_SURVEY`                                        | drop                   | drop                              | yes                 | drop                         |
| `PERFORM_PROJECTION`                                  | drop                   | yes, while `originIsSet`          | drop                | drop                         |
| `PROJECTOR_PAUSE`, `PROJECTOR_PLAY`, `PROJECTION_END` | drop                   | drop                              | drop                | yes                          |
| `SET_LASER_POWER`                                     | drop                   | yes                               | yes                 | yes                          |
| `NUDGE_X`, `NUDGE_Y`                                  | drop                   | drop                              | drop                | yes                          |


The promo tablet has no control that sends a nudge. The CCU still accepts `NUDGE_X` / `NUDGE_Y` as `int16` DAC steps during projection, because the command catalog includes them.

`END_SESSION` during survey fails every node not yet reported (`surveyFinished` on the last of those frames), then wipes the session. During projection it stops the projector, then wipes the session.

### Files

New program under `ccu/`. Existing lidar and projector GUI targets stay as they are. The new executable does not include `lib/utils/frontend_protocol.hpp`.


| File                      | Role                                                                                              |
| ------------------------- | ------------------------------------------------------------------------------------------------- |
| `ccu/main.cpp`            | Load `ccu_ports.cfg` and `ccu_survey.cfg`, call `session_poll` until quit                        |
| `ccu/session.cpp`         | File-static state, the loop, the switch                                                           |
| `ccu/session.hpp`         | Getters and setters only                                                                          |
| `ccu/command_allowed.cpp` | Pure accept table                                                                                 |
| `ccu/yaw.cpp`             | `wrap_2pi`, `effective_yaw(north, stage)`                                                         |
| `ccu/protocol.cpp`        | Pack and unpack every promo frame                                                                 |
| `ccu/telemetry.cpp`       | Build `REGULAR_TELEMETRY` from getters                                                            |
| `ccu/device_id.cpp`       | Build `DEVICE_ID` and `PAIRING_ACCEPTED`                                                          |
| `ccu/survey.cpp`          | File-static survey progress, `survey_begin`, `survey_poll`, `survey_abort`, `survey_reset`        |
| `ccu/project.cpp`         | File-static path, `project_begin`, `project_poll`, `project_pause`, `project_play`, `project_end` |
| `ccu/galvo_model.cpp`     | Naive DAC ↔ angle                                                                                 |
| `ccu/lidar_model.cpp`     | Naive motor ↔ angle, azimuth sign flip, sample → east-north-up, range → up                      |
| `ccu/path_engine.cpp`     | Naive corner list                                                                                 |
| `ccu/serial.cpp`          | POSIX open, non-blocking read, write. This is the implementation the Pi runs.                     |
| `ccu/angle.cpp`           | Parse `0xA0`, ignore roll, pitch, and IMU yaw, publish stage                                      |
| `ccu/dongle.cpp`          | TCP listen for the tablet. UDP beacon `SMFIND` / `SMHERE` on port 8766. Paired when a tablet connects. |
| `ccu/lidar.cpp`           | Non-blocking lidar parser and commands. Tests inject status and control frames.                  |
| `ccu/projector.cpp`       | Parse projector status, `projector_point`, `projector_path`                                       |
| `ccu_ports.cfg`           | `angle=/dev/ttyACM0`, `dongle=tcp:8765`, `projector=/dev/ttyACM1`, `lidar=/dev/ttyACM3`           |
| `ccu_survey.cfg`          | Survey wedge, tolerance, dwell, and coarse-scan parameters. The program only reads this file.     |


Each `.cpp` stays a single job. Frame structs live in `ccu/protocol.hpp`. Model constants live next to the function that uses them.

Port functions keep their handle in the `.cpp` file. They call `serial_open`, `serial_read`, and `serial_write`. Unit tests do not open a serial port.

`main` reads `ccu_ports.cfg` from the working directory, or the path in `argv[1]`. It reads `ccu_survey.cfg` from the working directory. A missing survey file keeps the defaults below and the log says so. No USB probing. The orientation board and the tablet dongle share VID `303A` / PID `1001`, so the port file names the ports. On the Pi the working directory is `~/smart_marker/CCU`. `deploy-pi.ps1` copies `ccu_survey.cfg` with the tree.

### Naive models

Constants below are placeholders. Change the constant or the function body later. Callers stay the same.

**GalvoModel.** Device centre and beam origin are the same point. DAC `32768` is boresight. `kGalvoRadPerDac = 1.0e-4` rad per count (about 0.1 mrad).

```
azimuth   = (dac_x - 32768) * kGalvoRadPerDac
elevation = (dac_y - 32768) * kGalvoRadPerDac
```

The inverse clamps into `0 … 65535`. A point in device-centre coordinates uses `atan2` for azimuth and elevation. Mirror offset is zero.

**LidarModel.** Motor angles use the existing neutral in `lib/scan/Geometry.hpp` (`kMechNeutralX`, `kMechNeutralY`) as a stand-in. Rangefinder origin is `(0, 0, 0)` relative to the device centre. Reported range is the distance. Up of a return is `range_mm * sin(elevation)`, elevation positive above the horizontal. `kFloorUpMm = -150` is the stand-in height of the floor under a level device when a range is missing.

`kDeviceAzimuthSign = -1`. Motor X is `kMechNeutralX − device_azimuth`, so a clockwise plan azimuth aims to the right. The neutral-to-boresight map is unchanged: geometry azimuth is `mechanical_x − kMechNeutralX`, and device azimuth is the negation of that. A control sample uses `associated_angle_*` with `distance` in centimetres. Distance `0` is ignored.

**PathEngine.** Input is a list of device-centre aim points that already survived the FOV check, plus a laser duty. For each node, emit five projector points: laser off on the segment from the previous node, laser on for a 40 mm square corner (four corners, then back to the aim). No slope, no blanking curve. `projector_path` sends that list.

**Survey step.** One node at a time. `effectiveYaw` at `survey_begin` is frozen for the whole job. A later stage sample does not change that job's aim. Azimuth of a node at `(east, north)` mm, device at the origin:

```
bearing = wrap_2pi(atan2(east, north))    // clockwise from north
device_azimuth = wrap_pi(bearing - frozenYaw)
```

Farther than `D_MAX_M`, or with `device_azimuth` outside ±`ANGLE_MAX_RAD`: `SURVEY_RESULT` failed, error `OUT_OF_FOV`. The gimbal stays still for that node.

The first in-FOV node enables the motors when status says they are off, then turns Scan Engine on. Point-to-point goes to `(device_azimuth, −el)` and then to `(device_azimuth, +el)`. `el` is `ANGLE_MAX_RAD`, clamped into the mechanical Y window (about −0.79 to +0.51 rad) so a 35° top target is accepted. Samples count only on that upward leg. Each sample becomes east, north, and up with the frozen yaw:

```
device_azimuth = -(associated_x - kMechNeutralX)
bearing = wrap(frozenYaw + device_azimuth)
east  = range * cos(elevation) * sin(bearing)
north = range * cos(elevation) * cos(bearing)
up    = range * sin(elevation)
```

The sample whose horizontal position is closest to the node is the hit, and only when that distance is within `SURVEY_TOL_M`. Otherwise the result is `NOT_VISIBLE` and the job moves on. A hit sends the gimbal back to that sample's associated motor angles. Arrival is the live mechanical angle within `ARRIVE_TOL_RAD`. A move that exceeds `MOVE_TIMEOUT_S` is `NOT_VISIBLE` for that node. The job continues.

A coarse Lissajous then runs for `DWELL_TIME_S`. Amplitude and frequency come from `ccu_survey.cfg`, phase is 0, and the amplitude is shrunk so the centre ± amplitude stays inside the mechanical box. Capture starts once `scan_mode` is coarse and `dynamic_routine` is 0. `STOP_COARSE_SCANNING` ends the capture. The points stay in RAM.

The dwell cloud is an unconstrained least-squares plane, centred on the closest sweep point `(e0, n0)`:

```
up = a * (east - e0) + b * (north - n0) + c
```

The node's up is that plane at the node's `(east, north)`. `localSlopeNormal` is `normalize(-a, -b, 1)`, flipped so the up component is positive. Fewer than 3 valid dwell points, or a near-vertical plane, is `NOT_VISIBLE`.

A success stores one Node under the PlanNode id: east, north, up, both corner angles, and `localSlopeNormal`. A later success for the same id replaces it. A failure is reported and leaves any previous Node for that id in place. `SURVEY_RESULT` stays 26 bytes and carries up, not the normal. The last result sets `surveyFinished`. Work mode returns to idle, and the motors are disabled when this job enabled them. `ABORT_SURVEY` reports `ABORTED` for every node not yet reported, disables the motors, and returns to idle. `END_SESSION` does that and then clears the stored Nodes.

`survey_poll` returns every pass. The session loop writes each result as `SURVEY_RESULT`. `AzimuthPointFinder` and the blocking `LidarController` stay unused.

`ccu_survey.cfg` defaults:

| Key | Default |
| --- | --- |
| `D_MAX_M` | `13` |
| `ANGLE_MAX_RAD` | `0.61086524` (35°) |
| `SURVEY_TOL_M` | `0.30` |
| `DWELL_TIME_S` | `1.0` |
| `COARSE_AMP_X_RAD`, `COARSE_AMP_Y_RAD` | `0.10` |
| `COARSE_FREQ_X_HZ`, `COARSE_FREQ_Y_HZ` | `2`, `4` |
| `ARRIVE_TOL_RAD` | `0.02` |
| `MOVE_TIMEOUT_S` | `8` |

The tablet enables **Survey** only for vertices inside its own ±35° / 13 m wedge. A CCU `OUT_OF_FOV` from the tablet means `ccu_survey.cfg` is tighter than that wedge.

**Projection step.** Drop any node id that has no stored survey result (`NOT_SURVEYED`) or that now sits outside the wedge (`OUT_OF_FOV`). Run `PathEngine` on the rest. Send `PROJECTION_READY`. `projectorStatus` goes `COMPUTING_PATH` then `PROJECTING`. `project_poll` feeds the projector. Pause freezes the stream. End discards the path and turns the laser off.

### Orientation firmware

Edit OrientationEngine only as follows. Leave the BNO085 code alone. With the chip missing, roll, pitch, and IMU yaw already stay at 0, and the CCU discards those three floats.


| Change                                                      | From                                   | To                                                     |
| ----------------------------------------------------------- | -------------------------------------- | ------------------------------------------------------ |
| `kOrientationIntervalMs` in `include/orientation_frame.hpp` | `20`                                   | `100`                                                  |
| Stage low-pass                                              | `alpha = 0.25` in `src/as5048_pwm.cpp` | first-order, `tau = 1.0` s, on the sin/cos accumulator |


Flash the `esp32-c3-serial` environment so `0xA0` comes out of the sensor board's own USB CDC. On the Pi that port is `/dev/ttyACM0`.

Frame, unchanged, 16-byte payload: `roll`, `pitch`, `yaw` (IMU), `stage` (AS5048), little-endian floats. Command byte `0xA0`.

---

## Wire layout

Frame format stays the one in [CMD_PROTOCOL.md](CMD_PROTOCOL.md): `cmd u8`, `length u16` little-endian, then payload. Never send `0xE4`, `0xE5`, or `0xE6` on the tablet link. The TCP socket is marked paired when the tablet connects, and the CCU then sends `DEVICE_ID`.

Strings are a `u8` length plus that many UTF-8 bytes, not NUL-terminated. Lengths above 32 are rejected and the frame is dropped. Floats are IEEE-754 binary32, little-endian. Bools are `0` or `1`.

`0x05` `PROJECTION_READY` is a new device → frontend packet. It is sent once when a projection is accepted, before the first point.

### Shared structs

**Pose** (24 bytes), same field order the tablet already uses:


| Offset | Type | Field       |
| ------ | ---- | ----------- |
| 0      | f32  | `east_mm`   |
| 4      | f32  | `north_mm`  |
| 8      | f32  | `up_mm`     |
| 12     | f32  | `roll_rad`  |
| 16     | f32  | `pitch_rad` |
| 20     | f32  | `yaw_rad`   |


Promo telemetry writes six zeros.

**PlanNode** (18 bytes), carried by `PERFORM_SURVEY`, `PERFORM_PROJECTION`, and `PROJECTION_READY`:


| Offset | Type | Field                    |
| ------ | ---- | ------------------------ |
| 0      | u16  | `id`                     |
| 2      | f32  | `east_mm`                |
| 6      | f32  | `north_mm`               |
| 10     | f32  | `corner_inner_angle_rad` |
| 14     | f32  | `corner_orientation_rad` |


Ids are the integer form of the tablet labels (`"V01"` → `1`), the same mapping as `NodeIds` today.

### Device → tablet

`0x01` ****`DEVICE_ID`

`deviceId` string, `displayName` string, `sessionActive` bool, `sessionId` string. Promo identity is `deviceId = "sm-ccu"`, `displayName = "Smart Marker"`. `sessionId` is empty while no session is active. A new session id is 8 hex characters from a counter plus the start time.

Heartbeat period while the link is down: 2 Hz.

`0x02` ****`PAIRING_ACCEPTED`

`deviceId` string. Then the CCU latches the link up and stops `DEVICE_ID`.

`0x03` ****`REGULAR_TELEMETRY`

`deviceId` string, `sessionId` string, `originIsSet` bool, `lastKnownPose` Pose, `effectiveYaw` f32, `moved` bool, `originSetStatus` u8, `tagFindingStatus` u8, `surveyStatus` u8, `projectorStatus` u8, `projectorRunning` bool, `laserOn` bool, `laserPower` u8.

10 Hz while a session exists, including while the tablet is disconnected. The tablet ignores a frame whose ids do not match. Ignored frames do not refresh the 5 s connected timer.

Promo fills `moved = 0`, pose zeros, `originSetStatus = 0`, `tagFindingStatus = 0`. `laserPower` is 15–99. Default duty before the tablet sends one is 30.

`0x04` ****`SURVEY_RESULT`


| Field                    | Type |
| ------------------------ | ---- |
| `id`                     | u16  |
| `east_mm`                | f32  |
| `north_mm`               | f32  |
| `up_mm`                  | f32  |
| `corner_inner_angle_rad` | f32  |
| `corner_orientation_rad` | f32  |
| `surveyed`               | bool |
| `success`                | bool |
| `errorCode`              | u8   |
| `surveyFinished`         | bool |


`up_mm` is in the device-centre frame (pose is the origin, so this is also plan up). On failure, `up_mm = 0` and `surveyed = 0`.

`0x05` ****`PROJECTION_READY`

`count` u16, then `count` PlanNodes that will actually be drawn, then `errorCount` u8, then that many `{ id u16, errorCode u8 }` for the nodes that were left out.

### Tablet → device


| cmd                         | Payload                                                     |
| --------------------------- | ----------------------------------------------------------- |
| `0x10` `CONNECTED`          | `tabletId` string. Promo tablet id is `"pad"`.              |
| `0x20` `BEGIN_NEW_SESSION`  | empty                                                       |
| `0x21` `JOIN_SESSION`       | `sessionId` string                                          |
| `0x22` `END_SESSION`        | `sessionId` string                                          |
| `0x23` `BEGIN_ORIGIN_SET`   | empty                                                       |
| `0x24` `BEGIN_TAG_FINDING`  | empty. Dropped.                                             |
| `0x25` `PERFORM_SURVEY`     | `count` u16, then `count` PlanNodes                         |
| `0x26` `ABORT_SURVEY`       | empty                                                       |
| `0x27` `PERFORM_PROJECTION` | same as survey                                              |
| `0x28` `PROJECTOR_PAUSE`    | empty                                                       |
| `0x29` `PROJECTOR_PLAY`     | empty                                                       |
| `0x2A` `PROJECTION_END`     | empty                                                       |
| `0x2B` `SET_LASER_POWER`    | `duty` u8, legal range 15–99. Outside that, drop the frame. |
| `0x2C` `NUDGE_X`            | `dx` i16 DAC counts                                         |
| `0x2D` `NUDGE_Y`            | `dy` i16 DAC counts                                         |
| `0x2E` `TAG_ALIGNED`        | `tagId` string. Dropped.                                    |


There is no `REGISTER_NODES` and no `FIRMWARE_VERSION`. The tablet sends the plan vertices inside the survey and projection commands. The CCU keeps survey results in `survey.cpp` until `END_SESSION`.

### Error codes


| Code | Name           | Used on                                   |
| ---- | -------------- | ----------------------------------------- |
| 0    | `NONE`         | success                                   |
| 1    | `NOT_VISIBLE`  | survey, no range in the dwell window      |
| 2    | `OUT_OF_FOV`   | survey or projection                      |
| 3    | `ABORTED`      | survey cancelled or session ended mid-job |
| 4    | `NOT_SURVEYED` | projection                                |


### Enums

Unchanged from [CMD_PROTOCOL.md](CMD_PROTOCOL.md): `OriginSetStatus`, `TagFindingStatus`, `SurveyStatus`, `ProjectorStatus`. Promo never leaves origin-set `IDLE` or tag-finding `POSE_KNOWN`. `SurveyStatus` is `SCANNING` (`1`) for the duration of `survey_poll`'s job. `ProjectorStatus` is `COMPUTING_PATH` (`1`) while `PathEngine` runs, then `PROJECTING` (`2`) until end. `projectorRunning` follows play and pause, and is false whenever status is not `PROJECTING`.

---

## Tablet

Repo: `SmartMarkerFrontend`. Replace the 40-byte codec. `docs/FRONTEND_PROTOCOL.md` on the CCU side is not the promo contract.


| Area                                           | Change                                                                                            |
| ---------------------------------------------- | ------------------------------------------------------------------------------------------------- |
| `data/DeviceProtocol.kt`                       | Cmd bytes and payloads from the layout above                                                      |
| `device/SerialFrameParser.kt`                  | Decode `DEVICE_ID`, `PAIRING_ACCEPTED`, `REGULAR_TELEMETRY`, `SURVEY_RESULT`, `PROJECTION_READY`  |
| `device/TabletFrameEncoder.kt`                 | Encode the tablet rows above. `BEGIN_ORIGIN_SET` is empty                                         |
| `device/SerialFrameParserTest.kt`              | Rewrite for the new frames                                                                        |
| `ui/screens/PoseScreen.kt`                     | **Set Origin**, then **Pose Identified** and **Continue to plan**                                 |
| `ui/SmartMarkerViewModel.kt`                   | `setOrigin` sends only `BEGIN_ORIGIN_SET`. Drop tag search, tag confirm, reposition, clear origin |
| `ui/chrome/AppChrome.kt`                       | Chip is origin-not-set or pose-known. Connection dialog gains Start, Join, and End session        |
| `ui/canvas/PlanCanvas.kt` and `data/Models.kt` | Device and wedge at `(0, 0)`. Yaw 0 points +north. Clockwise yaw turns the wedge toward +east     |
| `ui/screens/PlanScreen.kt`                     | Survey and project send PlanNodes. Remove the "device moved" banner                               |
| Keepalive                                      | While telemetry for the open session is fresher than 5 s, send `CONNECTED` at 10 Hz               |


`posed` becomes true only from `originIsSet` on a matching telemetry frame. The view-model does not mark the pose locally on the button press.

Wedge geometry, screen y up = north:

```
east  = sin(effectiveYaw)
north = cos(effectiveYaw)
```

A node at `(east, north)` is in view when its distance from `(0, 0)` is at most 13 m and the clockwise bearing `atan2(east, north)` is within ±35° of `effectiveYaw`.

Laser slider values are clamped to 15–99 before `SET_LASER_POWER`.

---

## Unit-test gates

C++ tests follow `tests/protocol_test.cpp`: a `check` helper, `FAIL:` on stderr, exit 0 only when nothing failed. Kotlin tests follow the existing JUnit classes under `app/src/test`. Each gate is its own executable or test class. A gate must pass before the next one is started. G1–G6, G8–G11, and G14–G17 have passed. G7, G12, and G13 are reopened for the elevation-sweep survey. Hardware gates run on the Pi.

Port I/O is not part of these gates. Session-static tests call `session_reset()` first and run one at a time.


| Gate | Proves                                                                                                                                                                                                                                                                                     | Run                                               |
| ---- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | ------------------------------------------------- |
| G1   | C++ pack/unpack of every frame in the layout, including overlong strings dropped, duty outside 15–99 dropped, and `0xE5` refused                                                                                                                                                           | `ccu_protocol_test`                               |
| G2   | Kotlin parser and encoder agree with the same byte fixtures as G1                                                                                                                                                                                                                          | `SerialFrameParserTest`, `TabletFrameEncoderTest` |
| G3   | `effective_yaw`: at equal angles yaw is 0; a stage decrease of `π/2` yields yaw `π/2` (clockwise); wrap into `[0, 2π)`                                                                                                                                                                     | `ccu_yaw_test`                                    |
| G4   | Kotlin FOV: device at `(0,0)`, yaw 0 includes a node on +north and excludes one on +east; yaw `π/2` does the opposite; `geometry.marker` is not the device point                                                                                                                           | `PlanCoordinatesTest`                             |
| G5   | `command_allowed` matches the accept table, including tag commands dropped in every mode, and survey refused while origin is clear                                                                                                                                                         | `ccu_command_allowed_test`                        |
| G6   | `GalvoModel` round-trips a DAC pair through angle and back, clamps at 0 and 65535, and uses `32768` as boresight                                                                                                                                                                           | `ccu_galvo_model_test`                            |
| G7   | `LidarModel` maps the neutral motor pair to azimuth 0, elevation 0; a known elevation and range produce the expected up; origin offset is `(0,0,0)`. Device azimuth `+0.2` rad maps to motor X `kMechNeutralX − 0.2`, and the inverse recovers `+0.2`                             | `ccu_lidar_model_test`                            |
| G8   | `PathEngine` on two nodes emits a laser-off connector and a laser-on 4-point corner for each, and an empty input emits an empty path                                                                                                                                                       | `ccu_path_engine_test`                            |
| G9   | Bearing helper: a node on +north has bearing 0, a node on +east has bearing `π/2`, outside ±35° is out of FOV                                                                                                                                                                              | `ccu_bearing_test`                                |
| G10  | `session_reset`, then `BEGIN_NEW_SESSION` latches running and a session id. `BEGIN_ORIGIN_SET` with stage `1.2` rad stores that north offset, sets origin, yaw 0. A later stage `1.2 - π/2` reports yaw `π/2`. A second origin command is dropped. `END_SESSION` clears origin and running | `ccu_session_test`                                |
| G11  | Telemetry built from those getters has pose zeros, `moved = 0`, both status enums `0`, and the yaw from G10                                                                                                                                                                                | `ccu_telemetry_test`                              |
| G12  | Scripted lidar frames, no serial port. An out-of-wedge node emits `OUT_OF_FOV` and no point-to-point command. An in-wedge sweep with a known plane stores a Node (id, up, `localSlopeNormal`) and sets `surveyFinished` on that result. A sweep whose closest sample is outside `SURVEY_TOL` emits `NOT_VISIBLE` and stores no Node. A second success for the same id replaces the Node. A failure leaves the previous Node in place | `ccu_survey_test`                                 |
| G13  | Projection of an unsurveyed id yields `PROJECTION_READY` with that id and `NOT_SURVEYED`. A Node planted through the G12 transport is included, and the path from G8 is stored                                                                                                             | `ccu_project_test`                                |
| G14  | `END_SESSION` mid-survey emits `ABORTED` for the remaining ids and `survey_reset` leaves no results for a later projection                                                                                                                                                                 | `ccu_session_test` (second case)                  |


G12's seam is a transport on `ccu/lidar.hpp`. The test pushes `StatusPayload` and `ControlPayload` and reads the command frames. The Pi build uses the serial port. The test loads a short `DWELL_TIME_S` and does not sleep. That is the only test seam into the lidar port.

Kotlin UI gates, still without a device:


| Gate | Proves                                                                                                                                                               |
| ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| G15  | `setOrigin` encodes a single empty `BEGIN_ORIGIN_SET` and does not mark `posed` until telemetry says `originIsSet`                                                   |
| G16  | Telemetry with `moved = 1` does not clear `posed` and does not show a moved banner. Promo firmware will not send that bit; the UI still treats pose as `originIsSet` |
| G17  | Start session encodes `BEGIN_NEW_SESSION`. Join encodes the advertised id. End session encodes `END_SESSION` and clears local origin                                 |


---

## Pi host

The CCU runs as `will` on the Pi at `192.168.4.2`. That user is in `dialout`, so it can open the tty devices. The tree on the Pi is `~/smart_marker/CCU`.

The tablet link is the phone hotspot `willys_little_chicken`. The saved Wi-Fi client profile is `netplan-wlan0-willys_little_chicken`. The CCU listens on TCP `8765`. Each `dongle_poll` also answers UDP port `8766`: a datagram `SMFIND` gets `SMHERE` back, and the tablet opens TCP to the address that answered. The phone must let devices on the hotspot reach each other.

The access-point profile is `Hotspot` (SSID `SmartMarker`). It autoconnects at boot and owns `wlan0`, so the phone profile stays off the radio. The access-point setup also set `connection.autoconnect no` on `netplan-wlan0-willys_little_chicken`. Run the following in the Ethernet SSH session. These commands delete every access-point profile and turn autoconnect back on for the phone hotspot. A reboot then joins `willys_little_chicken` and has no access point to start.

```
sudo nmcli connection down Hotspot || true
sudo nmcli connection modify Hotspot connection.autoconnect no connection.autoconnect-priority -1 || true
sudo nmcli connection delete Hotspot || true
nmcli -t -f NAME,802-11-wireless.mode connection show | while IFS=: read -r name mode; do
  if [ "$mode" = "ap" ]; then
    sudo nmcli connection down "$name" || true
    sudo nmcli connection modify "$name" connection.autoconnect no connection.autoconnect-priority -1
    sudo nmcli connection delete "$name"
    echo "deleted $name"
  fi
done
sudo nmcli connection modify "netplan-wlan0-willys_little_chicken" connection.autoconnect yes connection.autoconnect-priority 100
sudo nmcli connection up "netplan-wlan0-willys_little_chicken"
nmcli connection show
nmcli -f NAME,TYPE,DEVICE connection show --active
ip -4 addr show wlan0
```

`nmcli connection show` after that has no row whose wireless mode is `ap`. The active Wi-Fi device is `netplan-wlan0-willys_little_chicken`. `ip -4 addr show wlan0` prints the address the phone handed out. The tablet learns that address from the beacon. If `connection down` says a profile is already inactive, the delete still runs.

After a reboot, the same `connection show --active` lists the phone profile on `wlan0`.


| Device             | Config name | Port           | USB identity                                              |
| ------------------ | ----------- | -------------- | --------------------------------------------------------- |
| Orientation sensor | `angle`     | `/dev/ttyACM0` | Espressif `88:56:A6:2C:40:08`                             |
| Tablet link        | `dongle`    | `tcp:8765`     | phone hotspot `willys_little_chicken`, UDP beacon `8766` |
| Projector          | `projector` | `/dev/ttyACM1` | Teensy `18997040`                                         |
| Lidar              | `lidar`     | `/dev/ttyACM3` | Teensy `18996070`                                         |


`ccu_ports.cfg` is those four `name=path` lines. `scripts/pi-serial-ports.ps1` lists the same USB identities if a replug moves a tty number; update the config to match before the next gate.

Before H1, the SSH session installs the Pi build packages once:

```
sudo apt update && sudo apt install -y build-essential cmake ninja-build git pkg-config libboost-dev libgl1-mesa-dev libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
```

That command installs `cmake` and the Boost headers. `g++` is already on the Pi. GLFW is built with Wayland off, so `wayland-scanner` is not required.

For each hardware gate, from this PC:

```
.\scripts\deploy-pi.ps1 -Gate H1
```

The script copies `CMakeLists.txt`, `ccu_ports.cfg`, `ccu/`, `tests/`, `lib/`, and `docs/` to `will@192.168.4.2:~/smart_marker/CCU`. It then prints the SSH commands for that gate. Run them in a session you opened yourself:

```
ssh -4 will@192.168.4.2
cd ~/smart_marker/CCU
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target ccu
./build/ccu ccu_ports.cfg
```

Ctrl+C any `ccu` already running in that session before starting the next one.

---

## Hardware gates

Each step runs on the Pi after `deploy-pi.ps1 -Gate Hn`. Watch the real device. These are not unit tests.


| Step | Setup                                                            | Pass                                                                                                                                                                                                                                                         |
| ---- | ---------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| H1   | `ccu_ports.cfg` names the four ports above. `ccu_survey.cfg` is in the working directory. Build and run `ccu`. | Process stays up. Each port opened. Log shows `wifi listening 0.0.0.0:8765` and `wifi beacon 0.0.0.0:8766`. Log shows the survey file loaded (`D_MAX_M` 13, `ANGLE_MAX_RAD` about 0.611). Rename the survey file away and start again: the process stays up and the log says the defaults are in use. Put the file back. |
| H2   | Orientation board flashed, 10 Hz, tau 1 s. Turn the base slowly. | Log of stage updates at about 10 Hz. A fast flick does not jump the filtered angle by the full flick. Roll, pitch, and IMU yaw in the log stay 0.                                                                                                            |
| H3   | Tablet and Pi both on `willys_little_chicken`. App installed over the PC USB lead. | Dialog shows **Smart Marker**. Session idle. No telemetry.                                                                                                                                                         |
| H4   | Tap **Start session**.                                           | `PAIRING_ACCEPTED`. Telemetry at 10 Hz. Pose screen still says **Set Origin**. `originIsSet` is false. `moved` is false. Pose zeros.                                                                                                                         |
| H5   | Point the device due north. Tap **Set Origin**.                  | Next telemetry has `originIsSet`. Pose screen shows **Pose Identified**. Wedge sits on `(0, 0)` and points up.                                                                                                                                               |
| H6   | Turn the base clockwise about 90°.                               | Wedge turns toward +east. `effectiveYaw` near `π/2`. Close the tablet app, or leave the phone hotspot, for 6 s. `DEVICE_ID` resumes. Origin stays. Open the app on the phone hotspot and tap **Join session**. Wedge matches the current base. |
| H7   | **End session**, start again.                                    | Origin is clear. Wedge hidden. Set Origin is required again.                                                                                                                                                                                                 |
| H8   | Origin set. Survey sub-gates H8.1–H8.8 below, in order. Deploy with `-Gate H8`. | Each sub-gate passes. After H8.3 the vertex shows **Surveyed** and **Survey** can be tapped again. |
| H9   | Project the vertex surveyed in H8.3.                             | `PROJECTION_READY` lists it. The laser draws the 40 mm corner on the floor at that vertex, at the plane-fit height. **Pause** turns it off, **Play** starts it again, **End** finishes the routine and discards the pathway. A vertex the tablet shows as **Not visible** keeps **Project** disabled. |
| H10  | Video path, cold start.                                          | H3 through H6, then two in-wedge floor vertices surveyed with the base held still, then both projected.                                                                                                                     |


H10 is the promo-video check.

### Survey sub-gates

These extend H8. Watch the gimbal and the tablet. The **SURVEYING** overlay is the progress readout (`done / total`). **Cancel** on that overlay sends `ABORT_SURVEY`. The tablet paints every failed result as **Not visible**, so a still gimbal is what distinguishes `OUT_OF_FOV` from a sweep that ends `NOT_VISIBLE`.

| Step | Setup | Pass |
| ---- | ----- | ---- |
| H8.1 | In `ccu_survey.cfg` set `ANGLE_MAX_RAD` to `0.20`. Restart `ccu`. Origin set, wedge up. Select one vertex the tablet still draws inside its 35° wedge and outside ±0.20 rad of the facing. Tap **Survey**. Restore `ANGLE_MAX_RAD` to `0.61086524` and restart before H8.2. | **Survey** is enabled. The gimbal stays still. The overlay reaches `1 / 1` and closes. The vertex shows **Not visible**. **Survey** can be tapped again. |
| H8.2 | Default `ccu_survey.cfg`. One in-wedge vertex on the floor, within 13 m. Tap **Survey** and watch the gimbal through the upward sweep. | Motors enable. The gimbal seeks the low end of that azimuth, then sweeps upward to the high clamp (the mechanical Y limit, about +0.51 rad, because 35° is wider than the gimbal). |
| H8.3 | Same survey as H8.2, from the return onward. The floor at that vertex is within 0.30 m of the plan point. | The gimbal returns to a pose on the same azimuth. A small Lissajous runs for about 1 s, then stops, and the motors disable. The overlay closes. The vertex shows **Surveyed**. **Survey** can be tapped again. |
| H8.4 | Default config. A different vertex from H8.3, in the tablet wedge, whose plan point is not on a surface within 0.30 m (past the near wall, still inside 13 m). Tap **Survey**. | The gimbal sweeps that azimuth from low to high and does not start the Lissajous. The vertex shows **Not visible**. The H8.3 vertex still shows **Surveyed**. The overlay finishes. The motors disable. |
| H8.5 | Two other in-wedge floor vertices. Mark the boresight on the floor with tape. Tap **Survey**. While the first sweep is moving, turn the base about 20°. | The second sweep stays on the tape line from the tap. The on-screen wedge turns with the base. Each of these two shows **Surveyed** when its frozen-yaw return lands within 0.30 m of the plan point, and **Not visible** when it does not. The overlay finishes at `2 / 2`. The motors disable. |
| H8.6 | Turn the base back until two unsurveyed floor vertices sit in the wedge. Tap **Survey**. During the first dwell, tap **Cancel**. | The overlay closes. The Lissajous stops and the motors disable. Those two stay unsurveyed. **Survey** can be tapped again. **Project** stays disabled while they are the only selection. |
| H8.7 | The H8.3 vertex still shows **Surveyed**. Survey it again. During the dwell, **End session**. | The gimbal stops and the motors disable. The next screen requires **Set Origin**. Start a new session, set origin, and select that vertex: it is unsurveyed, and **Project** stays disabled. |
| H8.8 | Set `DWELL_TIME_S` to `0.3`. Restart `ccu`. Survey one in-wedge floor vertex. Restore `DWELL_TIME_S` to `1.0` and restart. | The Lissajous is a short burst, much shorter than the dwell in H8.3. The vertex shows **Surveyed**. |

`lib/utils/frontend_protocol.hpp` and `frontend_gui` keep compiling for the old applet. `ccu` does not link them.

---

## Link sub-gates

The tablet and the Pi join the phone hotspot `willys_little_chicken`. The ESP32-C3 dongles stay off this link. H2.5 and H2.6 are not run. H2.0 through H2.4 stay as passed host tests of the frame bytes. The orientation bench on the H2 row is the current gate.

The tablet USB-C does one job at a time. In the PC, it takes a new app build. In the Wi-Fi dongle, it talks to the CCU. A gate that needs the other lead waits until that cable is moved. H2.0 through H2.4 leave the tablet lead where it is. H2.5 uses a dongle on the Pi and the other dongle on this PC at COM9.


| Gate | Proves                                                                                                                                                                                                                                                   | Run                                                   |
| ---- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------- |
| H2.0 | `ccu_h2_0` writes 16 raw bytes on a localhost socket. `H2BytePipeTest` reads those bytes unchanged and writes them back. The payload contains `0x00` and `0xE4` `0xE5` `0xE6`, so a text filter or a dongle-local filter cannot pass. Passed on this PC. | `.\scripts\h2-0-bytes.ps1`                            |
| H2.1 | The same socket carries one `DEVICE_ID` from `pack_device_id`. `SerialFrameParser` yields display name **Smart Marker** and an idle session. Passed on this PC.                                                                                          | `.\scripts\h2-1-device-id.ps1`                        |
| H2.2 | Injected dongle bytes, no serial port. `"ready"` clears paired. `"paired"` sets it. A tablet frame before paired is dropped. `DEVICE_ID` is withheld until paired, then the bytes match `pack_device_id`. Passed on this PC.                             | `.\scripts\h2-2-dongle.ps1`                           |
| H2.3 | Tablet open emits `DONGLE_RESET`, then accepts a following `DEVICE_ID`. A preceding `DONGLE_STATUS` is consumed and is not a device advertisement. Passed on this PC.                                                                                    | `.\scripts\h2-3-tablet-open.ps1`                      |
| H2.4 | With paired injected, the idle session loop produces `DEVICE_ID` on the pipe. The tablet parser is the reader. Passed on this PC.                                                                                                                        | `.\scripts\h2-4-idle.ps1`                             |
| H2.5 | One dongle on the Pi, the other on the PC at COM9. The same `scripts/h2-5.py` runs on both. Each side sends the H2.0 payload inside one forwarded frame and must receive that payload from the other side. No session and no tablet UI.                  | `python3 scripts/h2-5.py` and `py -3 scripts/h2-5.py` |
| H2.6 | `ccu` idle on the Pi. A reader on the tablet dongle logs `DEVICE_ID` with display name **Smart Marker**.                                                                                                                                                 | not started                                           |


H2.0 payload, in order:

```
10 01 02 03 00 FF 7F 80 E4 E5 E6 0A 0D 20 41 00
```

`H2BytePipeTest`, `H2DeviceIdPipeTest`, and `H2IdleSessionPipeTest` skip unless their script passes the executable path, so the rest of the tablet unit tests stay quiet. `H2TabletOpenTest` always runs. Each script's last line is `H2.n PASS` or `H2.n FAIL`.

H2.5 is one dongle on the Pi and the other on this PC at COM9. Both machines run the same `scripts/h2-5.py`. The dongle forwards a whole frame, so the 16 H2.0 bytes are the payload of command `0x7F` with length 16. `0xE4` `0xE5` `0xE6` stay inside that payload. The C3 USB port does not transmit until the host writes a packet, so the script writes an empty status frame first. Copy the tree, then start it on the Pi and on this PC. Either side may start first. Both must print `H2.5 PASS`.

```
.\scripts\deploy-pi.ps1 -Gate H2.5
```

Pi:

```
python3 scripts/h2-5.py
```

PC:

```
py -3 scripts/h2-5.py
```

---

## Order of work

1. Done. `ccu/protocol` and G1, against the layout in [CMD_PROTOCOL.md](CMD_PROTOCOL.md). Kotlin codec and G2, G4, G15–G17.
2. Done. `yaw`, `command_allowed`, the three models, bearing helper. G3, G5–G9.
3. Done. `session` command dispatch and telemetry builder, with `session_reset`. G10, G11, G14.
4. Done. `survey` and `project`, including the function-pointer range read. G12, G13.
5. POSIX `serial.cpp` and the four port files. `session_poll` calls them. `ccu_ports.cfg` is already the Pi port list. Firmware interval and stage tau. Flash `esp32-c3-serial`. The board enumerates on the Pi as `/dev/ttyACM0`.
6. Done. Tablet UI: session dialog, pose screen, wedge, survey and project payloads. G15–G17.
7. The tablet and the Pi join the phone hotspot `willys_little_chicken`. The access-point profile `Hotspot` is deleted. H2.0 through H2.4 passed on this PC. H2.5 and H2.6 are not run.
8. Hardware gates H1–H7: `.\scripts\deploy-pi.ps1 -Gate Hn`, then run the printed commands in an SSH session.
9. Elevation-sweep survey. `ccu_survey.cfg`, the lidar command path, frozen yaw, the plane fit, the Node store, `SURVEY_RESULT` on the link, and idle when the job finishes. Re-run G7, G12, and G13. Then H8.1–H8.8, H9, and H10.

---

## Video script (H10)

1. Device on the floor at the plan corner, level, pointed due north. Tablet disconnected, then connected.
2. **Start session.** Pose screen: **Set Origin.**
3. Tap **Set Origin.** **Pose Identified.** Open the plan. Wedge at the corner, pointing up the west edge.
4. Turn the base until two vertices enter the wedge.
5. Survey them. Leave the base still. Each vertex is swept low to high, the gimbal dwells, and the overlay advances. Both show **Surveyed**.
6. Project them. The naive corners appear on the floor. Pause, play, end.

