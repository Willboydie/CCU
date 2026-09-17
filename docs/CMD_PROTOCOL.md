# CCU ↔ frontend command protocol

Shared command-byte catalog for the CCU and the frontend. Every packet and command on this link uses a unique `cmd` byte. Payload field lists here are the PoC contract; binary packing follows the frame format below.

Related: [POC_WORKFLOW.md](POC_WORKFLOW.md) (session / connection flow), [WIFI_DONGLE_HOSTING.md](WIFI_DONGLE_HOSTING.md) (USB-C ESP-NOW hop).

**Never use `0xE5`.** The Wi-Fi dongle reserves it for chip-to-chip pairing and will not forward it.

---

## Frame format

Every message is one frame. Little-endian.

```
 offset  size  field
 0       1     cmd        u8    identity of this packet / command (this document)
 1       2     length     u16   payload byte count (not including this header)
 3       N     payload    N = length
```

Total size = `3 + length`. On an unknown `cmd`, drop the frame.

### Ranges

| Range | Direction | Kind |
| --- | --- | --- |
| `0x01`–`0x0F` | device → frontend | packets |
| `0x10`–`0x1F` | frontend → device | packets |
| `0x20`–`0x3F` | frontend → device | commands |
| `0xE5` | — | **reserved** (dongle pairing; not a protocol message) |

---

## Command bytes (unique)

### Device → frontend packets

| cmd | Name | When |
| --- | --- | --- |
| `0x01` | `DEVICE_ID` | Heartbeat while disconnected |
| `0x02` | `PAIRING_ACCEPTED` | Device accepted a join / begin-session request |
| `0x03` | `REGULAR_TELEMETRY` | 10 Hz while In Session |
| `0x04` | `SURVEY_RESULT` | Once per surveyed node (final frame has `surveyFinished = true`) |

### Frontend → device packets

| cmd | Name | When |
| --- | --- | --- |
| `0x10` | `CONNECTED` | 10 Hz keep-alive while the UI considers itself connected |

### Frontend → device commands

| cmd | Name | When |
| --- | --- | --- |
| `0x20` | `BEGIN_NEW_SESSION` | Start a session on a recognised device |
| `0x21` | `JOIN_SESSION` | Join an already Active session |
| `0x22` | `END_SESSION` | End the named session; wipe session data |
| `0x23` | `BEGIN_ORIGIN_SET` | Enter the Origin Set routine (initial tag registration) |
| `0x24` | `BEGIN_TAG_FINDING` | Enter relocation Tag Finding (`originIsSet` must be `true`) |
| `0x25` | `PERFORM_SURVEY` | Survey the listed nodes |
| `0x26` | `ABORT_SURVEY` | Cancel an in-progress survey |
| `0x27` | `PERFORM_PROJECTION` | Project the listed nodes |
| `0x28` | `PROJECTOR_PAUSE` | Pause an active projection |
| `0x29` | `PROJECTOR_PLAY` | Resume a paused projection |
| `0x2A` | `PROJECTION_END` | Discard the path and disable the projector |
| `0x2B` | `SET_LASER_POWER` | Set laser PWM duty |
| `0x2C` | `NUDGE_X` | Steer the beam (Origin Set / Tag Finding / projection) |
| `0x2D` | `NUDGE_Y` | Steer the beam (Origin Set / Tag Finding / projection) |
| `0x2E` | `TAG_ALIGNED` | Operator aligned the beam on the named tag |

---

## Device → frontend

### `0x01` `DEVICE_ID`

Heartbeat while the device is disconnected. Stops when the device latches Connected.

| Field | Type | Notes |
| --- | --- | --- |
| `deviceId` | string | Stable device identity |
| `displayName` | string | UI label |
| `sessionActive` | bool | `true` if a session is already Active (UI may `JOIN_SESSION`) |
| `sessionId` | string | Current session id; empty / unused when not Active |

### `0x02` `PAIRING_ACCEPTED`

Sent when the device accepts `BEGIN_NEW_SESSION` or `JOIN_SESSION`. The device then latches Connected and stops `DEVICE_ID` heartbeats.

| Field | Type | Notes |
| --- | --- | --- |
| `deviceId` | string | Echo of the paired device |

### `0x03` `REGULAR_TELEMETRY`

10 Hz while In Session, whether or not the frontend is connected. The frontend ignores a frame whose `deviceId` / `sessionId` do not match; ignored frames do not count toward connected status.

| Field | Type | Notes |
| --- | --- | --- |
| `deviceId` | string | |
| `sessionId` | string | |
| `originIsSet` | bool | `false` until Origin Set completes; `false` again on session end. While `false`, the CCU performs no work functions (no IMU relocation monitoring, Tag Finding, survey, or projection). Origin Set itself is the exception. |
| `lastKnownPose` | Pose | Last registered pose. Origin Pose after Origin Set. |
| `effectiveYaw` | float | `[0, 2π)` — yaw about up, from `lastKnownPose` and base rotation |
| `moved` | bool | Latches true on IMU motion that is not base rotation. Stays `false` while `originIsSet = false` (IMU relocation monitoring is off). |
| `originSetStatus` | OriginSetStatus | See enumerations. Distinct from `tagFindingStatus`. |
| `tagFindingStatus` | TagFindingStatus | See enumerations. Relocation Tag Finding only; ignore while `originIsSet = false`. |
| `surveyStatus` | SurveyStatus | See enumerations |
| `projectorStatus` | ProjectorStatus | See enumerations |
| `projectorRunning` | bool | Follows `PROJECTOR_PLAY` / `PROJECTOR_PAUSE`. Always `false` when `projectorStatus ≠ PROJECTING` |
| `laserOn` | bool | Emission status |
| `laserPower` | uint8 | Duty cycle, **15–99** |

### `0x04` `SURVEY_RESULT`

One frame per node attempted during `PERFORM_SURVEY`. The last frame of a job has `surveyFinished = true`.

| Field | Type | Notes |
| --- | --- | --- |
| `node` | Node | Includes `upMm` (float) and `surveyed` (bool) |
| `success` | bool | |
| `errorCode` | uint8 | Meaningful when `success = false` |
| `surveyFinished` | bool | `true` on the final result of this survey |

---

## Frontend → device

### `0x10` `CONNECTED`

Keep-alive. Sent at 10 Hz while the UI has seen `REGULAR_TELEMETRY` in the last 5 s. After 5 s without this packet, the device latches disconnected and resumes `DEVICE_ID`. Disconnect does not end the session.

| Field | Type | Notes |
| --- | --- | --- |
| `tabletId` | string | |

### `0x20` `BEGIN_NEW_SESSION`

Start a new session. No payload.

### `0x21` `JOIN_SESSION`

Join an Active session advertised in `DEVICE_ID`.

| Field | Type | Notes |
| --- | --- | --- |
| `sessionId` | string | Must match the advertised session |

### `0x22` `END_SESSION`

End the session. Internal session data is wiped and defaults reinstated.

| Field | Type | Notes |
| --- | --- | --- |
| `sessionId` | string | |

### `0x23` `BEGIN_ORIGIN_SET`

Enter the Origin Set macro routine. Sent when the operator taps **Set Origin**. No payload.

Accepted while `originIsSet = false`. This is the only work routine the CCU will run in that state. The UI and CCU treat Origin Set separately from relocation Tag Finding (`BEGIN_TAG_FINDING`). See `MACRO_ROUTINES.md`.

### `0x24` `BEGIN_TAG_FINDING`

Enter the relocation Tag Finding macro routine. No payload.

Accepted only while `originIsSet = true`. Used when the operator repeats tag alignment after the device is moved. Do not use this command for initial site registration — that is `BEGIN_ORIGIN_SET`.

### `0x25` `PERFORM_SURVEY`

| Field | Type | Notes |
| --- | --- | --- |
| `nodesToSurvey` | Node `[N]` | Nodes of interest |

### `0x26` `ABORT_SURVEY`

Cancel an in-progress survey. No payload.

### `0x27` `PERFORM_PROJECTION`

| Field | Type | Notes |
| --- | --- | --- |
| `nodesToProject` | Node `[N]` | Nodes of interest |

### `0x28` `PROJECTOR_PAUSE`

Pause an active projection. No payload. `projectorRunning` becomes `false`.

### `0x29` `PROJECTOR_PLAY`

Resume a paused projection. No payload. `projectorRunning` becomes `true` while `projectorStatus = PROJECTING`.

### `0x2A` `PROJECTION_END`

Discard the projection path and disable the projector. No payload.

### `0x2B` `SET_LASER_POWER`

| Field | Type | Notes |
| --- | --- | --- |
| `laserDutyCycle` | uint8 | Duty cycle, **15–99** |

### `0x2C` `NUDGE_X`
| Field | Type | Notes |
| --- | --- | --- |
| `dx` | int16 | DAC increment of the X mirror. |

### `0x2D` `NUDGE_Y`
| Field | Type | Notes |
| --- | --- | --- |
| `dy` | int16 | DAC increment of the Y mirror. |

### `0x2E` `TAG_ALIGNED`

Used by both Origin Set and relocation Tag Finding.

| Field | Type | Notes |
| --- | --- | --- |
| `tagId` | string | `Tag.id` of the aligned tag |

---

## Shared types

CCU and UI share these objects. They are the in-memory contract for tag geometry; `TAG_ALIGNED` carries only `tagId` (the CCU fills angles from the current beam).

### `Angle`

Azimuth and elevation of the beam to a tag, as a true point source at the middle of the lower galvo mirror.

| Field | Type | Notes |
| --- | --- | --- |
| `azimuth` | float | |
| `elevation` | float | |

### `Tag`

| Field | Type | Notes |
| --- | --- | --- |
| `id` | string | Stable tag identity |
| `newAngle` | Angle | Latest alignment (relocation Tag Finding) |
| `prevAngle` | Angle | Reference alignment (written during Origin Set; previous `newAngle` after a later alignment) |

---

## Enumerations

These are payload enums, **not** command bytes. Values may overlap cmd bytes; they live in typed fields.

### `OriginSetStatus` (`uint8`)

Origin Set only. The UI must not reuse these values as relocation Tag Finding state.

| Value | Name | Meaning |
| --- | --- | --- |
| `0` | `IDLE` | Origin Set is not running |
| `1` | `WAITING_FOR_TAG` | Origin Set is active; operator is aligning a tag |

### `TagFindingStatus` (`uint8`)

Relocation Tag Finding only (`BEGIN_TAG_FINDING`). Meaningful when `originIsSet = true`.

| Value | Name | Meaning |
| --- | --- | --- |
| `0` | `POSE_KNOWN` | Pose is known |
| `1` | `COMPUTING` | Solving; UI shows a loading screen |
| `2` | `WAITING_FOR_TAG` | Relocation Tag Finding routine is active |

### `SurveyStatus` (`uint8`)

| Value | Name | Meaning |
| --- | --- | --- |
| `0` | `IDLE` | Not surveying |
| `1` | `SCANNING` | Looking for a node on the terrain |

### `ProjectorStatus` (`uint8`)

| Value | Name | Meaning |
| --- | --- | --- |
| `0` | `IDLE` | Not projecting |
| `1` | `COMPUTING_PATH` | Building the projection pathway |
| `2` | `PROJECTING` | Path is active (play/pause via `projectorRunning`) |

---

## Copy-paste catalog

```
// Device → frontend packets
DEVICE_ID            = 0x01
PAIRING_ACCEPTED     = 0x02
REGULAR_TELEMETRY    = 0x03
SURVEY_RESULT        = 0x04

// Frontend → device packets
CONNECTED            = 0x10

// Frontend → device commands
BEGIN_NEW_SESSION    = 0x20
JOIN_SESSION         = 0x21
END_SESSION          = 0x22
BEGIN_ORIGIN_SET     = 0x23
BEGIN_TAG_FINDING    = 0x24
PERFORM_SURVEY       = 0x25
ABORT_SURVEY         = 0x26
PERFORM_PROJECTION   = 0x27
PROJECTOR_PAUSE      = 0x28
PROJECTOR_PLAY       = 0x29
PROJECTION_END       = 0x2A
SET_LASER_POWER      = 0x2B
NUDGE_X              = 0x2C
NUDGE_Y              = 0x2D
TAG_ALIGNED          = 0x2E

// Reserved
DONGLE_PAIRING       = 0xE5   // do not use
```
