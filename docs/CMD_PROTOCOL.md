# CCU ↔ frontend command protocol

Shared command-byte catalog for the CCU and the frontend. Every packet and command on this link uses a unique `cmd` byte. Each payload section below is the byte layout.

Related: [POC_WORKFLOW.md](POC_WORKFLOW.md) (session / connection flow, including the promo build), [MACRO_ROUTINES.md](MACRO_ROUTINES.md) (work routines, including the promo build), [WIFI_DONGLE_HOSTING.md](WIFI_DONGLE_HOSTING.md) (USB-C ESP-NOW hop).

**Never send `0xE4`, `0xE5`, or `0xE6`.** The dongle consumes those locally (`DONGLE_RESET`, chip-to-chip pairing, `DONGLE_STATUS`).

---

## Frame format

Every message is one frame. Little-endian.

```
 offset  size  field
 0       1     cmd        u8    identity of this packet / command (this document)
 1       2     length     u16   payload byte count (not including this header)
 3       N     payload    N = length
```

Total size = `3 + length`. On an unknown `cmd`, drop the frame. On a payload whose bytes do not walk exactly to `length`, drop the frame.

### Primitive types

Little-endian. Floats are IEEE-754 binary32.

| Type | Size | Encoding |
| --- | --- | --- |
| `u8` / `bool` | 1 | Bool is `0` or `1`. Any non-zero byte is true on receive. |
| `u16` | 2 | unsigned |
| `i16` | 2 | two's complement |
| `f32` | 4 | binary32 |
| `string` | 1 + N | `u8` byte count, then that many UTF-8 bytes. Not NUL-terminated. A count above **32** drops the frame. |

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
| `0x05` | `PROJECTION_READY` | Once when a projection is accepted, before the first point |

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
| `0x23` | `BEGIN_ORIGIN_SET` | Set origin. Promo build: one press, no tag walk. See the promo section in `MACRO_ROUTINES.md`. |
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
| `0x2E` | `TAG_ALIGNED` | Projector on the named tag; LiDAR ranges at the same pointing `Angle` (offset origin) |

---

## Device → frontend

### `0x01` `DEVICE_ID`

Heartbeat while the device is disconnected. Stops when the device latches Connected. Promo build sends this at **2 Hz**.

Bytes, in order: `deviceId` string, `displayName` string, `sessionActive` bool, `sessionId` string.

| Field | Type | Notes |
| --- | --- | --- |
| `deviceId` | string | Stable device identity. Promo build: `"sm-ccu"`. |
| `displayName` | string | UI label. Promo build: `"Smart Marker"`. |
| `sessionActive` | bool | `true` if a session is already Active (UI may `JOIN_SESSION`) |
| `sessionId` | string | Current session id. Empty while no session is active. Promo build: 8 hex characters when a session starts. |

### `0x02` `PAIRING_ACCEPTED`

Sent when the device accepts `BEGIN_NEW_SESSION` or `JOIN_SESSION`. The device then latches Connected and stops `DEVICE_ID` heartbeats.

Bytes: `deviceId` string.

| Field | Type | Notes |
| --- | --- | --- |
| `deviceId` | string | Echo of the paired device |

### `0x03` `REGULAR_TELEMETRY`

10 Hz while In Session, whether or not the frontend is connected. The frontend ignores a frame whose `deviceId` / `sessionId` do not match; ignored frames do not count toward connected status.

Bytes, in order:

| Field | Type | Notes |
| --- | --- | --- |
| `deviceId` | string | |
| `sessionId` | string | |
| `originIsSet` | bool | `false` until origin is set; `false` again on session end. While `false`, the CCU performs no survey or projection. |
| `lastKnownPose` | Pose | 24 bytes. Promo build writes `(0, 0, DEVICE_HEIGHT, 0, 0, 0)` on every frame. `DEVICE_HEIGHT` is the device-centre height in metres from `ccu_survey.cfg` (default `1.276`). |
| `effectiveYaw` | f32 | `[0, 2π)`. Promo build: clockwise from due north, `wrap(northOffset − stage)`. See the promo section in `POC_WORKFLOW.md`. |
| `moved` | bool | Later product: latches true on IMU motion that is not base rotation. Promo build writes `0` on every frame. |
| `originSetStatus` | u8 | `OriginSetStatus`. Promo build writes `IDLE` (`0`) on every frame. |
| `tagFindingStatus` | u8 | `TagFindingStatus`. Promo build writes `POSE_KNOWN` (`0`) on every frame. |
| `surveyStatus` | u8 | `SurveyStatus` |
| `projectorStatus` | u8 | `ProjectorStatus` |
| `projectorRunning` | bool | Follows `PROJECTOR_PLAY` / `PROJECTOR_PAUSE`. Always `false` when `projectorStatus ≠ PROJECTING` |
| `laserOn` | bool | Emission status |
| `laserPower` | u8 | Duty cycle, **15–99**. Promo default before the tablet sends one: **30**. |

### `0x04` `SURVEY_RESULT`

One frame per node attempted during `PERFORM_SURVEY`. Fixed payload, **26 bytes**. The last frame of a job has `surveyFinished = true`.

| Offset | Size | Field | Type | Notes |
| --- | --- | --- | --- | --- |
| 0 | 2 | `id` | u16 | Plan node id |
| 2 | 4 | `east_m` | f32 | |
| 6 | 4 | `north_m` | f32 | |
| 10 | 4 | `up_m` | f32 | Plan up, metres. Device-centre up is `up_m − DEVICE_HEIGHT`. `0` on failure. |
| 14 | 4 | `corner_inner_angle_rad` | f32 | |
| 18 | 4 | `corner_orientation_rad` | f32 | |
| 22 | 1 | `surveyed` | bool | `0` on failure |
| 23 | 1 | `success` | bool | |
| 24 | 1 | `errorCode` | u8 | `SurveyError`. `NONE` when `success = 1`. |
| 25 | 1 | `surveyFinished` | bool | `true` on the final result of this survey |

### `0x05` `PROJECTION_READY`

Sent once when `PERFORM_PROJECTION` is accepted, before projection streaming begins. Lists the nodes that will be drawn, then the nodes that were left out.

Bytes: `count` u16, then `count` `PlanNode` records, then `errorCount` u8, then `errorCount` pairs of `{ id u16, errorCode u8 }`.

| Field | Type | Notes |
| --- | --- | --- |
| `count` | u16 | Nodes that will be projected |
| `nodes` | PlanNode `[count]` | |
| `errorCount` | u8 | Nodes omitted from the path |
| `errors` | `{ id u16, errorCode u8 } [errorCount]` | `ProjectionError` |

---

## Frontend → device

### `0x10` `CONNECTED`

Keep-alive. Sent at 10 Hz while the UI has seen `REGULAR_TELEMETRY` in the last 5 s. After 5 s without this packet, the device latches disconnected and resumes `DEVICE_ID`. Disconnect does not end the session.

Bytes: `tabletId` string. Promo build: `"pad"`.

| Field | Type | Notes |
| --- | --- | --- |
| `tabletId` | string | |

### `0x20` `BEGIN_NEW_SESSION`

Start a new session. No payload.

### `0x21` `JOIN_SESSION`

Join an Active session advertised in `DEVICE_ID`.

Bytes: `sessionId` string.

| Field | Type | Notes |
| --- | --- | --- |
| `sessionId` | string | Must match the advertised session |

### `0x22` `END_SESSION`

End the session. Internal session data is wiped and defaults reinstated.

Bytes: `sessionId` string.

| Field | Type | Notes |
| --- | --- | --- |
| `sessionId` | string | |

### `0x23` `BEGIN_ORIGIN_SET`

Sent when the operator taps **Set Origin**. No payload.

The promo build accepts this while the session is running, the work mode is idle, and `originIsSet = false`. That build is one press and does not align tags. See the promo section in [MACRO_ROUTINES.md](MACRO_ROUTINES.md) and [POC_WORKFLOW.md](POC_WORKFLOW.md).

The later product, described in those same documents after the promo section, uses this command to start tag registration (`originSetStatus = WAITING_FOR_TAG`). That product is not this build.

### `0x24` `BEGIN_TAG_FINDING`

Enter the relocation Tag Finding macro routine. No payload.

Later product: accepted only while `originIsSet = true`, after the device is moved. The promo build drops this command in every mode.

### `0x25` `PERFORM_SURVEY`

Bytes: `count` u16, then `count` `PlanNode` records (18 bytes each). There is no separate register-nodes command. The tablet sends the vertices in this payload.

| Field | Type | Notes |
| --- | --- | --- |
| `count` | u16 | |
| `nodesToSurvey` | PlanNode `[count]` | Nodes of interest |

### `0x26` `ABORT_SURVEY`

Cancel an in-progress survey. No payload.

### `0x27` `PERFORM_PROJECTION`

Bytes: same as `PERFORM_SURVEY` (`count` u16, then `count` `PlanNode` records).

| Field | Type | Notes |
| --- | --- | --- |
| `count` | u16 | |
| `nodesToProject` | PlanNode `[count]` | Nodes of interest |

### `0x28` `PROJECTOR_PAUSE`

Pause an active projection. No payload. `projectorRunning` becomes `false`.

### `0x29` `PROJECTOR_PLAY`

Resume a paused projection. No payload. `projectorRunning` becomes `true` while `projectorStatus = PROJECTING`.

### `0x2A` `PROJECTION_END`

Discard the projection path and disable the projector. No payload.

### `0x2B` `SET_LASER_POWER`

Bytes: `laserDutyCycle` u8. A value outside **15–99** drops the frame.

| Field | Type | Notes |
| --- | --- | --- |
| `laserDutyCycle` | u8 | Duty cycle, **15–99** |

### `0x2C` `NUDGE_X`

Bytes: `dx` i16. The promo tablet has no control that sends this. The promo CCU accepts it only while projecting.

| Field | Type | Notes |
| --- | --- | --- |
| `dx` | i16 | DAC increment of the X mirror. |

### `0x2D` `NUDGE_Y`

Bytes: `dy` i16. Same accept rule as `NUDGE_X`.

| Field | Type | Notes |
| --- | --- | --- |
| `dy` | i16 | DAC increment of the Y mirror. |

### `0x2E` `TAG_ALIGNED`

Later product: the projector is already on the tag. The CCU commands the LiDAR to the same pointing `Angle` (`GalvoModel` / `LidarModel`); origins stay offset. Recorded distance is from the **device centre**, using the rangefinder origin offset owned by `LidarModel`.

The promo build drops this command in every mode.

Bytes: `tagId` string.

| Field | Type | Notes |
| --- | --- | --- |
| `tagId` | string | `Tag.id` of the aligned tag |

---

## Shared types

### `Pose` (24 bytes)

Site metres and attitude. Angles are radians. The promo build writes `(0, 0, DEVICE_HEIGHT, 0, 0, 0)` in telemetry. `yaw_rad` on a stored pose is not the live heading; live heading is `effectiveYaw`.

| Offset | Size | Field | Type |
| --- | --- | --- | --- |
| 0 | 4 | `east_m` | f32 |
| 4 | 4 | `north_m` | f32 |
| 8 | 4 | `up_m` | f32 |
| 12 | 4 | `roll_rad` | f32 |
| 16 | 4 | `pitch_rad` | f32 |
| 20 | 4 | `yaw_rad` | f32 |

### `PlanNode` (18 bytes)

A plan vertex. Carried by `PERFORM_SURVEY`, `PERFORM_PROJECTION`, and `PROJECTION_READY`. The tablet maps a label such as `"V01"` to `id = 1`. Firmware never sees the string.

| Offset | Size | Field | Type |
| --- | --- | --- | --- |
| 0 | 2 | `id` | u16 |
| 2 | 4 | `east_m` | f32 |
| 6 | 4 | `north_m` | f32 |
| 10 | 4 | `corner_inner_angle_rad` | f32 |
| 14 | 4 | `corner_orientation_rad` | f32 |

Both angles are radians in `[0, 2π)`. `corner_inner_angle_rad` is the acute angle between the two walls (`≤ π/2`). `corner_orientation_rad` is clockwise from north to the first hand.

### `Angle`

Azimuth and elevation of the projector beam from the **top** galvo mirror (beam-exit mirror). Not from the device centre (bottom-mirror middle).

| Field | Type | Notes |
| --- | --- | --- |
| `azimuth` | float | |
| `elevation` | float | |

### `Tag`

Distances are from the **device centre** (bottom-mirror middle) to the tag. `LidarModel` applies the rangefinder origin offset it owns — not raw LiDAR range, not from the top galvo mirror.

| Field | Type | Notes |
| --- | --- | --- |
| `id` | string | Stable tag identity |
| `newAngle` | Angle | Latest projector alignment (relocation Tag Finding) |
| `prevAngle` | Angle | Reference projector alignment (written during Origin Set; previous `newAngle` after a later alignment) |
| `newDistance` | float | Latest range from the device centre (relocation Tag Finding) |
| `prevDistance` | float | Reference range from the device centre (written during Origin Set; shifts with `newAngle` → `prevAngle`) |

---

## Enumerations

These are payload enums, **not** command bytes. Values may overlap cmd bytes; they live in typed fields.

### `SurveyError` (`uint8`)

On `SURVEY_RESULT.errorCode`. `NONE` when `success = 1`.

| Value | Name | Meaning |
| --- | --- | --- |
| `0` | `NONE` | Success |
| `1` | `NOT_VISIBLE` | No range in the dwell window |
| `2` | `OUT_OF_FOV` | Node is outside the wedge or the 13 m range |
| `3` | `ABORTED` | Survey cancelled, or the session ended mid-job |

### `ProjectionError` (`uint8`)

On `PROJECTION_READY` error pairs.

| Value | Name | Meaning |
| --- | --- | --- |
| `2` | `OUT_OF_FOV` | Node is outside the current wedge |
| `4` | `NOT_SURVEYED` | No stored survey result for this id |

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
PROJECTION_READY     = 0x05

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
