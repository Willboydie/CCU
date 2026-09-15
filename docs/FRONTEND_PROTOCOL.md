# Smart Marker — tablet ↔ device protocol

This is the wire contract between the **Android tablet app** and **marker firmware**, forwarded by the USB-C ESP-NOW dongle. It is written for the firmware author. The tablet’s Kotlin types live in `app/.../data/DeviceProtocol.kt`. The field list this spec implements is `docs/data_packets.txt`.

Peak throughput on the USB-C hop is about **10 KB/s**. ESP-NOW payloads are at most **250 bytes**. Keep every frame at or under 250 bytes including the header, unless the dongle fragments and reassembles logical frames transparently.

Protocol version: **1**.

---



## 1. Frame format

Every message is one frame. Little-endian. No checksum (USB is reliable; the dongle must retry ESP-NOW).

```
 offset  size  field
 0       1     cmd
 1       2     length     uint16 — payload byte count (not including this header)
 3       N     payload    N = length
```

Total size = `3 + length`. Parser: read 3 bytes, then read `length` bytes. One USB write / one ESP-NOW datagram should contain exactly one frame. On an unknown `cmd` or a `length` that does not match the payload layout, drop the frame and stay in the current state.

### Primitive types


| Type   | Size | Encoding                                                    |
| ------ | ---- | ----------------------------------------------------------- |
| `u8`   | 1    | unsigned                                                    |
| `u16`  | 2    | little-endian                                               |
| `i16`  | 2    | little-endian, two’s complement                             |
| `f32`  | 4    | IEEE-754 binary32, little-endian                            |
| `bool` | 1    | `0` = false, `1` = true. Any non-zero value is true.        |
| UTF-8  | N    | not NUL-terminated; size is `length` or an inner `u8` count |




### Shared structs

**Pose** (24 bytes) — site millimetres and attitude. Every angle is radians in **[0, 2π)**.

```
 offset  size  field
 0       4     east_mm     f32
 4       4     north_mm    f32
 8       4     up_mm       f32
 12      4     roll_rad    f32
 16      4     pitch_rad   f32
 20      4     yaw_rad     f32
```

`east_mm` / `north_mm` / `up_mm` are millimetres in the plan frame (same as DXF extraction). On the tablet, **+east is right** and **+north is up**. `yaw_rad` is heading clockwise from north (same convention as node orientation). Wrap each of `roll_rad`, `pitch_rad`, and `yaw_rad` into `[0, 2π)` before sending.

**Node** (18 bytes) — a plan **vertex**. Firmware aims at these corners.

```
 offset  size  field
 0       2     id                         u16  tablet-assigned, stable for the loaded plan
 2       4     east_mm                    f32
 6       4     north_mm                   f32
 10      4     corner_inner_angle_rad     f32
 14      4     corner_orientation_rad     f32
```

The tablet maps labels such as `"V01"` to these integer ids. Firmware never sees the string.

Both angles are radians in **[0, 2π)**.

- `corner_inner_angle_rad` is the **acute** angle between the two walls that meet at the vertex (`≤ π/2`; a square corner is `π/2`). Never send the reflex / exterior.
- `corner_orientation_rad` is measured **from north, clockwise, to the first hand** — the incident wall whose bearing clockwise from north is smaller.

```
        N
        ↑
        |  orientation
        | /
        |/  first hand
        ●------ second hand
         \
          inner (acute)
```

The tablet computes both from the loaded plan when it sends `REGISTER_NODES`.

### Enumerations

**ConnectionStatus** (`u8`) — ESP-NOW link between dongle and marker, as the device reports it.


| Value | Name        | Meaning                             |
| ----- | ----------- | ----------------------------------- |
| 0     | `Linked`    | Marker is reachable                 |
| 1     | `Searching` | Dongle/marker is looking for a peer |
| 2     | `Lost`      | Peer was up and is now gone         |


**ProjectionMode** (`u8`) — how the laser is being used.


| Value | Name     | Meaning                                        |
| ----- | -------- | ---------------------------------------------- |
| 0     | `Off`     | Beam idle (may still be dark at the last duty) |
| 1     | `Survey`  | Capture returns on vertices                    |
| 2     | `Project` | Hold / slew a visible projected beam           |


---



## 2. Command ids

Device → tablet: `0x01`–`0x0F`. Tablet → device: `0x10`–`0x1F`.


| cmd    | Name                  | Direction       | When                                          |
| ------ | --------------------- | --------------- | --------------------------------------------- |
| `0x01` | `FIRMWARE_VERSION`    | device → tablet | Once, on successful connection                |
| `0x02` | `TELEMETRY`           | device → tablet | Periodic, while linked                        |
| `0x03` | `SURVEY_POINT`        | device → tablet | Once per requested vertex during a survey     |
| `0x04` | `PROJECTION_START`    | device → tablet | Once, when a projection job is accepted       |
| `0x10` | `SET_LASER_POWER`     | tablet → device | Operator changes brightness                   |
| `0x11` | `SET_PROJECTION_MODE` | tablet → device | Mode / halt                                   |
| `0x12` | `BEGIN_TAG_SEARCH`    | tablet → device | Pose screen: begin Tag Finding (or the next tag) |
| `0x13` | `BEGIN_SURVEY`        | tablet → device | Operator starts a survey of selected nodes    |
| `0x14` | `BEGIN_PROJECTION`    | tablet → device | Operator starts projecting selected nodes     |
| `0x15` | `TAG_FOUND`           | tablet → device | Operator confirms the current tag on the pad  |
| `0x16` | `NUDGE_BEAM_X`        | tablet → device | D-pad azimuth step                            |
| `0x17` | `NUDGE_BEAM_Y`        | tablet → device | D-pad elevation step                          |
| `0x18` | `SET_ORIGIN`          | tablet → device | Optional: force origin / pose (not the Pose CTA) |
| `0x19` | `REGISTER_NODES`      | tablet → device | Connection, or a new plan is opened           |


There are no command-ack packets. The tablet infers success or rejection from the next `TELEMETRY` (and from `SURVEY_POINT` / `PROJECTION_START` where those apply).

---



## 3. Device program states

Firmware keeps one **mode** plus a few **flags**. Telemetry reports the flags every tick; the tablet UI is driven from those flags.

### Modes (mutually exclusive)


| Mode         | `isSurveying` | `isProjecting` | Typical `laserOn` |
| ------------ | ------------- | -------------- | ----------------- |
| `UNLINKED`   | —             | —              | off, no frames    |
| `IDLE`       | 0             | 0              | 0                 |
| `TAG_SEARCH` | 0             | 0              | 1                 |
| `SURVEYING`  | 1             | 0              | 0                 |
| `PROJECTING` | 0             | 1              | 1                 |


`UNLINKED` is “no tablet session”. As soon as the NOW/USB session is up, send `FIRMWARE_VERSION` and enter `IDLE` (even if origin is unset).

### Flags (orthogonal, reported in telemetry)


| Flag          | Set when                                                                                         | Cleared when                                                                                                               |
| ------------- | ------------------------------------------------------------------------------------------------ | -------------------------------------------------------------------------------------------------------------------------- |
| `healthOK`    | Self-test / IMU / laser driver are good                                                          | Fault                                                                                                                      |
| `originIsSet` | `SET_ORIGIN` accepted                                                                            | Power loss. Tablet **Clear origin** is UI-only (no wire command) — stay set until reboot unless you add your own policy    |
| `moved`       | IMU latch: the **base translated or tilted**, not a turret swivel                                | A completed tag pair that refreshes `lastKnownPose`, or `SET_ORIGIN`                                                       |
| `surveyReady` | May accept `BEGIN_SURVEY` (see below)                                                            | Origin missing, `moved`, not `IDLE`, no nodes, `healthOK` false, or link not `Linked`                                      |
| `poseValid`   | `lastKnownPose` is usable (completed tag pair, or `originIsSet`)                                 | `moved`                                                                                                                    |


`surveyReady` is a derived bit. Compute it as:

```
healthOK
&& originIsSet
&& !moved
&& poseValid
&& nodesRegistered
&& mode == IDLE
&& connectionStatus == Linked
```

`effectiveYaw` is `lastKnownPose.yaw_rad` plus the IMU yaw delta since that pose was stored, wrapped to `[0, 2π)`. Turret / base **swivel** updates `effectiveYaw` and does **not** set `moved`. Physical relocation sets `moved` and invalidates pose.

```
                 link up
 UNLINKED ──────────────────► IDLE ── periodic TELEMETRY + FIRMWARE_VERSION once
                                │
                                │ BEGIN_TAG_SEARCH     (IDLE; origin not required)
                                ├──────────────────► TAG_SEARCH
                                │                         │
                                │              TAG_FOUND / timeout
                                │                         │
                                ◄─────────────────────────┘
                                │  two distinct tags → originIsSet, poseValid,
                                │                      lastKnownPose, moved=0
                                │
         SET_ORIGIN             │ BEGIN_SURVEY          (requires surveyReady)
         (optional)             ├──────────────────► SURVEYING
              │                 │                         │
              ▼                 │              SURVEY_POINT (each id)
         originIsSet            │                         │
         poseValid              ◄─────────────────────────┘
         lastKnownPose          │
                                │ BEGIN_PROJECTION
                                ├──────────────────► PROJECTING
                                │                         │
                                │     SET_PROJECTION_MODE(Off)
                                │     or job finished
                                ◄─────────────────────────┘

 SET_PROJECTION_MODE(Off) from TAG_SEARCH / SURVEYING / PROJECTING returns to IDLE
 (survey: emit `SURVEY_POINT` with success=0 for any ids not yet reported).
 moved == true: abort SURVEYING / PROJECTING / TAG_SEARCH → IDLE, laser off.
 Forget locked tag IDs (`poseValid = 0`). The tablet sets every Tag.aligned = false
 and returns the Pose screen to “Click to begin Tag Finding”.
```



### What the device sends in each mode


| Mode           | Device sends                                                          | Device expects from tablet                                                           |
| -------------- | --------------------------------------------------------------------- | ------------------------------------------------------------------------------------ |
| `UNLINKED`     | nothing                                                               | nothing                                                                              |
| `IDLE`         | `TELEMETRY` at 10 Hz                                                  | any command in the accept table below                                                |
| just connected | `FIRMWARE_VERSION` once, then `IDLE`                                  | `REGISTER_NODES`, then `SET_LASER_POWER` for the UI slider                           |
| `TAG_SEARCH`   | `TELEMETRY` (`laserOn` = 1)                                           | `TAG_FOUND`, `NUDGE_BEAM_*`, `SET_LASER_POWER`, `SET_PROJECTION_MODE(Off)` to cancel |
| `SURVEYING`    | `TELEMETRY` (`isSurveying` = 1), plus one `SURVEY_POINT` per requested id | `SET_PROJECTION_MODE(Off)` to cancel; ignore new jobs                                |
| `PROJECTING`   | `PROJECTION_START` immediately, then `TELEMETRY` (`isProjecting` = 1) | `NUDGE_BEAM_*`, `SET_LASER_POWER`, `SET_PROJECTION_MODE(Off)` to halt                |




### Command accept table


| Command               | `IDLE`                       | `TAG_SEARCH`            | `SURVEYING` | `PROJECTING`            |
| --------------------- | ---------------------------- | ----------------------- | ----------- | ----------------------- |
| `SET_LASER_POWER`     | yes                          | yes                     | yes         | yes                     |
| `SET_PROJECTION_MODE` | yes                          | Off cancels             | Off cancels | Off cancels             |
| `BEGIN_TAG_SEARCH`    | yes                          | ignore                  | ignore      | ignore                  |
| `BEGIN_SURVEY`        | if `surveyReady`             | ignore                  | ignore      | ignore                  |
| `BEGIN_PROJECTION`    | if origin + nodes + `!moved` | ignore                  | ignore      | ignore (already in job) |
| `TAG_FOUND`           | ignore                       | yes                     | ignore      | ignore                  |
| `NUDGE_BEAM_X/Y`      | yes                          | yes                     | ignore      | yes                     |
| `SET_ORIGIN`          | yes                          | cancel search, then set | ignore      | ignore                  |
| `REGISTER_NODES`      | yes (replace table)          | ignore                  | ignore      | ignore                  |


“Ignore” means: drop the frame, do not change mode, do not NACK. The tablet will see unchanged flags in `TELEMETRY`.

---



## 4. Device → tablet



### `0x01` `FIRMWARE_VERSION`

**Time:** immediately after the session is up (USB enumerated **and** NOW linked, or as soon as the marker can talk to the dongle that is already on USB). Send once per connection. Do not wait for the tablet to ask.

**Payload:**

```
 offset  size  field
 0       1     protocolVersion    u8  must be 1
 1       N     firmwareVersion    UTF-8, N = length - 1
```

Example: firmware `"1.4.2"` → `length = 6`, bytes `{ 0x01, '1', '.', '4', '.', '2' }`.

**State:** `UNLINKED` → `IDLE`. Start the telemetry ticker after this frame.

**Tablet:** stores the string for Settings → About. Then sends `REGISTER_NODES` for the open plan and `SET_LASER_POWER` for the current brightness.

---



### `0x02` `TELEMETRY`

**Time:** while linked, **10 Hz** recommended (40-byte payload → well under 10 KB/s). Send the first tick right after `FIRMWARE_VERSION`.

**Payload** (40 bytes, `length = 40`):

```
 offset  size  field
 0       1     healthOK            bool
 1       1     moved               bool     IMU latch (relocation, not swivel)
 2       1     originIsSet         bool
 3       1     isSurveying         bool
 4       1     surveyReady         bool
 5       1     isProjecting        bool
 6       1     laserOn             bool
 7       1     connectionStatus    u8       ConnectionStatus
 8       1     projectionMode      u8       ProjectionMode
 9       1     laserPower          u8       0–100 PWM duty
 10      1     poseValid           bool     lastKnownPose is meaningful
 11      1     reserved            u8       write 0
 12      4     effectiveYaw        f32      rad in [0, 2π), pose yaw + IMU swivel
 16      24    lastKnownPose       Pose     ignore fields if poseValid = 0
```

**State:** does not change mode. It **reports** mode and flags. If `moved` becomes true, firmware must leave `TAG_SEARCH` / `SURVEYING` / `PROJECTING` for `IDLE`, turn the laser off, and set `surveyReady = 0`, `poseValid = 0`.

**Tablet expects:**

- `healthOK = 0` → show a fault, do not start jobs.
- `poseValid = 1` → Pose screen: “Pose Identified” (pad hidden). Reposition / Clear origin stay available.
- `poseValid = 0` → Pose screen: “Pose not known: Click to begin Tag Finding”, unless Tag Finding is already in progress (pad visible after `BEGIN_TAG_SEARCH`).
- `moved = 1` → “Device moved: reposition”; pose is stale. Tablet leaves Tag Finding and waits for begin again.
- `originIsSet` is echoed; the tablet does not send a clear-origin frame.
- `isSurveying` / `isProjecting` → run overlays / halt control.
- `effectiveYaw` → FOV wedge on the plan.
- `laserOn` / `laserPower` → header chip and settings slider echo.

---



### `0x03` `SURVEY_POINT`

**Time:** once for **each** node id in the `BEGIN_SURVEY` list, after that vertex has been attempted (about 5 s dwell in the test applet). Send a result even on failure (unknown id, out of FOV, cancelled, or `moved`). Do not clear `isSurveying` until every requested id has had a `SURVEY_POINT`.

**Payload** (27 bytes, `length = 27`):

```
 offset  size  field
 0       2     nodeId      u16
 2       1     success     bool   1 = return captured, 0 = not surveyed
 3       24    pose        Pose   measured pose if success; zeros otherwise
```

`pose` is the measured return (east/north/up in plan mm, roll/pitch/yaw at capture) when `success = 1`.

**State:** stay in `SURVEYING` until the last requested id is reported, then `SURVEYING` → `IDLE`. `isSurveying = 0`. Recompute `surveyReady`. `projectionMode = Off`. `laserOn = 0`.

**Tablet:** stays on the survey loading screen while `isSurveying` is set (or until every requested id has a result). Each packet updates that vertex to Surveyed or Not visible and advances the progress readout (`done / requested`). It does not send an ack.

---



### `0x04` `PROJECTION_START`

**Time:** once, immediately after `BEGIN_PROJECTION` is accepted, **before** the first dwell. Lists the nodes the device will actually project onto (a subset if some ids are unknown or out of FOV).

**Payload:**

```
 offset  size  field
 0       2     count     u16
 2       18n   nodes     Node[n]
```

Positions and corner angles should match the registered plan vertices.

**State:** already `PROJECTING` when this is sent. `isProjecting = 1`, `projectionMode = Project`, `laserOn = 1`. Stay in `PROJECTING` until `SET_PROJECTION_MODE(Off)` or `moved`.

**Tablet:** treats those ids as Projected and shows the projecting overlay.

---



## 5. Tablet → device



### `0x10` `SET_LASER_POWER`

**Time:** Settings brightness slider (tablet range 10–100). Also sent after connect so firmware matches the UI.

**Payload:** `length = 1`

```
 u8 duty    0–100 PWM percent
```

**State:** no mode change. Store duty. Echo it as `laserPower` on the next `TELEMETRY`. If `laserOn` is currently true, apply immediately.

---



### `0x11` `SET_PROJECTION_MODE`

**Time:** tablet wants a mode without a new node list, or wants to **halt** a job.

**Payload:** `length = 1`

```
 u8 mode    ProjectionMode
```

**Effect:**


| Incoming mode | Effect                                                                                                                                       |
| ------------- | -------------------------------------------------------------------------------------------------------------------------------------------- |
| `Off` (0)     | Cancel `TAG_SEARCH` / `SURVEYING` / `PROJECTING` → `IDLE`. If a survey was running, send `SURVEY_POINT` with success=0 for every id not yet reported. Laser off. |
| `Survey` (1)  | Only meaningful in `IDLE`; sets `projectionMode` so a following `BEGIN_SURVEY` is unambiguous. Do not start a job by itself.                 |
| `Project` (2) | Same, for projection. Do not start a job by itself.                                                                                          |


---



### `0x12` `BEGIN_TAG_SEARCH`

**Time:** Pose screen, when pose is not known and the operator taps **Click to begin Tag Finding**. Also sent again after each `TAG_FOUND` until two distinct tags are locked (the pad stays up for the next tag). Origin is **not** required.

**Payload:** empty (`length = 0`). Which tag is being sought is **not** in this packet; it arrives later as `TAG_FOUND`.

**State:** `IDLE` → `TAG_SEARCH`. Laser on, slew/search for a fiducial (or hold a beam for the operator to walk onto a tag). `projectionMode` may stay `Off`. Timeout is firmware-defined (**8 s** in the test applet); on timeout return to `IDLE` with `laserOn = 0`. The tablet keeps the pad visible until `poseValid` or the operator taps Reposition / Clear origin.

**Tablet then sends:** `NUDGE_BEAM_`* while the operator aims, then `TAG_FOUND` from the pad’s align control.

---



### `0x13` `BEGIN_SURVEY`

**Time:** Plan screen, Survey on the current selection. Tablet only sends this when it believes the site is posed and the selected nodes are in view; firmware still enforces `surveyReady`.

**Payload:**

```
 offset  size  field
 0       2     count     u16
 2       2n    ids       u16[n]
```

**State:** `IDLE` and `surveyReady` → `SURVEYING`. `isSurveying = 1`, `surveyReady = 0`, `projectionMode = Survey`, `laserOn = 1`. Walk the list: slew to each registered node, dwell, then send one `SURVEY_POINT`. Unknown ids get `success = 0`. After a result for every requested id, return to `IDLE` (`isSurveying = 0`).

---



### `0x14` `BEGIN_PROJECTION`

**Time:** Plan screen, Project on the current selection (after a survey in the UI, but firmware may still be asked to project any registered id).

**Payload:** same as `BEGIN_SURVEY` (`u16 count` + `u16 ids[n]`).

**State:** `IDLE`, `originIsSet`, `!moved`, nodes registered → `PROJECTING`. Send `PROJECTION_START` with the nodes that will be projected. `isProjecting = 1`, `projectionMode = Project`, `laserOn = 1`. Cycle / dwell on those aim points until halted.

Unknown ids: omit them from `PROJECTION_START`. If the resulting list is empty, do not enter `PROJECTING`; stay `IDLE` and send no `PROJECTION_START`.

---



### `0x15` `TAG_FOUND`

**Time:** during `TAG_SEARCH`, when the operator taps **TAG n ALIGNED** on the Pose pad. `tagID` is the survey tag index on the Pose screen (`1`, `2`, …), **not** a plan node id.

**Payload:** `length = 2`

```
 u16 tagID
```

**State:** `TAG_SEARCH` → `IDLE`. Record the current beam / IMU pose against `tagID`. Laser off. If the device now has a usable site pose (firmware policy: **two distinct tags**), set `lastKnownPose` from the current attitude, `originIsSet = 1`, `poseValid = 1`, `moved = 0`, and recompute `surveyReady`. The tablet then hides the pad and shows **Pose Identified**.

If this frame arrives outside `TAG_SEARCH`, ignore it. After the first lock the tablet immediately sends another `BEGIN_TAG_SEARCH` for the second tag.

---



### `0x16` `NUDGE_BEAM_X` / `0x17` `NUDGE_BEAM_Y`

**Time:** Pose D-pad, and while projecting if the operator trims the spot. Fine step is 0.05°, coarse is 0.5°. The payload is the **delta already scaled** (signed degrees).

**Payload:** `length = 4`

```
 f32 dx    azimuth delta, deg   (X)   positive = CCW as seen from above
 f32 dy    elevation delta, deg (Y)   positive = up
```

**State:** apply to the beam immediately. Does not change mode. Do **not** set `moved` (this is turret motion). `effectiveYaw` may change for an X nudge if azimuth is the base/turret yaw. Ignore during `SURVEYING` so a capture is not disturbed.

---



### `0x18` `SET_ORIGIN`

**Time:** optional. The Pose screen’s primary path is Tag Finding, not this command. The test applet can still inject it. The tablet ViewModel still knows how to send it, but the UI does not.

**Payload:** `length = 24` — one `Pose`. A typical inject is `(east = 0, north = 0, up = 0, roll = 0, pitch = 0, yaw = …)`. Prefer the IMU attitude at receipt if it is more accurate; then echo that in `lastKnownPose`.

**State:** from `IDLE` or `TAG_SEARCH` (search is cancelled). Set origin, `originIsSet = 1`, `lastKnownPose =` that pose, `poseValid = 1`, `moved = 0`, `effectiveYaw =` that yaw (wrapped to `[0, 2π)`). Recompute `surveyReady`. Because `poseValid` becomes 1, a connected tablet treats pose as identified and hides the Tag Finding pad.

---



### `0x19` `REGISTER_NODES`

**Time:**

1. After `FIRMWARE_VERSION` on a new connection (open plan).
2. Whenever the operator opens or imports a different plan.

**Payload:**

```
 offset  size  field
 0       2     count     u16
 2       18n   nodes     Node[n]
```

This **replaces** the entire node table. `n = 0` clears it (`surveyReady` becomes 0).

**State:** only in `IDLE`. Drop any previous survey measurements keyed by old ids. Coordinates are plan millimetres; firmware transforms them with `lastKnownPose` / `effectiveYaw` when aiming.

If `2 + 18n > 247`, split across multiple frames using the optional chunk flags in §7. Typical site plans (≤ 13 vertices) fit in one ESP-NOW frame.

---



## 6. Session sequences



### Link up

```
 marker                         tablet
   |-- FIRMWARE_VERSION -------->|
   |-- TELEMETRY (10 Hz) ------->|
   |<-- REGISTER_NODES ----------|
   |<-- SET_LASER_POWER ---------|
   |-- TELEMETRY (originIsSet,    |
        surveyReady, …) -------->|
```



### Tag Finding (identify pose)

```
   |<-- BEGIN_TAG_SEARCH --------|     TAG_SEARCH, laser on
   |<-- NUDGE_BEAM_X/Y ----------|     optional (Pose pad)
   |<-- TAG_FOUND (tag 1) -------|     IDLE
   |<-- BEGIN_TAG_SEARCH --------|     TAG_SEARCH again for the next tag
   |<-- TAG_FOUND (tag 2) -------|     originIsSet=1, poseValid=1, moved=0
   |-- TELEMETRY poseValid=1 --->|     tablet: “Pose Identified”, pad hidden
```

`SET_ORIGIN` is not part of this path. If it is sent, firmware sets `poseValid` immediately and the tablet skips the pad.



### Survey then project

```
   |<-- BEGIN_SURVEY [ids] ------|     SURVEYING
   |-- TELEMETRY isSurveying=1 ->|
   |-- SURVEY_POINT (id, ok) --->|     one frame per requested id (~5 s)
   |-- SURVEY_POINT … ---------->|
   |-- TELEMETRY isSurveying=0 ->|     IDLE after the last result
   |<-- BEGIN_PROJECTION [ids] --|
   |-- PROJECTION_START -------->|     PROJECTING
   |-- TELEMETRY isProjecting=1 >|
   |<-- SET_PROJECTION_MODE Off -|     halt → IDLE
```



### Relocation

IMU sets `moved = 1` → abort job, laser off, `surveyReady = 0`, `poseValid = 0`, every survey tag `aligned = false`. Telemetry is enough for pose. If a survey was running, send `SURVEY_POINT` with success=0 for each id not yet reported, then `isSurveying = 0`. Operator taps Reposition (or **Click to begin Tag Finding**) and runs Tag Finding again.

---



## 7. ESP-NOW size (RegisterNodes chunks)

ESP-NOW max payload is 250 bytes → max `length` = 247 if the dongle does not reassemble.

`REGISTER_NODES` with a leading flags byte is allowed so large plans can be split. If you use chunks, **all** `REGISTER_NODES` frames on that firmware build include the flags byte:

```
 offset  size  field
 0       1     flags     u8   bit0 = 1 start (clear table first)
                              bit1 = 1 last chunk (commit)
 1       2     count     u16  nodes in this chunk
 3       18n   nodes     Node[n]
```

Start+last in one frame is a normal small plan. Until `last` is set, do not use the table for aiming. `SURVEY_POINT` is 27 bytes and always fits. `BEGIN_SURVEY` / `BEGIN_PROJECTION` / `PROJECTION_START` for v1 are assumed to fit in one 250-byte datagram (about 120 ids, or ~13 projection nodes). Larger jobs are out of scope until the dongle reassembles.

---



## 8. Implementation notes

- Pack by hand or with `__attribute__((packed))`. Do not send C structs with padding.
- Node ids are unsigned 16-bit. Do not reuse an id for a different vertex while that plan is loaded.
- `laserOn` is owned by firmware (search / survey / project). There is no separate laser-on command. Duty is owned by the tablet via `SET_LASER_POWER`.
- Pose is identified by Tag Finding (`BEGIN_TAG_SEARCH` + two `TAG_FOUND`), not by a prior `SET_ORIGIN`. `SET_ORIGIN` remains on the wire as an optional force-pose.
- Tablet **Reposition** / **Clear origin** send no frames. A live `TAG_SEARCH` ends on timeout (8 s in the applet) or `SET_PROJECTION_MODE(Off)`.
- `connectionStatus` is the NOW hop, not USB. USB drop looks like silence on the tablet side.
- Coordinate units are millimetres. Attitude and node-corner angles are radians in `[0, 2π)`.
- On the tablet plan view, +east is right and +north is up.
- v1 has no CRC and no sequence number. If you need them later, add a new `cmd` rather than changing these layouts.

