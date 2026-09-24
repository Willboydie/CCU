# Proof-of-concept workflow

High-level operating model for the CCU during a UI-driven session: how a session starts, how the frontend stays connected, when the device is allowed to run macros, and what regular telemetry reports.

Detailed routine behaviour lives in `MACRO_ROUTINES.md`. Packet layouts live in `CMD_PROTOCOL.md`.

---

## Promo build

This is the build in progress. The state diagram and the sections after this one describe the later product: tag registration, relocation tag finding, and an IMU `moved` latch. That text stays. This build uses the rules below instead.

| Rule | Promo behavior |
| --- | --- |
| Set Origin | One press. The operator places the device at the plan origin (DXF `(0, 0)`), levels it, points it due north, and sends `BEGIN_ORIGIN_SET` with an empty payload. The CCU stores the AS5048 stage angle as `northOffset` and sets `originIsSet` immediately. `originSetStatus` stays `IDLE`. There is no tag walk. A second press while origin is set is dropped. |
| Pose | Level, at `(east, north, up) = (0, 0, 0)`. `lastKnownPose` is six zeros on every telemetry frame. |
| `moved` | `false` on every frame. There is no relocation routine. |
| Yaw | `effectiveYaw = wrap(northOffset − stage)`, clockwise from due north, in `[0, 2π)`. At the moment of Set Origin, yaw is 0. Stage samples are the `stage` float in the orientation frame `0xA0`, 10 Hz, from the sensor board's own USB CDC. Roll, pitch, and IMU yaw in that frame are ignored. |
| Tag commands | `BEGIN_TAG_FINDING` and `TAG_ALIGNED` are dropped in every mode. |
| `tagFindingStatus` | `POSE_KNOWN` on every frame. |
| Clear origin | `END_SESSION` clears origin and `northOffset`. A link drop does not. |

`END_SESSION` also clears survey results and any projection path. Power-off does the same, because none of that is stored on disk.

---

## State at a glance

```
Power on
   │
   ▼
Out of Session  ── BEGIN_SESSION / join ──►  In Session
   ▲                                              │
   └──── END_THIS_SESSION / power off ────────────┘

While In Session:
  Disconnected  ←── 5 s without Connected packet ──►  Connected
       │                    (does not end the session)       │
       └── DeviceID heartbeat              RegularTelemetry @ 10 Hz
                                           UI Connected @ 10 Hz

  originIsSet=false  (no IMU, no Tag Finding / survey / projection)
       │
       │  UI Set Origin → BEGIN_ORIGIN_SET
       ▼
  Origin Set  (align every on-site tag → Tag.prevAngle + prevDistance)
       │  originIsSet=true
       ▼
  Running
       └── Tag Finding (on move) / Survey / Projection
```


| Layer   | States                      | What changes it                             |
| ------- | --------------------------- | ------------------------------------------- |
| Session | Out of Session → In Session | `BEGIN_SESSION` (or join an Active session) |
| Session | In Session → Out of Session | `END_THIS_SESSION`, or power off            |
| Link    | Disconnected → Connected    | UI join / begin; then Regular Telemetry     |
| Link    | Connected → Disconnected    | 5 s with no UI `Connected` packet           |
| Work    | `originIsSet` false → true  | Origin Set completes (`BEGIN_ORIGIN_SET`)   |


---



## 1. Sessions

The device starts **Out of Session** when the program first runs.


| Event                       | Effect                                                                          |
| --------------------------- | ------------------------------------------------------------------------------- |
| UI sends `BEGIN_SESSION`    | Session starts. Telemetry is enabled. Work routines stay locked until Origin Set completes. |
| UI sends `END_THIS_SESSION` | Session ends. Internal session data is wiped and defaults are reinstated.       |
| Device power off            | Session ends. Session data is **not** retained in non-volatile memory.          |


A frontend disconnect does **not** end a session. Only an explicit UI end request (or power off) does.

---



## 2. Frontend connection



### Advertising (disconnected)

While disconnected, the device heartbeats a **DeviceID** packet through the Wi-Fi chip. The packet includes:

- `id` string
- `displayName`
- `sessionStatus`

The UI listens on the Wi-Fi dongle for incoming Device IDs. If it recognises one, it can:

- **Join** the current session, if `sessionStatus = Active`
- **Start** a new session otherwise

On a join or begin request, the device sends a PAIRING_ACCEPTED packet, latches **Connected** and stops sending DeviceID packets.

### Keep-alive (connected)


| Side                          | Rule                                                           |
| ----------------------------- | -------------------------------------------------------------- |
| UI considers itself connected | It has received a Regular Telemetry packet in the last **5 s** |
| UI while connected            | Sends `Connected` packets at **10 Hz**                         |
| Device drop to disconnected   | **5 s** with no `Connected` packet from the frontend           |


After disconnect, the device restarts DeviceID heartbeats. The session stays open until the UI ends it.

---



## 3. Origin Setting

While `originIsSet = false`, the CCU performs **no work function**: no IMU relocation monitoring, no relocation Tag Finding, no survey, no projection. Session, connection, and telemetry still run so the UI can drive Origin Set.

Origin Set is a **separate** routine from relocation Tag Finding. The UI pose screen, while `originIsSet = false`, prompts the operator to place the device at the origin Pose `(east=0, north=0, up=0, roll=0, pitch=0, yaw=0)` and register every tag on site.

| Step | What happens |
| ---- | ------------ |
| UI | Operator places the device at the origin, taps **Set Origin**, sends `BEGIN_ORIGIN_SET` |
| CCU | Enters Origin Set (`originSetStatus = WAITING_FOR_TAG`). Laser on, nudges accepted |
| UI | Operator steers the projector onto each on-site tag in turn, then sends `TAG_ALIGNED` with that `Tag.id` |
| CCU | Commands the LiDAR to the same pointing `Angle` as the projector (`GalvoModel` + `LidarModel`; origins stay offset). `LidarModel` maps LiDAR range to distance from the **device centre** using its origin offset. Stores **`prevAngle`** and **`prevDistance`**. No `PoseSolver` |
| CCU | After the last tag of the pass is aligned: `originIsSet = true`, `lastKnownPose` = origin Pose, `originSetStatus = IDLE`. IMU monitoring may start |

The UI must not send `BEGIN_TAG_FINDING` for this pass. That command is relocation-only, after origin is set.

---



## 4. Macro routines

These are flow overviews only. See `MACRO_ROUTINES.md` for the routines themselves.

### Tag Finding (relocation)

Used when the device has been moved, **after** origin is set. Distinct from Origin Set: the UI and CCU must not mix the two. `BEGIN_TAG_FINDING` is ignored while `originIsSet = false`.

The projector laser runs at a low duty cycle. The program responds to UI **Nudges**, which increment the projector beam angle. On **Align**, the LiDAR is commanded to the same pointing `Angle` as the galvo (offset origin; tag radius is larger than the offset). `LidarModel` maps the range to distance from the device centre using the rangefinder origin offset it owns.

Tags already exist from Origin Set (`prevAngle` and `prevDistance` filled). Each alignment writes `newAngle` and `newDistance`. On the first alignment since Origin Set, the Origin Set reference is left in `prevAngle` / `prevDistance`. On later relocations, the previous `newAngle` / `newDistance` become `prevAngle` / `prevDistance`.


| Step | What happens                                                                                       |
| ---- | -------------------------------------------------------------------------------------------------- |
| UI   | Sends `BEGIN_TAG_FINDING`                                                                          |
| CCU  | Enters relocation Tag Finding (`tagFindingStatus = WAITING_FOR_TAG`)                               |
| UI   | Operator steers the projector onto a tag, then sends `TAG_ALIGNED`                                 |
| CCU  | Commands the LiDAR to the same pointing `Angle`; maps range to device-centre distance via `LidarModel` offset; writes `newAngle` / `newDistance` |
| CCU  | Solves pose from `lastKnownPose`, `prevAngle` / `prevDistance`, and `newAngle` / `newDistance`     |
| CCU  | Always asks for **2** tag alignments; may ask for a **third** if the solver cannot narrow the pose |


The UI flow is the same for every tag:

1. Navigate the projector onto the tag
2. Press **Align** (LiDAR ranges at that angle)
3. See **Computing...**
4. Either another tag request, or the **Pose Known** screen



### Survey

Survey takes East–North coordinates, scans the terrain, and returns matching **Up** values.


| Step | What happens                                                                                                        |
| ---- | ------------------------------------------------------------------------------------------------------------------- |
| UI   | Sends `PERFORM_SURVEY` with the Node IDs of interest                                                                |
| CCU  | Checks that it recognises the nodes and that they are in the device FOV                                             |
| CCU  | Looks for each node in turn (`surveyStatus = SCANNING`)                                                             |
| CCU  | Sends a `SURVEY_RESULT` per node: `SUCCESS` + Up value, or `FAILED` + error (e.g. `"Not visible.", "Outwith FOV."`) |
| UI   | Updates the Scan Progress readout as results arrive                                                                 |
| CCU  | Final `SURVEY_RESULT` has `surveyFinished = true`.                                                                  |




### Projection

The system can project a set of nodes at once, or a single node. Nodes appear as standalone corner pieces on the ground.


| Step  | What happens                                                                        |
| ----- | ----------------------------------------------------------------------------------- |
| UI    | Sends `PERFORM_PROJECTION` with the Node IDs of interest                            |
| CCU   | Checks that those nodes have survey results and are in the device FOV               |
| CCU   | Computes the projection pathway (`projectorStatus = COMPUTING_PATH`), then projects |
| UI    | May send `PAUSE` / `PLAY`, or `END`                                                 |
| `END` | Path is discarded and the projector is disabled                                     |


---



## 5. Regular telemetry

While **In Session**, the device transmits `RegularTelemetry` at **10 Hz**, whether or not the frontend is connected. The frontend expects a certain deviceId and sessionId. If they don't match, the packet is ignored. Ignored packets do not count towards the frontend's connected status.


| Field              | Type             | Meaning                                                                                                                                                       |
| ------------------ | ---------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `deviceid`         | string           |                                                                               |
| `sessionId`        | string           |                                                                          |
| `originIsSet`      | bool             | `false` until Origin Set completes; `false` again on session end. While `false`, no IMU relocation monitoring, Tag Finding, survey, or projection |
| `lastKnownPose`    | Pose             | Last registered pose. Origin Pose after Origin Set                                                                                                 |
| `effectiveYaw`     | float            | Yaw about the vertical (up) axis, from `lastKnownPose` and `baseRotationAngle`                                                                     |
| `moved`            | bool             | Latches `false` → `true` when IMU motion is deemed **not** base rotation. Stays `false` while `originIsSet = false`                                |
| `originSetStatus`  | OriginSetStatus  | `IDLE` or `WAITING_FOR_TAG`. Origin Set only; not relocation Tag Finding                                                                            |
| `tagFindingStatus` | TagFindingStatus | `POSE_KNOWN`, `COMPUTING`, or `WAITING_FOR_TAG`. Relocation Tag Finding only (`originIsSet = true`). `COMPUTING` → UI loading screen              |
| `surveyStatus`     | SurveyStatus     | `IDLE` or `SCANNING`                                                                                                                                          |
| `projectorStatus`  | ProjectorStatus  | `IDLE`, `COMPUTING_PATH`, or `PROJECTING`                                                                                                                     |
| `projectorRunning` | bool             | Follows projection `PLAY` / `PAUSE`. Always `false` when `projectorStatus != PROJECTING`                                                                      |
| `laserOn`          | bool             | Laser emission status                                                                                                                                         |
| `laserPower`       | int              | Set duty cycle of the laser                                                                                                                                   |


