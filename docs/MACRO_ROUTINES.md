# Macro routines

Behaviour of the CCU work routines. Command bytes live in `CMD_PROTOCOL.md`. Session and connection flow live in `POC_WORKFLOW.md`.

There are **two Tag Finding variants**. The UI and CCU treat them as separate routines:

| Routine | Command | When | PoseSolver |
| --- | --- | --- | --- |
| **Origin Set** | `BEGIN_ORIGIN_SET` | Initial registration and alignment of every tag on site (`originIsSet = false`) | No |
| **Tag Finding** | `BEGIN_TAG_FINDING` | Repeated after the device is moved (`originIsSet = true`) | Yes |

While `originIsSet = false`, the CCU performs no work function except Origin Set. That includes IMU relocation monitoring (`moved` stays `false`). Survey, projection, and relocation Tag Finding are ignored.

---

## Shared types

CCU and UI use the same `Angle` and `Tag` objects.

### `Angle`

`(azimuth, elevation)` pair: true beam angles to a tag, modelled as a point source at the middle of the lower galvo mirror.

| Field | Type |
| --- | --- |
| `azimuth` | float |
| `elevation` | float |

### `Tag`

Stored by the CCU under each tag `id`.

| Field | Type | Notes |
| --- | --- | --- |
| `id` | string | Stable identity |
| `newAngle` | Angle | Latest alignment from relocation Tag Finding |
| `prevAngle` | Angle | Reference alignment. Written during Origin Set; after a later Tag Finding alignment, the previous `newAngle` becomes `prevAngle` |

---

## Origin Set

Initial registration. The operator places the device at the origin Pose `(east=0, north=0, up=0, roll=0, pitch=0, yaw=0)` and aligns **every** on-site tag. The UI pose screen, while `originIsSet = false`, prompts this process. Tapping **Set Origin** sends `BEGIN_ORIGIN_SET`.

`originSetStatus` reports this routine. Do not use `tagFindingStatus` for Origin Set.

### On entry

The projector is enabled, projection mode is set to POINT. The system listens for Nudge events. Laser duty is 30%. The laser stays ON throughout the routine. `originSetStatus = WAITING_FOR_TAG`.

`lastKnownPose` is the origin Pose. `originIsSet` remains `false` until Origin Set finishes.

### Upon Nudge events

The system applies `NUDGE_X` / `NUDGE_Y` from the UI to the projector.

### Upon Aligned events (`TAG_ALIGNED`)

The system translates the current DAC commands into true beam angles with `GalvoModel` and stores them on the `Tag` with that `id`:

- If the `id` is unknown, a new `Tag` entry is created.
- The measured `Angle` is saved as **`prevAngle`**. `newAngle` is not used in this routine.

Origin Set does **not** run `PoseSolver`. The UI walks through all tags one-by-one (navigate, Align, next tag). After the last on-site tag is aligned, Origin Set completes:

- `originIsSet = true`
- `originSetStatus = IDLE`
- IMU relocation monitoring may start
- Relocation Tag Finding, survey, and projection may be accepted

---

## Tag Finding (relocation)

Repeated whenever the device is moved. Requires `originIsSet = true`. The UI sends `BEGIN_TAG_FINDING`. This is not Origin Set: the operator aligns tags so `PoseSolver` can recover pose from stored `prevAngle` values versus new measurements.

`tagFindingStatus` reports this routine.

### On entry

The projector is enabled, projection mode is set to POINT. The system listens for Nudge events. Laser duty is 30%. The laser stays ON throughout the routine. `tagFindingStatus = WAITING_FOR_TAG`.

### Upon Nudge events

The system applies `NUDGE_X` / `NUDGE_Y` from the UI to the projector.

### Upon Aligned events (`TAG_ALIGNED`)

The system translates the current DAC commands into true beam angles with `GalvoModel`.

Those beam angles are saved on the `Tag` with the corresponding `id` as **`newAngle`**.

- If that tag already has a `newAngle` from a previous relocation alignment, that value becomes `prevAngle` and the old `prevAngle` is discarded.
- If this is the first alignment since Origin Set, `prevAngle` stays as the Origin Set reference; only `newAngle` is written.

If the `id` is unknown, a new `Tag` entry is saved (should not happen after Origin Set).

Once the system has received **2** tag alignments, it attempts to compute device pose by feeding `lastKnownPose`, `prevAngle` of the aligned IDs, and `newAngle` of the aligned IDs to `PoseSolver`.

If `PoseSolver` outputs a Pose, the CCU sets `tagFindingStatus = POSE_KNOWN` and updates `lastKnownPose`. It always asks for 2 alignments and may ask for a third if the solver cannot narrow the pose. While solving, `tagFindingStatus = COMPUTING` (UI loading screen).

The UI flow for each tag:

1. Navigate the beam onto the tag
2. Press **Align**
3. See **Computing...**
4. Either another tag request, or the **Pose Known** screen

---

## Survey

The UI sending a PERFORM_SURVEY command (containing an array of Nodes of interest) triggers entry to the routine. Upon entry, the CCU surveys each point in turn. For each point, the CCU:
- Uses the North-East coordinate to calculate the azimuth line on which the point lines (if the point lies outwith the FOV, it is marked as such and the system moves on)
- Commands the projector to the bottom of the azimuth line, then to the top of it. During traversal each [elevation, distance] pair collected are used to find the associated Up (z) value. If that value is found, the CCU returns to the spot and dwells there, performing a lissajous pattern. The resulting raw Dwell Point Cloud is then processed. If no match within a specified tolerance is found, the SURVEY_RESULT packet returns FAILED with an error message.
- The LiDAR motor angles are mapped to a virtual point source originating from the middle of the lower galvo mirror (the device's local origin) by LidarModel.
- Fits a plane to the cloud, centred on the [azimuth, elevation, distance] point which was closest to the required East-North (XY) coordinate. From this plane, the system saves the normal (for use during projection) and computes the intersection of the plane and the vertical (cartesian) line representing the East-North point of interest.
- Sends a SURVEY packet containing the Node (with the Up field populated and surveyed=true), the survey result (SUCCESS/FAILED) and an errorCode. If the current Node is the final one in the request list, surveyFinished=true.

## Projection

The UI command PERFROM_PROJECTION trigegrs entry to the Projection routine. The command packet contains a list of Nodes of interest.
- The CCU examines each Node of interest to make sure they are all pre-sureyed (surveyed=True) and in the current FOV, takes those which are and uses PathEngine to turn the Node corner information (orientation and acuteness), along with the local slope from the Survey, into a smooth projection pathway. All points not part of a corner piece will be blanked (laser off).
- The CCU will then send a PROJECTION_READY packet containing the Nodes in the projection path and any Error messsages (eg "Vertex out of FOV").
- The UI will receive this message, the "Project" button will become "End" and above it will appear a stateful "Pause"/"Play". The UI updates its screen based on projectionStatus, the PROJECTION_READY command and projectorRunning.
- The CCU projects a single point by selecting POINT mode and moving to that location. It projects multiple nodes by selecting PATH mode and streaming a list of points to the projector. PLAY and PAUSE start the stream and stop and reset it, respectively, to which the projectorRunning telemtry field reacts. END stops the stream, discards the computed path and sets projectionStatus=Idle.

## Base Rotation Tracking, Effective Yaw and Relocation Detection

The CCU is wired to a 9DOF IMU transmitting global roll, pitch and yaw in quaternion form. The base of the device rotates, and the angle of rotation is fed back by a magnetic sensor also wired to the CCU. The OrientationEngine takes these values, along with the lastKnownPose and outputs the effectiveYaw (the yaw of the device with respect to the gravity vector) and governs the latching 'moved' telemetry field.

# Glossary
## GalvoModel

Internal CCU model. Converts DAC commands to true beam `Angle` (azimuth, elevation) for a point source at the middle of the lower galvo mirror (the device's assumed local origin). Used by both Origin Set and Tag Finding on `TAG_ALIGNED`. This process allows the device to map raw galvanometer DAC commands to real world coordinates.

## LidarModel

Internal CCU model. Maps LiDAR motor angles to the angle from a virtual point source originating from the middle of the lower galvanometer mirror. This is the location of the device's assumed origin. This process allows the device to map raw data from the LiDAR module to real world coordinates.

## PoseSolver

Internal CCU solver. Used **only** by relocation Tag Finding. Inputs: `lastKnownPose`, `prevAngle` and `newAngle` for the aligned tag IDs. Output: a Pose, or no solution (ask for another tag).

## PathEngine

Internal CCU solver. Finds a smooth projection path in angle space which results in accurate corner pieces and smooth, blanked connecting segments. Requires a list of cartesian coordinates of to-be-visited Nodes. It also requires the local slope at each vertex (found during the Survey) and the orientation and acuteness of each corner. The Node objects passed to the solver will contain all of these fields.

## OrientationEngine

