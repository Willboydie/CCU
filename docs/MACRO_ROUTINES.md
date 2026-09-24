# Macro routines

Behaviour of the CCU work routines. Command bytes live in `CMD_PROTOCOL.md`. Session and connection flow live in `POC_WORKFLOW.md`.

There are **two Tag Finding variants**. The UI and CCU treat them as separate routines. Both use the projector to aim and the LiDAR to range:

| Routine | Command | When | PoseSolver |
| --- | --- | --- | --- |
| **Origin Set** | `BEGIN_ORIGIN_SET` | Initial registration and alignment of every tag on site (`originIsSet = false`) | No |
| **Tag Finding** | `BEGIN_TAG_FINDING` | Repeated after the device is moved (`originIsSet = true`) | Yes |

While `originIsSet = false`, the CCU performs no work function except Origin Set. That includes IMU relocation monitoring (`moved` stays `false`). Survey, projection, and relocation Tag Finding are ignored.

The two routines in the table above, and the Origin Set / Tag Finding sections below, are the later product. This build runs the promo section instead.

---

## Promo build

The promo device is level and stands at the plan origin. It does not align tags, and it does not run relocation Tag Finding. `BEGIN_ORIGIN_SET` stores the current AS5048 stage angle as `northOffset` and sets `originIsSet`. The projector stays off for that command. No `Tag` record is written. Pose and yaw rules are in the promo section of `POC_WORKFLOW.md`.

Survey and projection call the naive models in this section. The azimuth-sweep survey, the lissajous dwell, the plane fit, and the smooth `PathEngine` later in this document are the later product. The constants here are placeholders. Replace the constant or the function body later. Callers stay the same.

### GalvoModel (promo)

Device centre and beam origin are the same point. Mirror offset is zero.

| Constant | Value |
| --- | --- |
| DAC boresight | `32768` |
| `kGalvoRadPerDac` | `1.0e-4` rad per count (about 0.1 mrad) |

```
azimuth   = (dac_x - 32768) * kGalvoRadPerDac
elevation = (dac_y - 32768) * kGalvoRadPerDac
```

The inverse clamps each DAC into `0 … 65535`. A device-centre XYZ point uses `atan2` for azimuth and elevation.

### LidarModel (promo)

| Constant | Value |
| --- | --- |
| Motor neutral | `kMechNeutralX`, `kMechNeutralY` in `lib/scan/Geometry.hpp` |
| Rangefinder origin | `(0, 0, 0)` relative to the device centre |
| `kFloorUpMm` | `-150` (stand-in floor height under a level device when a range is missing) |

Motor angle to pointing uses that neutral as the boresight stand-in. Reported range is the distance. Up of a return is `range_mm * sin(elevation)`, elevation positive above the horizontal.

### Survey (promo)

The aim of a survey is to find a best estimate for the Up value associated with each node's East-North coordinate on the real terrain. Local slope estimates are also collected for each node. This data is saved for each Node object and is required for an accurate projection. 

Upon entering the routine, the effectiveYaw in that moment is taken as the assumed yaw for the whole routine. The device cannot therefore be moved mid-survey.

One node is surveyed at a time. Each node is an East-North coordinate. The device is at the origin, so the clockwise bearing of a node is:

```
bearing = wrap(atan2(East, North))
device_azimuth = wrap_signed(bearing - effectiveYaw)
```

`effectiveYaw` is clockwise from due north. A node farther than D_MAX m (defined in config), or with `device_azimuth` outside ±ANGLE_MAX°, returns `SURVEY_RESULT` failed with `OUT_OF_FOV`. The lidar is not moved for that node.

Nodes unsurveyed are not known to the CCU. When a Node survey is successful, a new Node object is created internally.

The motor commands associated with the top and bottom of the elevation are given by inputing the device angles (device_azimuth, -`ANGLE_MAX`), (device_azimuth, +`ANGLE_MAX`) into the associated mapping function owned by `LidarModel`.

The CCU then enables the LiDAR (which begins 1KHz LiDAR telemetry - see @LiDAR_HOST_PROTOCOL) and the ScanEngine and commands the P2P target first to the bottom of the azimuth and then to the top. During azimuth traversal (elevation, distance) pairs are read from telemetry and the Up value they imply is computed in real time. The closest reading (also within a cartesian tolerance `SURVEY_TOL`) is recognised and the LiDAR is commanded to return to it. If there isn't a closest point within tolerance, then `SURVEY_RESULT` failed with `NOT_VISIBLE` and we move on to the next Node.

When the LiDAR returns to the closest point, a coarse scan is commanded. The dwell lasts for DWELL_TIME seconds and all raw points are saved. When the scan is commanded to stop, the raw points are inputed into LidarModel's mapping to recover the device angles and then the assumed stage yaw is used to convert these points back to East-North-Up coordinates. 

The resulting point cloud is processed by a naive least squares plane fit centred on the coordinate of the closest point from the azimuth traversal. The intersection point between this plane and the vertical line (East, North) to give the resulting Up value.

All surveyed nodes (successful or not) are reported after each individual node survey in a SURVEY_RESULT packet. The last node of the job sets `surveyFinished`.

### Projection (promo)

A node with no stored survey result is omitted with `NOT_SURVEYED`. A node now outside the wedge is omitted with `OUT_OF_FOV`. `PathEngine` runs on the rest. The CCU sends `PROJECTION_READY`, then waits for a PLAY command to begin streaming the list of DAC cmd pairs produced by `PathEngine` in a loop. Pause stops the stream. Play restarts the stream. End discards the path and turns the laser off. SET_LASER_POWER commands from the tablet are passed to the projector.

### PathEngine (promo)

Input is a set of surveyed nodes. The engine uses BASE_PATH_DENSITY, CORNER_DENSITY and CORNER_LEN (from config) and the corner_start_angle and corner_acuteness from each Node object.

#### Corner Pieces
The engine first generates corner pieces for each node made up of cartesian points. First the engine finds the start vector which lies on the plane Up=Node_Up and makes an angle of corner_start_angle with due north and the exit vector lying on the same plane which makes an angle of corner_start_angle+corner_acuteness with due north. The engine then produces a list of points which trace out the path from the vertex along the start vector, where the point density begins at CORNER_DENSITY at the vertex itself and increases linearly to BASE_PATH_DENSITY CORNER_LEN from the vertex. An analogous list of points is produced along the exit vector. The two lists are combined to create a list of cartesian points which form the corner, the middle point of which is the vertex itself.

Each point is passed through GalvoModel to create a list of corresponding DAC cmd pairs.

#### Joining Segments
Between each node, the engine then generates smooth adjoining segments.

For each node corner piece, the vector between the final two DAC cmd pairs of that corner piece and the vector between first two DAC pairs of the next node corner piece are computed. The magnitude of the first vector is taken as the segment_density. The directions of these vectors are taken at the segment_start_direction and segment_finish_direction. The engine then generates a hermite cubic curve of points of segment_density which begins in segment_start_direction, ends in segment_finish_direction and connects the last DAC pair of the node corner piece with the first DAC pair of the next node corner piece.

The segments and corner pieces are joined together to form a single list, which should form a loop. This list is the `PathEngine` output.


### Promo Config
- D_MAX, ANGLE_MAX (rad)
- SURVEY_TOL (m)
- BASE_PATH_DENSITY, CORNER_DENSITY (points/m)
- CORNER_LEN (m)

---

## Shared types

CCU and UI use the same `Angle` and `Tag` objects.

Both Origin Set and relocation Tag Finding use the **projector and the LiDAR**. They are different point sources. The device centre is the middle of the **bottom** galvo mirror. The projector beam leaves from the **top** galvo mirror. On `TAG_ALIGNED`, the CCU commands the LiDAR to the same pointing `Angle` as the projector; the rangefinder origin stays offset. Tag radius is greater than that offset, so the LiDAR still hits the tag. `LidarModel` then maps the LiDAR range to distance from the **device centre**, using the rangefinder origin offset it owns.

### `Angle`

`(azimuth, elevation)` pair: galvo pointing angle from the **top** mirror (the beam-exit mirror). Not from the device centre.

| Field | Type |
| --- | --- |
| `azimuth` | float |
| `elevation` | float |

### `Tag`

Stored by the CCU under each tag `id`. Distances are from the **device centre** (bottom-mirror middle) to the tag. `LidarModel` applies the rangefinder origin offset it owns — not raw LiDAR range, not from the top galvo mirror.

| Field | Type | Notes |
| --- | --- | --- |
| `id` | string | Stable identity |
| `newAngle` | Angle | Latest projector alignment from relocation Tag Finding |
| `prevAngle` | Angle | Reference projector alignment. Written during Origin Set; after a later Tag Finding alignment, the previous `newAngle` becomes `prevAngle` |
| `newDistance` | float | Latest range from the device centre (relocation Tag Finding) |
| `prevDistance` | float | Reference range from the device centre. Written during Origin Set; shifts with `newAngle` → `prevAngle` |

---

## Origin Set

The promo build does not run this section. See **Promo build** above.

Initial registration. The operator places the device at the origin Pose `(east=0, north=0, up=0, roll=0, pitch=0, yaw=0)` and aligns **every** on-site tag. The UI pose screen, while `originIsSet = false`, prompts this process. Tapping **Set Origin** sends `BEGIN_ORIGIN_SET`.

`originSetStatus` reports this routine. Do not use `tagFindingStatus` for Origin Set.

### On entry

The projector is enabled, projection mode is set to POINT. The system listens for Nudge events. Laser duty is 30%. The laser stays ON throughout the routine. `originSetStatus = WAITING_FOR_TAG`.

`lastKnownPose` is the origin Pose. `originIsSet` remains `false` until Origin Set finishes.

### Upon Nudge events

The system applies `NUDGE_X` / `NUDGE_Y` from the UI to the projector.

### Upon Aligned events (`TAG_ALIGNED`)

The projector is on the tag. The CCU then ranges with the LiDAR at the same pointing `Angle` (offset origin) and stores both on the `Tag` with that `id`:

1. `GalvoModel` maps the current DAC pair to galvo pointing `Angle` (from the top mirror) and to beam origin relative to the device centre.
2. The LiDAR is commanded to that same pointing `Angle`. `LidarModel` maps motor-angle pair ↔ rangefinder pointing angle, and motor angle → rangefinder origin relative to the device centre. The two origins stay offset.
3. Tag radius is greater than that offset, so the LiDAR still hits the tag. `LidarModel` maps the LiDAR range to distance from the **device centre** using the rangefinder origin offset it owns.
4. If the `id` is unknown, a new `Tag` entry is created.
5. The galvo `Angle` is saved as **`prevAngle`** and the device-centre distance as **`prevDistance`**. `newAngle` / `newDistance` are not used in this routine.

Origin Set does **not** run `PoseSolver`. The UI walks through all tags one-by-one (navigate, Align, next tag). After the last on-site tag is aligned, Origin Set completes:

- `originIsSet = true`
- `originSetStatus = IDLE`
- IMU relocation monitoring may start
- Relocation Tag Finding, survey, and projection may be accepted

---

## Tag Finding (relocation)

Repeated whenever the device is moved. Requires `originIsSet = true`. The UI sends `BEGIN_TAG_FINDING`. This is not Origin Set: the operator aligns tags with the projector, the LiDAR ranges at the same pointing `Angle` from its offset origin, and `PoseSolver` recovers pose from stored `prevAngle` / `prevDistance` versus new measurements.

`tagFindingStatus` reports this routine.

### On entry

The projector is enabled, projection mode is set to POINT. The system listens for Nudge events. Laser duty is 30%. The laser stays ON throughout the routine. `tagFindingStatus = WAITING_FOR_TAG`.

### Upon Nudge events

The system applies `NUDGE_X` / `NUDGE_Y` from the UI to the projector.

### Upon Aligned events (`TAG_ALIGNED`)

The projector is on the tag. The CCU then ranges with the LiDAR at the same pointing `Angle` (offset origin):

1. `GalvoModel` maps the current DAC pair to galvo pointing `Angle` (from the top mirror) and to beam origin relative to the device centre.
2. The LiDAR is commanded to that same pointing `Angle`. `LidarModel` supplies motor-angle pair ↔ rangefinder pointing angle and motor angle → rangefinder origin relative to the device centre.
3. Tag radius is greater than the origin offset, so the LiDAR still hits the tag. `LidarModel` maps the LiDAR range to distance from the **device centre** using the rangefinder origin offset it owns.

That `Angle` and device-centre distance are saved on the `Tag` with the corresponding `id` as **`newAngle`** and **`newDistance`**.

- If that tag already has a `newAngle` from a previous relocation alignment, that `newAngle` / `newDistance` become `prevAngle` / `prevDistance` and the old previous values are discarded.
- If this is the first alignment since Origin Set, `prevAngle` / `prevDistance` stay as the Origin Set reference; only `newAngle` / `newDistance` are written.

If the `id` is unknown, a new `Tag` entry is saved (should not happen after Origin Set).

Once the system has received **2** tag alignments, it attempts to compute device pose by feeding `lastKnownPose`, `prevAngle` / `prevDistance` of the aligned IDs, and `newAngle` / `newDistance` of the aligned IDs to `PoseSolver`.

If `PoseSolver` outputs a Pose, the CCU sets `tagFindingStatus = POSE_KNOWN` and updates `lastKnownPose`. It always asks for 2 alignments and may ask for a third if the solver cannot narrow the pose. While solving, `tagFindingStatus = COMPUTING` (UI loading screen).

The UI flow for each tag:

1. Navigate the beam onto the tag
2. Press **Align**
3. See **Computing...**
4. Either another tag request, or the **Pose Known** screen

---

## Survey

The promo build uses the naive survey in **Promo build** above. This section is the later product.

The UI sending a PERFORM_SURVEY command (containing an array of Nodes of interest) triggers entry to the routine. Upon entry, the CCU surveys each point in turn. For each point, the CCU:
- Uses the North-East coordinate to calculate the azimuth line on which the point lines (if the point lies outwith the FOV, it is marked as such and the system moves on).
- Requests the azimuth and elevation angles from 'LidarModel', which uses its internal mapping to produce the associated motor angle commands.
- Commands the projector to the bottom of the azimuth line, then to the top of it. During traversal each [elevation, distance] pair collected are used to find the associated Up (z) value. If that value is found, the CCU returns to the spot and dwells there, performing a lissajous pattern. The resulting raw Dwell Point Cloud is then processed. If no match within a specified tolerance is found, the SURVEY_RESULT packet returns FAILED with an error message.
- `LidarModel` maps each sample: motor-angle pair → rangefinder pointing angle, and motor angle → rangefinder origin relative to the device centre (bottom-mirror middle). `GalvoModel` / `LidarModel` place that return in device-centre coordinates.
- Fits a plane to the cloud, centred on the [azimuth, elevation, distance] point which was closest to the required East-North (XY) coordinate. From this plane, the system saves the normal (for use during projection) and computes the intersection of the plane and the vertical (cartesian) line representing the East-North point of interest.
- Sends a SURVEY packet containing the Node (with the Up field populated and surveyed=true), the survey result (SUCCESS/FAILED) and an errorCode. If the current Node is the final one in the request list, surveyFinished=true.

## Projection

The promo build uses the naive projection in **Promo build** above. This section is the later product.

The UI command PERFROM_PROJECTION trigegrs entry to the Projection routine. The command packet contains a list of Nodes of interest.
- The CCU examines each Node of interest to make sure they are all pre-sureyed (surveyed=True) and in the current FOV, takes those which are and uses PathEngine to turn the Node corner information (orientation and acuteness), along with the local slope from the Survey, into a smooth projection pathway. All points not part of a corner piece will be blanked (laser off).
- The CCU will then send a PROJECTION_READY packet containing the Nodes in the projection path and any Error messsages (eg "Vertex out of FOV").
- The UI will receive this message, the "Project" button will become "End" and above it will appear a stateful "Pause"/"Play". The UI updates its screen based on projectionStatus, the PROJECTION_READY command and projectorRunning.
- The CCU projects a single point by selecting POINT mode and moving to that location. It projects multiple nodes by selecting PATH mode and streaming a list of points to the projector. PLAY and PAUSE start the stream and stop and reset it, respectively, to which the projectorRunning telemtry field reacts. END stops the stream, discards the computed path and sets projectionStatus=Idle.

## OrientationEngine

The CCU is wired to a 9DOF IMU transmitting global roll, pitch and yaw in quaternion form. The base of the device rotates, and the angle of rotation is fed back by a magnetic sensor also wired to the CCU. The OrientationEngine takes these values and outputs a realtime Tilt value (roll and pitch). Tilt is fused with stage angle to give Yaw. Heavy LPFs are used to remove jitter. OrientationEngine runs on a peripheral ESP32-C3 SuperMini and values are fed to the CCU via serial.

# Glossary
## GalvoModel

Internal CCU model of the projector. Device centre is the middle of the **bottom** galvo mirror. The beam leaves from the **top** mirror — a different point source from the LiDAR.

Mappings:

| From | To |
| --- | --- |
| DAC command pair | Galvo pointing `Angle` (from the top mirror) |
| DAC command pair | Beam origin with respect to the device centre (bottom-mirror middle) |
| XYZ with respect to the device centre (bottom-mirror middle) | DAC command pair |

Used with `LidarModel` on `TAG_ALIGNED` (match pointing `Angle`) and by Survey / Projection.

## LidarModel

Internal CCU model of the rangefinder. The LiDAR is a different point source from the projector. It can be commanded to the **same pointing `Angle`** as the galvo; the origins stay offset. Tag radius is greater than that offset, so a tag-aligned beam still produces a valid range. `LidarModel` maps that range to distance from the **device centre** using the rangefinder origin offset it owns.

Mappings:

| From | To |
| --- | --- |
| Motor-angle pair | LiDAR (rangefinder) pointing angle |
| Motor angle | Rangefinder origin position with respect to the device centre (bottom-mirror middle) |

Used with `GalvoModel` on `TAG_ALIGNED` and by Survey.

## PoseSolver

Internal CCU solver. Used **only** by relocation Tag Finding. Inputs: `lastKnownPose`, `prevAngle` / `prevDistance` and `newAngle` / `newDistance` for the aligned tag IDs. Output: a Pose, or no solution (ask for another tag).

## PathEngine

Internal CCU solver. Finds a smooth projection path in angle space which results in accurate corner pieces and smooth, blanked connecting segments. Requires a list of cartesian coordinates of to-be-visited Nodes. It also requires the local slope at each vertex (found during the Survey) and the orientation and acuteness of each corner. The Node objects passed to the solver will contain all of these fields.