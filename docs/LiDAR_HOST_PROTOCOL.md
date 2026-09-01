# Host Serial Protocol

Binary command and telemetry interface for the dual-axis LiDAR gimbal controller.

Commands are not acknowledged. Observe `StatusPacket` (and, if enabled, debug frames) for effect. There is no checksum and no start-of-frame magic on the command path: the first byte of every host frame is the command ID.

---

## 1. Physical layer


| Parameter  | Value                              |
| ---------- | ---------------------------------- |
| Port       | USB CDC (`Serial`)                 |
| Baud       | 460800                             |
| Format     | 8 data bits, no parity, 1 stop bit |
| Byte order | Little-endian                      |
| Floats     | IEEE-754 binary32                  |


Open the port and wait until the device enumerates before transmitting.

---



## 2. Serial RX protocol (host → device)

Every command is one length-prefixed frame:

```
[ cmd : uint8 ] [ len : uint16 LE ] [ payload : len bytes ]
```


| Field     | Size  | Notes                                   |
| --------- | ----- | --------------------------------------- |
| `cmd`     | 1     | Command ID. See §3.                     |
| `len`     | 2     | Payload length in bytes, little-endian. |
| `payload` | `len` | Packed arguments. Empty when `len = 0`. |


**Rules**

- Maximum payload is **16 bytes**. Larger `len` values are drained and discarded (`FAULT_PROTO_OVERFLOW`).
- Zero-length frames are valid (toggles and mode commands).
- Multi-byte values occupy the payload in order, little-endian. A `float` is 4 bytes; two floats are 8 bytes (`float x` then `float y`).
- Extra payload bytes beyond those required by the command are ignored. Too few bytes for a float command is a framing error (`FAULT_PROTO_OVERFLOW`); the command is not applied.
- Unknown `cmd` values are ignored and raise `FAULT_CMD_INVALID`.
- There is no inter-frame delimiter. Send complete frames only. A truncated or extra byte is parsed as the start of the next command.

**Examples**

```
01 00 00              DISABLE_ENABLE (toggle)
19 04 00 8F C2 61 40  TARGET_X_SET = 3.54 rad
2D 08 00 <x:f32> <y:f32>   SET_POINT2POINT_TARGET
```

---



## 3. Command header reference

Payload `—` means `len = 0`. Units are SI unless noted. Angles are mechanical, radians.

### 3.1 System


| ID  | Hex    | Command               | Payload | Action                                                            |
| --- | ------ | --------------------- | ------- | ----------------------------------------------------------------- |
| 1   | `0x01` | `DISABLE_ENABLE`      | —       | Toggle motor enable.                                              |
| 2   | `0x02` | `SET_DEBUG_MODE`      | —       | Toggle debug text frames (§7). Default: on.                       |
| 3   | `0x03` | `TOGGLE_CONTROL_MODE` | —       | Toggle angle ↔ velocity control. Default: angle.                  |
| 27  | `0x1B` | `SLEEP`               | —       | Disable actuators and enter MCU sleep.                            |
| 28  | `0x1C` | `WAKE`                | —       | Exit sleep. Does not re-enable; send `DISABLE_ENABLE`.            |
| 29  | `0x1D` | `RESTART`             | —       | Sleep then wake. Does not re-enable.                              |
| 30  | `0x1E` | `ENTER_BOOTLOADER`    | —       | Disable, then jump to the MCU bootloader.                         |
| 31  | `0x1F` | `GET_ERROR_MSG`       | —       | Write each active fault as a debug packet, then clear all faults. |




### 3.2 Motion setpoints and gains

`TARGET_*` writes angle (rad) in angle mode, or velocity (rad/s) in velocity mode. PID writes apply to one axis. Filter and feed-forward writes apply to both axes.


| ID  | Hex    | Command                   | Payload | Action                                        |
| --- | ------ | ------------------------- | ------- | --------------------------------------------- |
| 8   | `0x08` | `VELOCITY_P_X_SET`        | `float` | Velocity-loop P, axis X.                      |
| 9   | `0x09` | `VELOCITY_I_X_SET`        | `float` | Velocity-loop I, axis X.                      |
| 10  | `0x0A` | `VELOCITY_D_X_SET`        | `float` | Velocity-loop D, axis X.                      |
| 11  | `0x0B` | `VELOCITY_P_Y_SET`        | `float` | Velocity-loop P, axis Y.                      |
| 12  | `0x0C` | `VELOCITY_I_Y_SET`        | `float` | Velocity-loop I, axis Y.                      |
| 13  | `0x0D` | `VELOCITY_D_Y_SET`        | `float` | Velocity-loop D, axis Y.                      |
| 14  | `0x0E` | `ANGLE_P_X_SET`           | `float` | Angle-loop P, axis X.                         |
| 15  | `0x0F` | `ANGLE_I_X_SET`           | `float` | Angle-loop I, axis X.                         |
| 16  | `0x10` | `ANGLE_D_X_SET`           | `float` | Angle-loop D, axis X.                         |
| 17  | `0x11` | `ANGLE_P_Y_SET`           | `float` | Angle-loop P, axis Y.                         |
| 18  | `0x12` | `ANGLE_I_Y_SET`           | `float` | Angle-loop I, axis Y.                         |
| 19  | `0x13` | `ANGLE_D_Y_SET`           | `float` | Angle-loop D, axis Y.                         |
| 20  | `0x14` | `VELOCITY_LPF_TAU_SET`    | `float` | Velocity LPF time constant (s), both axes.    |
| 21  | `0x15` | `ANGLE_LPF_TAU_SET`       | `float` | Angle LPF time constant (s), both axes.       |
| 22  | `0x16` | `FOC_VOLTAGE_LPF_TAU_SET` | `float` | FOC voltage LPF time constant (s), both axes. |
| 23  | `0x17` | `VELOCITY_FF_SET`         | `float` | Velocity feed-forward gain, both axes.        |
| 24  | `0x18` | `ACCELERATION_FF_SET`     | `float` | Acceleration feed-forward gain, both axes.    |
| 25  | `0x19` | `TARGET_X_SET`            | `float` | Axis X setpoint (rad or rad/s).               |
| 26  | `0x1A` | `TARGET_Y_SET`            | `float` | Axis Y setpoint (rad or rad/s).               |


Direct `TARGET_*` writes are overridden while Scan Engine updates are enabled.

### 3.3 Rangefinder


| ID  | Hex    | Command           | Payload | Action                                                                                                                   |
| --- | ------ | ----------------- | ------- | ------------------------------------------------------------------------------------------------------------------------ |
| 32  | `0x20` | `RECONFIGURE_LDM` | `float` | Garmin Lite V3 HP profile. Integer 0–6 encoded as float. Rejected while motors are enabled (`FAULT_CMD_REJECTED_STATE`). |



| Config | Profile                                       |
| ------ | --------------------------------------------- |
| 0      | Default, balanced                             |
| 1      | Short range, high speed                       |
| 2      | Default range, faster short-range termination |
| 3      | Maximum range                                 |
| 4      | High sensitivity                              |
| 5      | Low sensitivity                               |
| 6      | Short range, high speed, higher error         |




### 3.4 Scan Engine

`ENABLE` / `DISABLE` gate target writes. `ON` / `OFF` select the path. Typical sequence: **enable motors → Scan Engine on → begin coarse or set a P2P target**. Motor enable also sets the Scan Engine update gate; motor disable clears it.


| ID  | Hex    | Command                     | Payload            | Action                                                                                                                         |
| --- | ------ | --------------------------- | ------------------ | ------------------------------------------------------------------------------------------------------------------------------ |
| 35  | `0x23` | `BEGIN_COARSE_SCANNING`     | —                  | Wind up a Lissajous scan about the current pose. Requires `ON`. Rejected if the current pose ± amplitude exceeds angle limits. |
| 36  | `0x24` | `X_AMPLITUDE_SET`           | `float`            | Coarse X amplitude (rad). Range 0…0.6.                                                                                         |
| 37  | `0x25` | `Y_AMPLITUDE_SET`           | `float`            | Coarse Y amplitude (rad). Range 0…0.6.                                                                                         |
| 38  | `0x26` | `X_FREQUENCY_SET`           | `float`            | Coarse X frequency (Hz).                                                                                                       |
| 39  | `0x27` | `Y_FREQUENCY_SET`           | `float`            | Coarse Y frequency (Hz).                                                                                                       |
| 40  | `0x28` | `X_PHASE_SET`               | `float`            | Coarse X phase (rad).                                                                                                          |
| 41  | `0x29` | `Y_PHASE_SET`               | `float`            | Coarse Y phase (rad).                                                                                                          |
| 42  | `0x2A` | `STOP_COARSE_SCANNING`      | —                  | Taper the coarse scan, then seek neutral in point-to-point.                                                                    |
| 43  | `0x2B` | `SCANENGINE_ON`             | —                  | Enter point-to-point and seek the neutral pose.                                                                                |
| 44  | `0x2C` | `SCANENGINE_OFF`            | —                  | Leave Scan Engine mode immediately (does not home).                                                                            |
| 45  | `0x2D` | `SET_POINT2POINT_TARGET`    | `float x, float y` | New P2P target (rad). Valid only in point-to-point mode; must lie inside the angle limits.                                     |
| 46  | `0x2E` | `UPDATE_COARSE_SCAN_CENTRE` | `float x, float y` | Move the coarse-scan centre. Valid only in coarse mode; centre ± amplitude must stay inside the angle limits.                  |
| 47  | `0x2F` | `SEEK_NEUTRAL`              | —                  | Command a P2P move to the neutral pose. Valid only in point-to-point mode.                                                     |
| 48  | `0x30` | `SET_VELOCITY_MAX`          | `float`            | Set the maximum velocity reached by the module during a P2P movement                                                           |

---



## 4. Device → host packets

Telemetry uses the same three-byte prefix as commands. `len` is the payload size (excludes the prefix). Packets are packed structs, little-endian, no padding.

```
[ header : uint8 ] [ len : uint16 LE ] [ payload : len bytes ]
```


| Header  | Hex    | Rate  | Payload | Total     |
| ------- | ------ | ----- | ------- | --------- |
| Control | `0x55` | 1 kHz | 26 B    | 29 B      |
| Status  | `0xAA` | 10 Hz | 21 B    | 24 B      |
| Debug   | `0xFF` | event | `len` B | 3 + `len` |


A host RX state machine should switch on `header`, then read `len` bytes of payload. Headers `0x55`, `0xAA`, and `0xFF` are distinct from all command IDs.

### 4.1 ControlPacket (`0x55`)

Closed-loop telemetry.


| Offset | Type     | Field                | Unit                    |
| ------ | -------- | -------------------- | ----------------------- |
| 0      | `uint8`  | `header`             | `0x55`                  |
| 1      | `uint16` | `len`                | 26                      |
| 3      | `float`  | `target_angle_x`     | rad                     |
| 7      | `float`  | `target_angle_y`     | rad                     |
| 11     | `uint16` | `distance`           | cm                      |
| 13     | `float`  | `mechanical_angle_x` | rad, live rotor         |
| 17     | `float`  | `mechanical_angle_y` | rad, live rotor         |
| 21     | `float`  | `associated_angle_x` | rad, at last LDM sample |
| 25     | `float`  | `associated_angle_y` | rad, at last LDM sample |




### 4.2 StatusPacket (`0xAA`)

Housekeeping.


| Offset | Type     | Field             | Notes                          |
| ------ | -------- | ----------------- | ------------------------------ |
| 0      | `uint8`  | `header`          | `0xAA`                         |
| 1      | `uint16` | `len`             | 21                             |
| 3      | `uint8`  | `enabled`         | 1 = motors enabled             |
| 4      | `uint8`  | `scan_mode`       | §5                             |
| 5      | `uint8`  | `dynamic_routine` | §5                             |
| 6      | `uint8`  | `debug_mode`      | 1 = debug frames enabled       |
| 7      | `uint8`  | `has_fault`       | 1 = one or more latched faults |
| 8      | `float`  | `coarse_centre_x` | rad                            |
| 12     | `float`  | `coarse_centre_y` | rad                            |
| 16     | `float`  | `p2p_target_x`    | rad                            |
| 20     | `float`  | `p2p_target_y`    | rad                            |


---



## 5. Status enumerations

`scan_mode`


| Value | Mode           |
| ----- | -------------- |
| 0     | Off            |
| 1     | Coarse         |
| 2     | Point-to-point |


`dynamic_routine`


| Value | Routine                   |
| ----- | ------------------------- |
| 0     | None                      |
| 1     | Winding up to coarse      |
| 2     | Tapering down from coarse |


Non-zero `dynamic_routine` means a coarse wind-up or taper is in progress. Wait for `0` before issuing `BEGIN_COARSE_SCANNING` or `STOP_COARSE_SCANNING`.

---



## 6. Operating limits


| Quantity         | Range                      |
| ---------------- | -------------------------- |
| Axis X angle     | 2.5 … 4.2 rad              |
| Axis Y angle     | 2.6 … 3.9 rad              |
| Neutral pose     | X = 3.54 rad, Y = 3.39 rad |
| Coarse amplitude | 0 … 0.6 rad                |
| Velocity clamp   | ±15 rad/s                  |


Out-of-range P2P targets, coarse centres, and amplitudes are rejected (not applied).

---



## 7. Debug text frames

Emitted only when debug mode is on (`SET_DEBUG_MODE`, `StatusPacket.debug_mode`).

```
[ header : uint8 = 0xFF ] [ len : uint16 LE ] [ payload : len bytes ]
```

Maximum payload 256 bytes. Do not parse the payload as a command.

**Decoding.** Each payload byte is one ASCII character (`uint8` code point 0–127). There is no NUL terminator and no length prefix inside the payload — `len` is the character count. Map byte `b` to character `chr(b)` and concatenate in order.

```
payload bytes:  41 74 20 6E 65 75 74 72 61 6C 2E
decoded text:   At neutral.
```

C: `memcpy(text, payload, len); text[len] = '\0';`  
Python: `text = payload.decode("ascii")`