# Host Protocol — Laser Projection Firmware

This document describes how a host PC must interact with the Teensy 4.1 laser projection firmware over USB serial (CDC). It is the authoritative reference for wire format, commands, streaming behavior, and correct session sequencing.

**Target platform:** Teensy 4.1 (`teensy41` in PlatformIO)

**Reference implementation:** [tools/galvoController.py](tools/galvoController.py) (may lag the firmware; prefer this document and `lib/` sources)

---

## Connection

| Parameter | Value |
|-----------|-------|
| Interface | USB CDC virtual serial port |
| Nominal baud | 115200 (ignored by USB CDC; full USB throughput is available) |
| Byte order | Little-endian for all multi-byte **command** fields unless noted |
| Framing | Host→device commands and status use a 3-byte header + payload; flow-control / reset notifications are raw single bytes (see below) |

On connect, allow ~1–2 seconds for the device to enumerate and finish `setup()` before sending commands.

The host must continuously read from the serial port. The device sends status packets, raw flow-control bytes, debug/fault text, and other traffic on the same port.

---

## Wire framing

### Framed messages (commands and status)

Most host→device commands and the device→host status packet use:

```
[cmd: 1 byte][length_lo: 1 byte][length_hi: 1 byte][payload: length bytes]
```

- `length` is a **uint16 little-endian** payload size.
- Non-stream command payloads are limited to **1024 bytes** (`MAX_CMD_PAYLOAD`). Oversize payloads are discarded and latched as `FAULT_CMD_OVERSIZE`.
- Stream frames (`POINT_STREAM_HEADER`) also share that 1024-byte payload ceiling because they are buffered in the same payload buffer before being written to the ring.

### Host send example (Python)

```python
def send_command(ser, cmd: int, payload: bytes = b"") -> None:
    frame = bytes([cmd]) + len(payload).to_bytes(2, "little") + payload
    ser.write(frame)
```

### Unframed device→host notifications

`PAUSE_STREAM`, `RESUME_STREAM`, and outbound `RESET_STREAM` are sent as a **single raw byte** (the command value only — no length field). The host RX parser must accept these as one-byte signals interleaved with framed packets and debug lines.

### Host receive parser

The RX parser must handle **interleaved** traffic on one byte stream:

1. **Debug / fault lines** — prefix `0xFF 0x04 0x06`, then UTF-8 text until `\n`
2. **Raw flow-control bytes** — `0xF8` / `0xF9` / `0xF7` with no length
3. **Framed packets** — currently status (`0xFA`)

Maintain a receive buffer. For framed packets, wait until `3 + length` bytes are available before consuming. If the first byte does not match a known pattern and parsing fails, discard one byte and resync.

---

## Point record format

Each projection point is **5 bytes**:

| Offset | Size | Field | Encoding |
|--------|------|-------|----------|
| 0 | 2 | X | uint16 **big-endian** (MSB first) |
| 2 | 2 | Y | uint16 **big-endian** |
| 4 | 1 | laser | `0` = off, non-zero = on |

This layout is used inside stream payloads and inside `.bin` stream files.

**Exception:** `SET_POINT_CMD` uses **little-endian** X and Y (see [Point mode](#point-mode)).

Coordinate range: 0–65535 (16-bit DAC values).

---

## Device modes

On boot the device is **enabled**, **awake**, in **PATH** project mode, `projecting = false`, and `stream_state = RESET`. Debug mode defaults to **off** (`DEFAULT_DEBUG_SETTING = false`).

| Concept | Values | Meaning |
|---------|--------|---------|
| Power | awake / sleeping | Sleeping disables hardware; wake re-initializes |
| Enabled | on / off | When disabled, main-loop projection/flow work is skipped |
| Project mode | PATH / POINT | PATH = serial streaming + ISR; POINT = host-paced `SET_POINT` |
| Projecting | on / off | Host-requested projection intent (`PLAY` / `STOP`) |
| Stream state | RUNNING / PAUSED / RESET | Flow-control state machine (see [Status packet](#status-packet)) |

There is no separate "ready", "write", or closed-loop echo mode in the current firmware.

---

## Incoming commands (host → device)

All command byte values below are shown in hex. Only commands handled by `Interface::processCommand` (plus the stream header handled by `FrameParser`) are listed.

### System and power

| Command | Hex | Payload | Action |
|---------|-----|---------|--------|
| `SLEEP_CMD` | `0xE0` | 0 bytes | Enter low-power sleep; disable + deinit |
| `WAKE_CMD` | `0xE1` | 0 bytes | Wake from sleep; re-initialize and enable |
| `BOOTLOADER_CMD` | `0xE2` | 0 bytes | Disable, deinit, reboot into Teensy bootloader |
| `ENABLE_DISABLE_CMD` | `0x8F` | 0 bytes | Toggle enabled state; disabling stops ISR, clears ring, blanks laser |
| `RESTART_CMD` | `0x8E` | 0 bytes | Disable, deinit, reinit (soft restart) |

### Projection control

| Command | Hex | Payload | Action |
|---------|-----|---------|--------|
| `PLAY_CMD` | `0x07` | 0 bytes | Set `projecting = true` (does not clear the ring or reset packet index) |
| `STOP_CMD` | `0x08` | 0 bytes | Set `projecting = false` (ISR ends on the next `projector.flowControl()`) |
| `PLAY_PAUSE_CMD` | `0x09` | 0 bytes | Toggle — calls `STOP` if projecting, else `PLAY` |
| `SET_PLAYBACK_SPEED_CMD` | `0x0A` | 2 bytes, uint16 **LE**, kHz | Set point rate (see [Playback speed](#playback-speed)) |
| `SET_LASER_DUTY_CYCLE_CMD` | `0x1B` | 2 bytes, uint16 **LE**, percent 0–100 | Set laser PWM duty cycle |
| `SEND_STATUS_CMD` | `0x0B` | 0 bytes | Request a status packet (rate-limited to once per 100 ms) |
| `DEBUG_MODE_CMD` | `0x1A` | 0 bytes | Toggle debug text output on/off |

### Project mode and point control

| Command | Hex | Payload | Action |
|---------|-----|---------|--------|
| `TOGGLE_PROJECT_MODE_CMD` | `0x3A` | 0 bytes | Toggle PATH ↔ POINT |
| `SET_POINT_CMD` | `0x3B` | 5 bytes (see [Point mode](#point-mode)) | Set galvo position + laser (POINT mode only) |

Entering POINT mode while stream state is RUNNING or PAUSED causes `FlowControl` to clear the ring and emit `RESET_STREAM`.

### Streaming data

| Command | Hex | Payload | Action |
|---------|-----|---------|--------|
| `POINT_STREAM_HEADER` | `0xAA` | variable | Stream packet: packet index + point records (see [Stream packet format](#stream-packet-format)) |
| `RESET_STREAM_CMD` | `0xF7` | 0 bytes | Stop ISR, clear ring (resets packet index), set `projecting = false`, `stream_state = RESET`, echo `RESET_STREAM` to host |

Stream frames are accepted whenever they parse successfully; there is no separate "session accepting" gate. Use `RESET_STREAM` to clear the ring and re-sync the packet index before a new playback.

### Diagnostics

| Command | Hex | Payload | Action |
|---------|-----|---------|--------|
| `GET_FAULT_MSG_CMD` | `0xFC` | 0 bytes | Emit a debug-framed message for each latched fault, then clear all faults |

---

## Outgoing packets (device → host)

| Command | Hex | Framing | Payload | Meaning |
|---------|-----|---------|---------|---------|
| `PAUSE_STREAM_CMD` | `0xF8` | **raw 1 byte** | — | Ring above pause watermark — **stop sending** stream frames |
| `RESUME_STREAM_CMD` | `0xF9` | **raw 1 byte** | — | Ring below resume watermark while projecting — **may send** stream frames |
| `RESET_STREAM_CMD` | `0xF7` | **raw 1 byte** | — | Stream reset (host command ack, or automatic on PATH→POINT while active) |
| `RECEIVE_STATUS_PACKET_CMD` | `0xFA` | framed | 16 bytes | Status snapshot (see [Status packet](#status-packet)) |
| Debug / fault prefix | `0xFF 0x04 0x06` | special | UTF-8 text + `\n` | Debug lines (when debug on) or fault text (always framed this way) |

---

## Status packet

Sent in response to `SEND_STATUS_CMD`, but no more than once every **100 ms** even if polled faster.

**Frame:** `[0xFA][0x10][0x00]` + 16-byte payload

| Index | Field | Type | Values |
|-------|-------|------|--------|
| 0 | `power_state` | uint8 | `0` = sleeping, `1` = awake |
| 1 | `mode` | uint8 | `0` = PATH, `1` = POINT (duplicate of byte 13) |
| 2 | `enabled` | uint8 | `0` / `1` |
| 3 | `projecting` | uint8 | `0` / `1` |
| 4 | `laser_duty_cycle` | uint8 | 0–100 (percent) |
| 5 | `projector_delay` | uint8 | Point period in µs (truncated to 8 bits) |
| 6–7 | `points_frequency` | uint16 LE | Measured point rate in kHz since last status (PATH + projecting only; else 0) |
| 8–11 | `file_size` | uint32 LE | Always `0` (legacy field) |
| 12 | `debug_mode` | uint8 | `0` / `1` |
| 13 | `project_mode` | uint8 | `0` = PATH, `1` = POINT |
| 14 | `stream_state` | uint8 | `0` = RUNNING, `1` = PAUSED, `2` = RESET |
| 15 | `has_fault` | uint8 | `0` / `1` — any latched fault in `FaultManager` |

### Stream state values

| Value | Name | Meaning |
|-------|------|---------|
| 0 | RUNNING | Flow control allows host to send (device will PAUSE if ring fills) |
| 1 | PAUSED | Host should not send; device will RESUME when ring drains |
| 2 | RESET | Default / after `RESET_STREAM`; no PAUSE/RESUME emitted from this state |

When `has_fault` is set, the host should send `GET_FAULT_MSG_CMD` to retrieve and clear fault messages.

---

## PATH streaming session

PATH mode is the primary operating mode for animation playback. The host streams points over serial into a **480 KB ring buffer**; a hardware timer ISR consumes one point per tick and drives the galvo DAC and laser.

### Session lifecycle

```
  Host                                    Device
  ────                                    ──────

  RESET_STREAM ─────────────────────────►  Stop ISR, clear ring, packet_idx=0
                                           projecting=false, stream_state=RESET
  ◄────────────────────────────────────  RESET_STREAM (raw byte ack)

  SET_PLAYBACK_SPEED
  SET_LASER_DUTY_CYCLE (optional)

  PLAY ─────────────────────────────────►  projecting=true
                                           ISR starts once ring ≥ ~25%

  POINT_STREAM (idx=1,2,3,…) ───────────►  Validate packet index, append records

  ◄────────────────────────────────────  PAUSE_STREAM  (raw; ring > 75%, if RUNNING)
  (stop sending)

  ◄────────────────────────────────────  RESUME_STREAM (raw; ring < 25%, if PAUSED)
  POINT_STREAM (resume idx) ────────────►  Continue filling

  STOP ─────────────────────────────────►  projecting=false; ISR ends
```

### Critical rules

1. **Stream frames use cmd `0xAA` (`POINT_STREAM_HEADER`), not `0x3C`.** Payload starts with a packet index byte, then raw 5-byte records.

2. **Packet index starts at 1 after `RESET_STREAM` / buffer reset.** The device expects `packet_index == (last_accepted + 1) mod 256`, with `last_accepted` starting at 0, so the first valid index is **1**. A gap latches `FAULT_MISSED_STREAM_PACKET` and drops the frame.

3. **`PLAY` / `STOP` only toggle `projecting`.** They do not clear the ring or reset the packet index. Call `RESET_STREAM` when you need a clean slate.

4. **Honor `PAUSE_STREAM` / `RESUME_STREAM` when received.** They are raw single bytes. Flow-control transitions are driven while `stream_state` is RUNNING or PAUSED.

5. **The projection ISR starts only when the ring holds ≥ 25% capacity** (~122,880 bytes) while `projecting` is true. Keep sending after `PLAY` until the buffer fills enough for playback to begin.

6. **Critical depletion:** if the ring falls below **5%** while the ISR is running, the device stops the ISR, clears `projecting`, and latches `FAULT_BUFFER_CRITICALLY_LOW`. An empty ring inside the ISR latches `FAULT_RING_BUFFER_EMPTY` and ends the ISR (host should check `has_fault` / status).

7. **Disable debug mode during streaming** to keep the RX path clean (fault messages still use the debug framing when fetched).

### Recovery after faults or corruption

Typical recovery:

1. Stop sending stream frames
2. Send `RESET_STREAM` (resets ring + packet index)
3. Wait for the outbound `RESET_STREAM` byte (optional but useful for sync)
4. Send `PLAY`
5. Resume sending with `packet_index` starting at **1**
6. If `has_fault` is set in status, send `GET_FAULT_MSG_CMD` to read and clear faults

---

## Stream packet format

Outer frame: `POINT_STREAM_HEADER` (`0xAA`) with payload:

| Offset | Size | Field | Description |
|--------|------|-------|-------------|
| 0 | 1 | `packet_index` | uint8 sequence; first after reset is **1**, then increments, wraps at 256 |
| 1… | N×5 | records | Raw point records (big-endian X/Y) |

There is **no** magic byte and **no** record-count field in the payload. The outer frame `length` is the sole length authority: it must be `1 + N×5` for N complete records.

**Maximum payload:** 1024 bytes → at most `(1024 - 1) / 5 = 204` records per frame.

### Host frame builder example (Python)

```python
POINT_STREAM_HEADER = 0xAA
RECORD_SIZE = 5

def build_stream_frame(records: bytes, packet_index: int) -> bytes:
    assert len(records) % RECORD_SIZE == 0
    return bytes([packet_index & 0xFF]) + records

def send_stream(ser, records: bytes, packet_index: int) -> None:
    payload = build_stream_frame(records, packet_index)
    frame = bytes([POINT_STREAM_HEADER]) + len(payload).to_bytes(2, "little") + payload
    ser.write(frame)
```

After each accepted frame, increment `packet_index` modulo 256 (skipping is not allowed; after reset use 1, 2, …, 255, 0, 1, …).

### Backpressure inside a frame

If the ring cannot accept the entire payload, `StreamBuffer::writeFromParser` writes what fits and returns a short count; the parser latches `FAULT_STREAM_FAILED`. Prefer keeping frames small enough that pause/resume watermarks prevent overrun.

---

## Flow control

The ring buffer is 480 KB (491,520 bytes = 98,304 records).

| Watermark | Condition in code | Bytes (approx.) | Approx. duration @ 20 kHz |
|-----------|-------------------|-----------------|---------------------------|
| Resume | `available < 25%` | 122,880 | ~1.2 s of points |
| Pause | `available > 75%` | 368,640 | ~3.7 s of points |
| Critical | `available < 5%` | 24,576 | ISR stop + fault while projecting |

| Transition | Condition | Device action |
|------------|-----------|---------------|
| RUNNING → PAUSED | projecting and ring above pause | Send raw `PAUSE_STREAM` |
| PAUSED → RUNNING | projecting and ring below resume | Send raw `RESUME_STREAM` |
| RUNNING → PAUSED | `projecting` cleared | Send raw `PAUSE_STREAM` |
| RUNNING/PAUSED → RESET | project mode is POINT | Send raw `RESET_STREAM`, clear ring |

Hysteresis uses strict `<` / `>` comparisons. At 20 kHz the data rate is **100 KB/s** (20,000 points × 5 bytes).

---

## Playback speed

`SET_PLAYBACK_SPEED_CMD` payload: **uint16 LE frequency in kHz**.

| Host value (kHz) | Period (µs) | Notes |
|------------------|-------------|-------|
| 1 | 1000 | |
| 10 | 100 | |
| 20 | 50 | Default (`DEFAULT_DELAY_US = 50`) |
| ≥ 200 | 10 | Requested period 0 is clamped to `MIN_POINT_PERIOD_US` |
| 0 | 20 | Invalid; latches `FAULT_CMD_INVALID_PLAYBACK_SPEED` |

Minimum period: **10 µs** (100 kHz ceiling, `MIN_POINT_PERIOD_US`).

Formula for 1 ≤ kHz < 200: `period_us = 1000 / kHz`.

---

## Point mode

POINT mode is for calibration and host-paced positioning. There is **no projection ISR** in POINT mode; each position is applied when the command is received. While `projecting` is true in POINT mode, the laser enable path is held on (blanking still follows each `SET_POINT` laser byte).

### Entry

Send `TOGGLE_PROJECT_MODE_CMD` from PATH mode (or ensure status shows POINT).

### SET_POINT_CMD payload (5 bytes)

| Offset | Size | Field | Encoding |
|--------|------|-------|----------|
| 0–1 | 2 | X | uint16 **little-endian** |
| 2–3 | 2 | Y | uint16 **little-endian** |
| 4 | 1 | laser | `0` = off, non-zero = on |

Requirements:

- Device must be **enabled**
- `projectMode` must be **POINT**
- Payload length must be exactly 5

### Typical workflow

1. `TOGGLE_PROJECT_MODE_CMD` until status shows POINT (or start from a known PATH state and toggle once)
2. `SET_LASER_DUTY_CYCLE_CMD` (once)
3. Optionally `PLAY` if you want the POINT-mode laser-hold behavior
4. Repeat: `SET_POINT_CMD` → host wait → next `SET_POINT_CMD`
5. Send `laser = 0` to blank without moving galvos

---

## Faults

Faults are latched in `FaultManager` and surfaced via status byte 15 (`has_fault`).

| Host action | Result |
|-------------|--------|
| Poll status | `has_fault == 1` if any fault is latched |
| `GET_FAULT_MSG_CMD` (`0xFC`) | For each latched fault, device writes `0xFF 0x04 0x06` + message + `\n`, then clears all faults |

Fault messages are always prefixed with the debug header, even when debug mode is off, so the host can parse them the same way as debug lines.

Common stream/buffer faults include missed packet index, stream write failure, critically low buffer, and empty ring.

---

## Debug output

When debug mode is on, the device writes:

```
0xFF 0x04 0x06  <UTF-8 text>  0x0A
```

Debug lines are **not** framed with a length field. The host parser must detect the `0xFF 0x04 0x06` prefix, then read until `\n`.

Toggle with `DEBUG_MODE_CMD`. Default at boot is **off**.

---

## Error conditions

| Condition | Device behavior | Host response |
|-----------|-----------------|---------------|
| Ring critically low (&lt; 5%) while ISR running | Stop ISR, `projecting=false`, fault | `RESET_STREAM`, then re-`PLAY`, re-prime buffer |
| Ring empty in ISR | Stop ISR, fault | Refill / `RESET_STREAM` as needed; check faults |
| Missed stream packet index | Frame dropped, fault | `RESET_STREAM`, restart index at 1 |
| Oversize command/stream (&gt;1024 B) | Payload discarded, fault | Use smaller frames |
| `SET_POINT` while disabled / not POINT | Fault | Enable / enter POINT mode |
| Unknown command | `FAULT_CMD_UNKNOWN` | — |
| `has_fault` in status | Latched until cleared | `GET_FAULT_MSG_CMD` |

---

## Reference host main loop

A correct host should:

1. **Drain serial** — parse debug/fault lines, status frames, and raw `PAUSE` / `RESUME` / `RESET` bytes
2. **Send stream frames** — only when local send-gate allows (honor PAUSE/RESUME)
3. **Poll status** — `SEND_STATUS_CMD` periodically; on `has_fault`, call `GET_FAULT_MSG_CMD`

Suggested start:

```python
packet_index = 1
send(RESET_STREAM_CMD)   # framed: cmd + length 0
# optional: wait for raw 0xF7 ack
send(PLAY_CMD)
# send POINT_STREAM_HEADER frames with packet_index = 1, 2, 3, ...
```

On raw `PAUSE_STREAM` (`0xF8`): stop sending.

On raw `RESUME_STREAM` (`0xF9`): resume sending (if session is active).

On `stream_state == RESET` or after faults: re-`RESET_STREAM`, set `packet_index = 1`, then `PLAY` again.

---

## Command quick reference

### Incoming (host → device)

| Hex | Name |
|-----|------|
| `0x07` | PLAY |
| `0x08` | STOP |
| `0x09` | PLAY_PAUSE (toggle) |
| `0x0A` | SET_PLAYBACK_SPEED |
| `0x0B` | SEND_STATUS |
| `0x1A` | DEBUG_MODE |
| `0x1B` | SET_LASER_DUTY_CYCLE |
| `0x3A` | TOGGLE_PROJECT_MODE |
| `0x3B` | SET_POINT |
| `0xAA` | POINT_STREAM (stream data) |
| `0x8E` | RESTART |
| `0x8F` | ENABLE_DISABLE |
| `0xE0` | SLEEP |
| `0xE1` | WAKE |
| `0xE2` | BOOTLOADER |
| `0xF7` | RESET_STREAM |
| `0xFC` | GET_FAULT_MSG |

### Outgoing (device → host)

| Hex | Name | Notes |
|-----|------|-------|
| `0xF7` | RESET_STREAM | Raw 1-byte notification |
| `0xF8` | PAUSE_STREAM | Raw 1-byte notification |
| `0xF9` | RESUME_STREAM | Raw 1-byte notification |
| `0xFA` | RECEIVE_STATUS_PACKET | Framed, 16-byte payload |
| `0xFF` | DEBUG / fault | Prefix `0xFF 0x04 0x06` |

Defines such as `STREAM_POINTS_CMD` (`0x3C`), `ENTER_PATH_MODE_CMD`, `ENTER_POINT_MODE_CMD`, `CLEAR_STREAM_BUFFER_CMD`, and `TOGGLE_CLOSED_TEST_CMD` still exist in `cmd_defs.h` but are **not handled** by the current firmware.

---

## Timing reference (20 kHz)

| Quantity | Value |
|----------|-------|
| Point period | 50 µs |
| Data rate | 100 KB/s |
| Full ring | ~4.9 s of points |
| ISR start threshold | ≥ 25% full (~1.2 s of points) |
| Pause threshold | > 75% full (~3.7 s of points) |
| Critical stop | < 5% full while ISR running |
