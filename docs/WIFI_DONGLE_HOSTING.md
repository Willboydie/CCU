# ESP-NOW serial bridge — hosting

Plug one flashed ESP32-C3 SuperMini into each of two hosts. The chips pair with each other over ESP-NOW and become a bidirectional wireless serial link. USB enumerates as CDC (Espressif VID `303A` / PID `1001`).

## Host duties

1. Open the USB serial port. Baud is ignored on CDC; if you must set one, use **921600**, 8N1.
2. Do not assert RTS/DTR in a way that holds the chip in reset (many monitors need RTS=0, DTR=0).
3. Send and read **raw binary frames**. CCU ↔ frontend traffic is forwarded unchanged. Dongle-local commands and status (below) are consumed on this USB hop and are **not** sent over ESP-NOW.
4. Do not use command bytes `0xE4`, `0xE5`, or `0xE6` on the CCU ↔ frontend protocol — the dongle intercepts them.
5. On **frontend** and **CCU** program start (including a forced restart of the tablet app or CCU), send `DONGLE_RESET` so both chips reboot and re-pair from a clean state. Then wait for `ready`, then `paired`, before sending link traffic.



## Frame format

Little-endian. No checksum, no start-of-frame magic.

```
[ cmd : uint8 ] [ len : uint16 LE ] [ payload : len bytes ]
```

The dongle trusts `len` and forwards the whole frame (`3 + len` bytes) as soon as it has arrived, unless `cmd` is a dongle-local command. Keep frames **≤ 1024 bytes** (ESP-NOW datagrams are 250 bytes; larger frames are split on air). Zero-length payloads are valid.

## Dongle-local commands

These are host ↔ dongle only. They are never forwarded to the far chip.


| cmd    | Name            | Direction     | Payload                   | Effect                                                                           |
| ------ | --------------- | ------------- | ------------------------- | -------------------------------------------------------------------------------- |
| `0xE4` | `DONGLE_RESET`  | host → dongle | empty (`len = 0`)         | Restart the dongle program                                                       |
| `0xE5` | (pairing)       | chip ↔ chip   | —                         | ESP-NOW twin pairing. Hosts must not send this byte                              |
| `0xE6` | `DONGLE_STATUS` | dongle → host | UTF-8, not NUL-terminated | `"ready"` after the program starts; `"paired"` when this chip pairs to the other |


The host may send `DONGLE_RESET` at any time. Send it whenever the frontend or CCU first initialises, so a tablet-app or CCU restart always forces a fresh reconnection between the two chips.

USB CDC may re-enumerate across a restart. Re-open the port if it drops, then wait for status:

```
 host                         dongle                      peer dongle
   |-- DONGLE_RESET (0xE4) ---->|  program restarts
   |<-- DONGLE_STATUS "ready" --|  (also sent on power-up / USB enumerate)
   |                            |---- ESP-NOW pair ------>|
   |<-- DONGLE_STATUS "paired" -|
```


| Payload  | `len` | Meaning                                                             |
| -------- | ----- | ------------------------------------------------------------------- |
| `ready`  | 5     | Dongle program is running and is accepting host frames              |
| `paired` | 6     | This chip is linked to the other dongle; forwarded traffic can flow |


Do not send CCU ↔ frontend frames until `"paired"`. A later peer loss returns the chip to search (see LED table); a new `"paired"` is sent when the twin is found again.

## Pairing and LED

Both chips must run this firmware. They find the first twin on Wi-Fi channel 1, then unicast host traffic (ACK/retry). Pairing itself needs no host command; `DONGLE_RESET` only restarts the program so that search begins again.


| LED (GPIO 8, blue) | Meaning                           |
| ------------------ | --------------------------------- |
| Fast blink         | Searching for a twin              |
| Slow blink         | Linked (`"paired"` has been sent) |
| Brief solid flash  | Frame TX or RX                    |
| Very fast blink    | Radio init failed                 |


If the peer is lost for ~1.5 s the chip searches again. Up to eight frames are held while unpaired, then sent after link-up.

## Flash another unit

```
pio run -t upload
```

Set `upload_port` in `platformio.ini` to that board’s COM port. Same binary on both ends.