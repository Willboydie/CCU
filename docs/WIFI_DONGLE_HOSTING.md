# ESP-NOW serial bridge — hosting

Plug one flashed ESP32-C3 SuperMini into each of two hosts. The chips pair with each other over ESP-NOW and become a bidirectional wireless serial link. USB enumerates as CDC (Espressif VID `303A` / PID `1001`).

## Host duties

1. Open the USB serial port. Baud is ignored on CDC; if you must set one, use **921600**, 8N1.
2. Do not assert RTS/DTR in a way that holds the chip in reset (many monitors need RTS=0, DTR=0).
3. Send and read **raw binary frames only**. The chip never prints text.
4. Do not use command byte `0xE5` — that is reserved for chip-to-chip pairing and is not forwarded to the far host.

## Frame format

Little-endian. No checksum, no start-of-frame magic.

```
[ cmd : uint8 ] [ len : uint16 LE ] [ payload : len bytes ]
```

The dongle trusts `len` and forwards the whole frame (`3 + len` bytes) as soon as it has arrived. Keep frames **≤ 1024 bytes** (ESP-NOW datagrams are 250 bytes; larger frames are split on air). Zero-length payloads are valid.

## Pairing and LED

Both chips must run this firmware. They find the first twin on Wi-Fi channel 1, then unicast host traffic (ACK/retry). No host command is required.

| LED (GPIO 8, blue) | Meaning |
| --- | --- |
| Fast blink | Searching for a twin |
| Slow blink | Linked |
| Brief solid flash | Frame TX or RX |
| Very fast blink | Radio init failed |

If the peer is lost for ~1.5 s the chip searches again. Up to eight frames are held while unpaired, then sent after link-up.

## Flash another unit

```
pio run -t upload
```

Set `upload_port` in `platformio.ini` to that board’s COM port. Same binary on both ends.
