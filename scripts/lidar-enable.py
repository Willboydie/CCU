#!/usr/bin/env python3
"""Toggle lidar motor enable and print StatusPacket.enabled.

Opens the gimbal the same way as lidar_transceiver: 460800 8N1, DTR and RTS
asserted. Stop ccu first; it owns the port.

  python3 scripts/lidar-enable.py
  python3 scripts/lidar-enable.py /dev/ttyACM3
"""

import array
import fcntl
import os
import select
import sys
import termios
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM3"
ENABLE = bytes((0x01, 0x00, 0x00))
STATUS_LEN = 21
CONTROL_LEN = 26

TIOCMGET = getattr(termios, "TIOCMGET", 0x5415)
TIOCMBIS = getattr(termios, "TIOCMBIS", 0x5416)
TIOCM_DTR = getattr(termios, "TIOCM_DTR", 0x002)
TIOCM_RTS = getattr(termios, "TIOCM_RTS", 0x004)


class Parser:
    def __init__(self):
        self.buf = bytearray()
        self.status_n = 0
        self.control_n = 0
        self.enabled = None
        self.scan = None
        self.routine = None
        self.debug_mode = None
        self.fault = None
        self.notes = []

    def feed(self, data):
        self.buf.extend(data)
        while len(self.buf) >= 3:
            header = self.buf[0]
            length = self.buf[1] | (self.buf[2] << 8)
            if header == 0xAA and length == STATUS_LEN:
                if len(self.buf) < 3 + STATUS_LEN:
                    return
                self.enabled = self.buf[3]
                self.scan = self.buf[4]
                self.routine = self.buf[5]
                self.debug_mode = self.buf[6]
                self.fault = self.buf[7]
                self.status_n += 1
                del self.buf[: 3 + STATUS_LEN]
            elif header == 0x55 and length == CONTROL_LEN:
                if len(self.buf) < 3 + CONTROL_LEN:
                    return
                self.control_n += 1
                del self.buf[: 3 + CONTROL_LEN]
            elif header == 0xFF and length <= 256:
                if len(self.buf) < 3 + length:
                    return
                text = bytes(self.buf[3 : 3 + length]).decode("ascii", "replace")
                if text and len(self.notes) < 8:
                    self.notes.append(text)
                del self.buf[: 3 + length]
            else:
                del self.buf[0]


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B460800
    attrs[5] = termios.B460800
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    bits = array.array("I", [TIOCM_DTR | TIOCM_RTS])
    fcntl.ioctl(fd, TIOCMBIS, bits)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def read_for(fd, parser, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        remain = deadline - time.monotonic()
        ready, _, _ = select.select([fd], [], [], min(0.1, remain))
        if not ready:
            continue
        try:
            chunk = os.read(fd, 4096)
        except BlockingIOError:
            continue
        if chunk:
            parser.feed(chunk)


def main():
    try:
        fd = open_port(PORT)
    except OSError as exc:
        print(f"lidar-enable FAIL open {PORT}: {exc}")
        return 1
    parser = Parser()
    try:
        read_for(fd, parser, 1.0)
        print(
            f"before status_n={parser.status_n} control_n={parser.control_n} "
            f"enabled={parser.enabled} scan={parser.scan} routine={parser.routine} "
            f"debug={parser.debug_mode} fault={parser.fault}"
        )
        if parser.status_n == 0:
            print("lidar-enable FAIL no status before the toggle")
            return 1
        written = os.write(fd, ENABLE)
        print(f"sent 01 00 00 wrote={written}")
        before = parser.enabled
        read_for(fd, parser, 2.0)
        print(
            f"after status_n={parser.status_n} control_n={parser.control_n} "
            f"enabled={parser.enabled} scan={parser.scan} routine={parser.routine} "
            f"debug={parser.debug_mode} fault={parser.fault}"
        )
        for note in parser.notes:
            print(f"debug: {note}")
        os.write(fd, bytes((0x1F, 0x00, 0x00)))
        print("sent 1f 00 00 get_error_msg")
        read_for(fd, parser, 0.5)
        for note in parser.notes:
            print(f"debug: {note}")
        if before == 0 and parser.enabled == 1:
            print("lidar-enable PASS motors reported enabled")
            return 0
        if before == 1 and parser.enabled == 0:
            print("lidar-enable PASS toggle ran; motors reported disabled")
            print("sent 01 00 00 again to turn them back on")
            os.write(fd, ENABLE)
            read_for(fd, parser, 1.0)
            print(f"final enabled={parser.enabled}")
            if parser.enabled == 1:
                return 0
        print("lidar-enable FAIL enabled did not change")
        return 1
    finally:
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
