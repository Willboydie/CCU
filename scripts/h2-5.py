#!/usr/bin/env python3
"""H2.5. One Wi-Fi dongle on this host, the other on the other machine.

Same program on the Pi and the PC. The C3 USB port stays quiet until the host
writes a packet, so this writes an empty status frame first. The H2.0 payload
then rides inside one forwarded frame (cmd 0x7F, length 16).
On Windows the PC dongle is COM9.
"""

import os
import sys
import time

PAYLOAD = bytes([
    0x10, 0x01, 0x02, 0x03, 0x00, 0xFF, 0x7F, 0x80,
    0xE4, 0xE5, 0xE6, 0x0A, 0x0D, 0x20, 0x41, 0x00,
])
FRAME = bytes([0x7F, len(PAYLOAD), 0x00]) + PAYLOAD
POKE = bytes([0xE6, 0x00, 0x00])
ORIENTATION = "98:A3:16:D3:F6:4C"
PC_PORT = "COM9"
SIDE = "pc" if os.name == "nt" else "pi"


class Parser:
    def __init__(self):
        self.phase = "cmd"
        self.length = 0
        self.frame = bytearray()
        self.ready = False
        self.paired = False
        self.got = False
        self.rx = 0
        self.preview = bytearray()

    def feed(self, data):
        for byte in data:
            self.rx += 1
            if len(self.preview) < 32:
                self.preview.append(byte)
            self._byte(byte)

    def _byte(self, byte):
        if self.phase == "cmd":
            self.frame = bytearray([byte])
            self.phase = "lo"
        elif self.phase == "lo":
            self.frame.append(byte)
            self.phase = "hi"
        elif self.phase == "hi":
            self.frame.append(byte)
            self.length = self.frame[1] | (self.frame[2] << 8)
            if self.length > 1024:
                self.phase = "cmd"
                return
            if self.length == 0:
                self._frame()
                self.phase = "cmd"
                return
            self.phase = "body"
        else:
            self.frame.append(byte)
            if len(self.frame) == 3 + self.length:
                self._frame()
                self.phase = "cmd"

    def _frame(self):
        cmd = self.frame[0]
        payload = bytes(self.frame[3:])
        if cmd == 0xE6:
            text = payload.decode("utf-8", "replace")
            print('H2.5 %s dongle "%s"' % (SIDE, text), flush=True)
            if payload == b"ready":
                self.ready = True
            if payload == b"paired":
                self.paired = True
        elif cmd == 0x7F and payload == PAYLOAD:
            self.got = True


def fail(why):
    print("FAIL: %s" % why, file=sys.stderr)
    print("H2.5 FAIL", flush=True)
    return 1


def linux_dongles():
    root = "/dev/serial/by-id"
    found = []
    if not os.path.isdir(root):
        return found
    for name in sorted(os.listdir(root)):
        if "Espressif" not in name:
            continue
        if ORIENTATION.lower() in name.lower():
            continue
        link = os.path.join(root, name)
        stem = name
        if "-if" in stem:
            stem = stem[: stem.rfind("-if")]
        serial = stem[stem.rfind("_") + 1 :] if "_" in stem else stem
        found.append((link, os.path.realpath(link), serial))
    return found


def windows_dongles():
    import winreg

    def port_for_prefix(prefix):
        path = r"SYSTEM\CurrentControlSet\Enum\USB\VID_303A&PID_1001&MI_00\%s&0000\Device Parameters" % prefix
        try:
            params = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, path)
            port, _typ = winreg.QueryValueEx(params, "PortName")
        except OSError:
            return None
        return port

    present = set()
    try:
        comm = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, r"HARDWARE\DEVICEMAP\SERIALCOMM")
        index = 0
        while True:
            try:
                _name, value, _typ = winreg.EnumValue(comm, index)
            except OSError:
                break
            index += 1
            present.add(value)
    except OSError:
        return []

    found = []
    root = r"SYSTEM\CurrentControlSet\Enum\USB\VID_303A&PID_1001"
    try:
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, root)
    except OSError:
        return found
    index = 0
    while True:
        try:
            serial = winreg.EnumKey(key, index)
        except OSError:
            break
        index += 1
        if ORIENTATION.lower() in serial.lower():
            continue
        try:
            dev = winreg.OpenKey(key, serial)
            prefix, _typ = winreg.QueryValueEx(dev, "ParentIdPrefix")
        except OSError:
            continue
        port = port_for_prefix(prefix)
        if port and port in present:
            found.append((port, port, serial))
    return found


def dongles():
    if len(sys.argv) == 2:
        name = sys.argv[1]
        return [(name, name, name)]
    if os.name == "nt":
        for item in windows_dongles():
            if item[0].upper() == PC_PORT:
                return [item]
        return [(PC_PORT, PC_PORT, PC_PORT)]
    return linux_dongles()


class LinuxPort:
    def __init__(self, path):
        import termios

        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attrs = termios.tcgetattr(self.fd)
        attrs[0] = 0
        attrs[1] = 0
        attrs[2] |= termios.CLOCAL | termios.CREAD
        crtscts = getattr(termios, "CRTSCTS", 0)
        hupcl = getattr(termios, "HUPCL", 0)
        attrs[2] &= ~(crtscts | hupcl)
        attrs[3] = 0
        speed = getattr(termios, "B921600", termios.B115200)
        attrs[4] = speed
        attrs[5] = speed
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attrs)

    def write(self, data):
        os.write(self.fd, data)

    def read(self, n):
        try:
            return os.read(self.fd, n)
        except BlockingIOError:
            return b""

    def close(self):
        os.close(self.fd)


class WindowsPort:
    def __init__(self, port):
        import ctypes
        from ctypes import wintypes

        self.kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
        self.kernel32.CreateFileW.argtypes = [
            wintypes.LPCWSTR, wintypes.DWORD, wintypes.DWORD, ctypes.c_void_p,
            wintypes.DWORD, wintypes.DWORD, wintypes.HANDLE,
        ]
        self.kernel32.CreateFileW.restype = ctypes.c_void_p
        path = r"\\.\%s" % port
        self.handle = self.kernel32.CreateFileW(
            path, 0x80000000 | 0x40000000, 0, None, 3, 0, None)
        invalid = ctypes.c_void_p(-1).value
        if not self.handle or self.handle == invalid:
            raise OSError(ctypes.get_last_error(), "open %s" % port)
        self._configure()
        self.kernel32.EscapeCommFunction(self.handle, 6)  # CLRDTR
        self.kernel32.EscapeCommFunction(self.handle, 4)  # CLRRTS

    def _configure(self):
        import ctypes
        from ctypes import wintypes

        class DCB(ctypes.Structure):
            _fields_ = [
                ("DCBlength", wintypes.DWORD),
                ("BaudRate", wintypes.DWORD),
                ("fBinary", wintypes.DWORD, 1),
                ("fParity", wintypes.DWORD, 1),
                ("fOutxCtsFlow", wintypes.DWORD, 1),
                ("fOutxDsrFlow", wintypes.DWORD, 1),
                ("fDtrControl", wintypes.DWORD, 2),
                ("fDsrSensitivity", wintypes.DWORD, 1),
                ("fTXContinueOnXoff", wintypes.DWORD, 1),
                ("fOutX", wintypes.DWORD, 1),
                ("fInX", wintypes.DWORD, 1),
                ("fErrorChar", wintypes.DWORD, 1),
                ("fNull", wintypes.DWORD, 1),
                ("fRtsControl", wintypes.DWORD, 2),
                ("fAbortOnError", wintypes.DWORD, 1),
                ("fDummy2", wintypes.DWORD, 17),
                ("wReserved", wintypes.WORD),
                ("XonLim", wintypes.WORD),
                ("XoffLim", wintypes.WORD),
                ("ByteSize", wintypes.BYTE),
                ("Parity", wintypes.BYTE),
                ("StopBits", wintypes.BYTE),
                ("XonChar", ctypes.c_char),
                ("XoffChar", ctypes.c_char),
                ("ErrorChar", ctypes.c_char),
                ("EofChar", ctypes.c_char),
                ("EvtChar", ctypes.c_char),
                ("wReserved1", wintypes.WORD),
            ]

        class COMMTIMEOUTS(ctypes.Structure):
            _fields_ = [
                ("ReadIntervalTimeout", wintypes.DWORD),
                ("ReadTotalTimeoutMultiplier", wintypes.DWORD),
                ("ReadTotalTimeoutConstant", wintypes.DWORD),
                ("WriteTotalTimeoutMultiplier", wintypes.DWORD),
                ("WriteTotalTimeoutConstant", wintypes.DWORD),
            ]

        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        if not self.kernel32.GetCommState(self.handle, ctypes.byref(dcb)):
            raise OSError(ctypes.get_last_error(), "GetCommState")
        dcb.BaudRate = 921600
        dcb.ByteSize = 8
        dcb.Parity = 0
        dcb.StopBits = 0
        dcb.fBinary = 1
        dcb.fParity = 0
        dcb.fOutxCtsFlow = 0
        dcb.fOutxDsrFlow = 0
        dcb.fDtrControl = 0
        dcb.fRtsControl = 0
        dcb.fOutX = 0
        dcb.fInX = 0
        dcb.fAbortOnError = 0
        if not self.kernel32.SetCommState(self.handle, ctypes.byref(dcb)):
            raise OSError(ctypes.get_last_error(), "SetCommState")
        timeouts = COMMTIMEOUTS()
        timeouts.ReadIntervalTimeout = 0xFFFFFFFF
        timeouts.ReadTotalTimeoutMultiplier = 0
        timeouts.ReadTotalTimeoutConstant = 0
        timeouts.WriteTotalTimeoutMultiplier = 0
        timeouts.WriteTotalTimeoutConstant = 1000
        if not self.kernel32.SetCommTimeouts(self.handle, ctypes.byref(timeouts)):
            raise OSError(ctypes.get_last_error(), "SetCommTimeouts")

    def write(self, data):
        import ctypes
        from ctypes import wintypes

        done = 0
        while done < len(data):
            wrote = wintypes.DWORD()
            buf = ctypes.create_string_buffer(data[done:])
            ok = self.kernel32.WriteFile(
                self.handle, buf, len(data) - done, ctypes.byref(wrote), None)
            if not ok or wrote.value == 0:
                raise OSError(ctypes.get_last_error(), "write")
            done += wrote.value

    def read(self, n):
        import ctypes
        from ctypes import wintypes

        buf = ctypes.create_string_buffer(n)
        got = wintypes.DWORD()
        ok = self.kernel32.ReadFile(self.handle, buf, n, ctypes.byref(got), None)
        if not ok:
            err = ctypes.get_last_error()
            if err in (0, 995, 996):
                return b""
            raise OSError(err, "read")
        return buf.raw[: got.value]

    def close(self):
        self.kernel32.CloseHandle(self.handle)


def open_port(path):
    if os.name == "nt":
        return WindowsPort(path)
    return LinuxPort(path)


def main():
    found = dongles()
    if len(found) != 1:
        print("Wi-Fi dongles on this host: %d (need 1; the other belongs on the other machine)"
              % len(found), file=sys.stderr)
        for _link, tty, serial in found:
            print("  %s  %s" % (tty, serial), file=sys.stderr)
        return fail("dongle count")
    link, tty, serial = found[0]
    print("H2.5 %s %s %s" % (SIDE, tty, serial), flush=True)
    port = open_port(link)
    parser = Parser()
    deadline = time.monotonic() + 60
    got_at = None
    try:
        while time.monotonic() < deadline:
            port.write(POKE)
            time.sleep(0.05)
            port.write(FRAME)
            data = port.read(256)
            if data:
                parser.feed(data)
            if parser.got and got_at is None:
                got_at = time.monotonic()
                print("H2.5 %s received" % SIDE, flush=True)
            if got_at is not None and time.monotonic() - got_at >= 15:
                print("H2.5 PASS", flush=True)
                return 0
            time.sleep(0.2)
    finally:
        port.close()
    print("paired=%d bytes=%d" % (1 if parser.paired else 0, parser.rx), file=sys.stderr)
    preview = " ".join("%02X" % b for b in parser.preview) or "(none)"
    print("  hex: %s" % preview, file=sys.stderr)
    return fail("no payload")


if __name__ == "__main__":
    sys.exit(main())
