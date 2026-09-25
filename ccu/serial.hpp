#pragma once

#include <cstdint>

namespace ccu {

// POSIX tty. Returns a fd, or -1 on failure (errno set).
// baud 921600 is the angle and projector rate. The lidar opens at 115200,
// the rate the bench swap used.
// assert_rts: leave this clear for the lidar and for the ESP32 orientation
// board. Asserting RTS on the ESP32 holds it in reset.
// flush_rx: drop bytes already queued on the host. The lidar leaves this
// clear so the open matches the bench host that delivered bytes.
int serial_open(const char* path, int baud = 921600, bool assert_rts = false, bool flush_rx = true);

// The open used by the bench host that registered lidar commands.
// 115200, raw, no RTS change, receive buffer left in place.
int serial_open_link(const char* path);
void serial_close(int fd);

// >0 bytes, 0 if none, -1 on a hard error.
int serial_read(int fd, uint8_t* dst, int cap);
// Bytes written, or -1 on a hard error.
int serial_write(int fd, const uint8_t* src, int n);

}  // namespace ccu
