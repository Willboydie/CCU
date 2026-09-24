#pragma once

#include <cstdint>

namespace ccu {

// POSIX tty. Returns a fd, or -1 on failure (errno set).
int serial_open(const char* path);
void serial_close(int fd);

// >0 bytes, 0 if none, -1 on a hard error.
int serial_read(int fd, uint8_t* dst, int cap);
// Bytes written, or -1 on a hard error.
int serial_write(int fd, const uint8_t* src, int n);

}  // namespace ccu
