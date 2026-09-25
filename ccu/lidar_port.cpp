#include "lidar.hpp"

#include "serial.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace ccu {
namespace {

int fd_ = -1;

int read_port(uint8_t* dst, int cap) {
    if (fd_ < 0) return 0;
    return serial_read(fd_, dst, cap);
}

int write_port(const uint8_t* src, int n) {
    if (fd_ < 0) return -1;
    return serial_write(fd_, src, n);
}

}  // namespace

bool lidar_open(const char* path) {
    fd_ = serial_open(path);
    if (fd_ < 0) {
        std::fprintf(stderr, "lidar: %s: %s\n", path, std::strerror(errno));
        return false;
    }
    lidar_bind_port(read_port, write_port);
    std::printf("lidar opened %s\n", path);
    std::fflush(stdout);
    return true;
}

void lidar_close() {
    lidar_bind_port(nullptr, nullptr);
    serial_close(fd_);
    fd_ = -1;
}

}  // namespace ccu
