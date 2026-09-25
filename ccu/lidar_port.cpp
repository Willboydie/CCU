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
    fd_ = serial_open_link(path);
    if (fd_ < 0) {
        std::fprintf(stderr, "lidar: %s: %s\n", path, std::strerror(errno));
        return false;
    }
    lidar_bind_port(read_port, write_port);
    // Same three bytes the link test writes before a command. Length 17 is
    // oversize, so the firmware parser drops the header and waits for a command.
    const uint8_t resync[] = {0xFF, 0x11, 0x00};
    for (int i = 0; i < 3; ++i) {
        if (serial_write(fd_, resync, 3) < 0) {
            std::fprintf(stderr, "lidar resync failed\n");
            break;
        }
    }
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
