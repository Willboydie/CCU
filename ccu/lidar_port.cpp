#include "lidar.hpp"

#include "serial.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace ccu {
namespace {

int fd_ = -1;

}  // namespace

bool lidar_open(const char* path) {
    fd_ = serial_open(path);
    if (fd_ < 0) {
        std::fprintf(stderr, "lidar: %s: %s\n", path, std::strerror(errno));
        return false;
    }
    std::printf("lidar opened %s\n", path);
    std::fflush(stdout);
    return true;
}

void lidar_close() {
    serial_close(fd_);
    fd_ = -1;
}

}  // namespace ccu
