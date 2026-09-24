#include "projector.hpp"

#include "serial.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace ccu {
namespace {

int fd_ = -1;

}  // namespace

bool projector_open(const char* path) {
    fd_ = serial_open(path);
    if (fd_ < 0) {
        std::fprintf(stderr, "projector: %s: %s\n", path, std::strerror(errno));
        return false;
    }
    std::printf("projector opened %s\n", path);
    std::fflush(stdout);
    return true;
}

void projector_close() {
    serial_close(fd_);
    fd_ = -1;
}

void projector_poll() {
    if (fd_ < 0) return;
    uint8_t buf[256];
    serial_read(fd_, buf, static_cast<int>(sizeof buf));
}

}  // namespace ccu
