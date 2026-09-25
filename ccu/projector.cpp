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
    if (fd_ >= 0) {
        uint8_t buf[256];
        serial_read(fd_, buf, static_cast<int>(sizeof buf));
        const std::vector<std::vector<uint8_t>> frames = projector_take_frames();
        for (const std::vector<uint8_t>& frame : frames) {
            if (!frame.empty()) serial_write(fd_, frame.data(), static_cast<int>(frame.size()));
        }
    }
}

}  // namespace ccu
