#include "serial.hpp"

#include <cerrno>
#include <chrono>
#include <thread>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace ccu {

int serial_open(const char* path) {
    const int fd = ::open(path, O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) return -1;

    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        ::close(fd);
        return -1;
    }
    cfmakeraw(&tio);
    cfsetispeed(&tio, B921600);
    cfsetospeed(&tio, B921600);
    tio.c_cflag |= (CLOCAL | CREAD);
    // HUPCL would drop DTR on close and reboot a CDC chip that is still enumerating.
    tio.c_cflag &= ~(CRTSCTS | HUPCL);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        ::close(fd);
        return -1;
    }

    // DTR means the host is connected. RTS stays clear so the chip is not held in boot.
    int bits = TIOCM_DTR;
    ioctl(fd, TIOCMBIS, &bits);
    bits = TIOCM_RTS;
    ioctl(fd, TIOCMBIC, &bits);
    return fd;
}

void serial_close(int fd) {
    if (fd >= 0) ::close(fd);
}

int serial_read(int fd, uint8_t* dst, int cap) {
    if (fd < 0 || dst == nullptr || cap <= 0) return -1;
    const ssize_t n = ::read(fd, dst, static_cast<size_t>(cap));
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;
        return -1;
    }
    return static_cast<int>(n);
}

int serial_write(int fd, const uint8_t* src, int n) {
    if (fd < 0 || src == nullptr || n < 0) return -1;
    int off = 0;
    int spins = 0;
    while (off < n) {
        const ssize_t w = ::write(fd, src + off, static_cast<size_t>(n - off));
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (++spins > 50) return -1;
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
                continue;
            }
            return -1;
        }
        off += static_cast<int>(w);
        spins = 0;
    }
    return off;
}

}  // namespace ccu
