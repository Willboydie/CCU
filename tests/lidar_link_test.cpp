// Read the lidar port the way ccu does and report whether status frames arrive.
//
//   ./build/ccu_lidar_link /dev/ttyACM3

#include "serial.hpp"
#include "utils/protocol.hpp"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include <sys/ioctl.h>

namespace {

int modem_bits(int fd) {
    int bits = 0;
    if (ioctl(fd, TIOCMGET, &bits) != 0) return -1;
    return bits;
}

struct Stats {
    int bytes = 0;
    int status_n = 0;
    int control_n = 0;
    int debug_n = 0;
    int enabled = -1;
    int scan = -1;
    int debug_mode = -1;
    int fault = -1;
    std::vector<uint8_t> head;
    std::vector<uint8_t> rx;
};

void feed(Stats& stats) {
    while (stats.rx.size() >= 3) {
        const uint8_t header = stats.rx[0];
        const uint16_t len = static_cast<uint16_t>(stats.rx[1] | (static_cast<uint16_t>(stats.rx[2]) << 8));
        auto take = [&](std::size_t n) {
            stats.rx.erase(stats.rx.begin(), stats.rx.begin() + static_cast<std::ptrdiff_t>(n));
        };
        if (header == kStatusHeader && len == kStatusPayloadSize) {
            if (stats.rx.size() < kStatusPacketSize) return;
            stats.enabled = stats.rx[3];
            stats.scan = stats.rx[4];
            stats.debug_mode = stats.rx[6];
            stats.fault = stats.rx[7];
            ++stats.status_n;
            take(kStatusPacketSize);
        } else if (header == kControlHeader && len == kControlPayloadSize) {
            if (stats.rx.size() < kControlPacketSize) return;
            ++stats.control_n;
            take(kControlPacketSize);
        } else if (header == kDebugHeader && len <= kDebugMaxLen) {
            if (stats.rx.size() < static_cast<std::size_t>(3 + len)) return;
            ++stats.debug_n;
            take(static_cast<std::size_t>(3 + len));
        } else {
            take(1);
        }
    }
}

void read_for(int fd, Stats& stats, double seconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    uint8_t buf[4096];
    while (std::chrono::steady_clock::now() < end) {
        const int n = ccu::serial_read(fd, buf, static_cast<int>(sizeof buf));
        if (n < 0) {
            std::printf("read error\n");
            return;
        }
        if (n == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        stats.bytes += n;
        if (stats.head.size() < 32) {
            const int keep = std::min(n, 32 - static_cast<int>(stats.head.size()));
            stats.head.insert(stats.head.end(), buf, buf + keep);
        }
        stats.rx.insert(stats.rx.end(), buf, buf + n);
        feed(stats);
    }
}

void print_stats(const char* label, const Stats& stats) {
    std::printf("%s bytes=%d status=%d control=%d debug_frames=%d enabled=%d scan=%d debug_mode=%d fault=%d\n",
                label, stats.bytes, stats.status_n, stats.control_n, stats.debug_n,
                stats.enabled, stats.scan, stats.debug_mode, stats.fault);
    if (stats.status_n == 0 && !stats.head.empty()) {
        std::printf("%s head", label);
        for (uint8_t b : stats.head) std::printf(" %02x", b);
        std::printf("\n");
    }
    std::fflush(stdout);
}

}  // namespace

int main(int argc, char** argv) {
    const char* path = argc > 1 ? argv[1] : "/dev/ttyACM3";
    const int fd = ccu::serial_open(path, 115200, false, false);
    if (fd < 0) {
        std::printf("lidar-link FAIL open %s: %s\n", path, std::strerror(errno));
        return 1;
    }
    const int bits = modem_bits(fd);
    std::printf("open %s fd=%d modem=0x%x dtr=%d rts=%d\n", path, fd, bits,
                bits >= 0 && (bits & TIOCM_DTR) ? 1 : 0,
                bits >= 0 && (bits & TIOCM_RTS) ? 1 : 0);

    Stats stats;
    read_for(fd, stats, 1.5);
    print_stats("before", stats);

    if (stats.status_n == 0) {
        ccu::serial_close(fd);
        std::printf("lidar-link FAIL no status frames\n");
        return 1;
    }

    const uint8_t resync[] = {0xFF, 0x11, 0x00};
    for (int i = 0; i < 3; ++i) {
        const int n = ccu::serial_write(fd, resync, 3);
        std::printf("sent ff 11 00 resync wrote=%d\n", n);
    }
    read_for(fd, stats, 0.2);

    const int debug_before = stats.debug_mode;
    const int status_before = stats.status_n;
    const uint8_t debug_toggle[] = {0x02, 0x00, 0x00};
    const int wrote_debug = ccu::serial_write(fd, debug_toggle, 3);
    std::printf("sent 02 00 00 set_debug_mode wrote=%d\n", wrote_debug);
    read_for(fd, stats, 1.0);
    print_stats("after-debug", stats);
    const bool debug_changed = stats.status_n > status_before && stats.debug_mode != debug_before;
    if (debug_changed) {
        const int wrote_back = ccu::serial_write(fd, debug_toggle, 3);
        std::printf("sent 02 00 00 again to restore wrote=%d\n", wrote_back);
        read_for(fd, stats, 0.5);
        print_stats("restored", stats);
    }

    const int enabled_before = stats.enabled;
    const int status_mid = stats.status_n;
    const uint8_t enable[] = {0x01, 0x00, 0x00};
    const int wrote_enable = ccu::serial_write(fd, enable, 3);
    std::printf("sent 01 00 00 enable wrote=%d\n", wrote_enable);
    read_for(fd, stats, 1.0);
    print_stats("after-enable", stats);
    const bool enable_changed = stats.status_n > status_mid && stats.enabled != enabled_before;

    ccu::serial_close(fd);
    if (!debug_changed && !enable_changed) {
        std::printf("lidar-link FAIL lidar did not change a status bit after host bytes\n");
        return 1;
    }
    std::printf("lidar-link PASS lidar received host serial (debug_changed=%d enable_changed=%d)\n",
                debug_changed ? 1 : 0, enable_changed ? 1 : 0);
    return 0;
}
