// Proves the CCU serial path and the lidar firmware swap the same bytes.
// With no path argument, checks the reply the firmware is required to send.
// With a tty path, opens that port the way the bench link did and exchanges lines.
//
//   ./ccu_byte_swap
//   ./ccu_byte_swap /dev/serial/by-id/usb-Teensyduino_USB_Serial_18997040-if00

#include "serial.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>

namespace {

std::string reversed(const std::string& text) {
    return std::string(text.rbegin(), text.rend());
}

std::string expect_for(const std::string& text) {
    if (text.empty()) return "SWAP\n";
    return "SWAP " + reversed(text) + "\n";
}

bool logic_ok() {
    const char* samples[] = {"hello", "pi-teensy", "abc123", ""};
    const char* wants[] = {"SWAP olleh\n", "SWAP ysneet-ip\n", "SWAP 321cba\n", "SWAP\n"};
    for (int i = 0; i < 4; ++i) {
        const std::string got = expect_for(samples[i]);
        if (got != wants[i]) {
            std::printf("byte-swap FAIL logic %s -> %s\n", samples[i], got.c_str());
            return false;
        }
    }
    std::printf("byte-swap logic ok\n");
    return true;
}

bool wait_for(int fd, const std::string& needle, int timeout_ms) {
    std::string buf;
    const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    uint8_t tmp[256];
    while (std::chrono::steady_clock::now() < end) {
        const int n = ccu::serial_read(fd, tmp, static_cast<int>(sizeof tmp));
        if (n < 0) return false;
        if (n == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        buf.append(reinterpret_cast<char*>(tmp), static_cast<size_t>(n));
        if (buf.find(needle) != std::string::npos) return true;
        if (buf.size() > 8192) buf.erase(0, buf.size() - 1024);
    }
    return false;
}

bool exchange(int fd, const std::string& text) {
    const std::string frame = text + "\n";
    const std::string needle = expect_for(text);
    const auto bytes = reinterpret_cast<const uint8_t*>(frame.data());
    for (int attempt = 0; attempt < 20; ++attempt) {
        if (ccu::serial_write(fd, bytes, static_cast<int>(frame.size())) < 0) return false;
        if (wait_for(fd, needle, 400)) {
            std::printf("tx: %s  rx: %s", text.c_str(), needle.c_str());
            return true;
        }
    }
    std::printf("byte-swap FAIL no reply for %s\n", text.c_str());
    return false;
}

}  // namespace

int main(int argc, char** argv) {
    if (!logic_ok()) return 1;
    if (argc < 2) {
        std::printf("byte-swap PASS logic\n");
        return 0;
    }

    const char* path = argv[1];
    const int fd = ccu::serial_open(path, 115200, false, false);
    if (fd < 0) {
        std::printf("byte-swap FAIL open %s\n", path);
        return 1;
    }

    const char* messages[] = {"hello", "pi-teensy", "abc123"};
    int failed = 0;
    for (const char* message : messages) {
        if (!exchange(fd, message)) ++failed;
    }
    ccu::serial_close(fd);
    if (failed != 0) {
        std::printf("byte-swap FAIL %d exchange(s)\n", failed);
        return 1;
    }
    std::printf("byte-swap PASS swapped 3 lines\n");
    return 0;
}
