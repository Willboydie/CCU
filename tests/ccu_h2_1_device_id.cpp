// H2.1. Listen on 127.0.0.1, send one idle DEVICE_ID from pack_device_id,
// and require those same bytes back.
// The other program is H2DeviceIdPipeTest, which parses them with SerialFrameParser.
// Idle advertisement: device id "sm-ccu", display name "Smart Marker",
// session inactive, empty session id.

#include "protocol.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#endif

namespace {

#ifdef _WIN32
using Socket = SOCKET;
constexpr Socket kBad = INVALID_SOCKET;
void close_socket(Socket s) {
    if (s != kBad) closesocket(s);
}
int half_close(Socket s) { return ::shutdown(s, SD_SEND); }
#else
using Socket = int;
constexpr Socket kBad = -1;
void close_socket(Socket s) {
    if (s != kBad) ::close(s);
}
int half_close(Socket s) { return ::shutdown(s, SHUT_WR); }
#endif

int fail(const char* why) {
    std::fprintf(stderr, "FAIL: %s\n", why);
    std::printf("H2.1 FAIL\n");
    std::fflush(stdout);
    std::fflush(stderr);
    return 1;
}

bool send_all(Socket s, const uint8_t* data, int n) {
    int off = 0;
    while (off < n) {
        const int w = ::send(s, reinterpret_cast<const char*>(data + off), n - off, 0);
        if (w <= 0) return false;
        off += w;
    }
    return true;
}

bool recv_all(Socket s, uint8_t* data, int n) {
    int off = 0;
    while (off < n) {
        const int r = ::recv(s, reinterpret_cast<char*>(data + off), n - off, 0);
        if (r <= 0) return false;
        off += r;
    }
    return true;
}

void set_timeouts(Socket s) {
#ifdef _WIN32
    const DWORD ms = 10000;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&ms), sizeof ms);
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&ms), sizeof ms);
#else
    timeval tv{};
    tv.tv_sec = 10;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
    setsockopt(s, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof tv);
#endif
}

std::vector<uint8_t> idle_device_id() {
    ccu::DeviceId msg;
    msg.device_id = "sm-ccu";
    msg.display_name = "Smart Marker";
    msg.session_active = false;
    msg.session_id.clear();
    const auto bytes = ccu::pack_device_id(msg);
    if (!bytes) return {};
    return *bytes;
}

}  // namespace

int main() {
    const std::vector<uint8_t> frame = idle_device_id();
    if (frame.empty()) return fail("pack_device_id");

#ifdef _WIN32
    WSADATA wsa{};
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return fail("WSAStartup");
#endif

    const Socket listen_fd = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd == kBad) {
#ifdef _WIN32
        WSACleanup();
#endif
        return fail("socket");
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(0);
    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 ||
        ::listen(listen_fd, 1) != 0) {
        close_socket(listen_fd);
#ifdef _WIN32
        WSACleanup();
#endif
        return fail("listen");
    }

    sockaddr_in bound{};
#ifdef _WIN32
    int bound_len = sizeof bound;
#else
    socklen_t bound_len = sizeof bound;
#endif
    if (::getsockname(listen_fd, reinterpret_cast<sockaddr*>(&bound), &bound_len) != 0) {
        close_socket(listen_fd);
#ifdef _WIN32
        WSACleanup();
#endif
        return fail("getsockname");
    }

    std::printf("H2.1 PORT %u\n", static_cast<unsigned>(ntohs(bound.sin_port)));
    std::fflush(stdout);

    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(listen_fd, &fds);
    timeval wait{};
    wait.tv_sec = 20;
#ifdef _WIN32
    const int ready = ::select(0, &fds, nullptr, nullptr, &wait);
#else
    const int ready = ::select(static_cast<int>(listen_fd) + 1, &fds, nullptr, nullptr, &wait);
#endif
    if (ready <= 0) {
        close_socket(listen_fd);
#ifdef _WIN32
        WSACleanup();
#endif
        return fail(ready == 0 ? "accept timeout" : "select");
    }

    const Socket client = ::accept(listen_fd, nullptr, nullptr);
    close_socket(listen_fd);
    if (client == kBad) {
#ifdef _WIN32
        WSACleanup();
#endif
        return fail("accept");
    }
    set_timeouts(client);

    int rc = 0;
    const int n = static_cast<int>(frame.size());
    if (!send_all(client, frame.data(), n) || half_close(client) != 0) {
        rc = fail("send");
    } else {
        std::vector<uint8_t> got(frame.size());
        if (!recv_all(client, got.data(), n)) {
            rc = fail("recv");
        } else if (std::memcmp(got.data(), frame.data(), frame.size()) != 0) {
            rc = fail("echo mismatch");
        } else {
            const auto back = ccu::unpack_device_id(got);
            if (!back || back->device_id != "sm-ccu" || back->display_name != "Smart Marker" ||
                back->session_active || !back->session_id.empty()) {
                rc = fail("echo is not an idle DEVICE_ID");
            } else {
                std::printf("H2.1 PASS\n");
                std::fflush(stdout);
            }
        }
    }

    close_socket(client);
#ifdef _WIN32
    WSACleanup();
#endif
    return rc;
}
