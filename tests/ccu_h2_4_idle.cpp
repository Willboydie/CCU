// H2.4. Paired is injected. One idle session_poll produces DEVICE_ID.
// H2IdleSessionPipeTest is the reader.

#include "dongle_link.hpp"
#include "protocol.hpp"
#include "session.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
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

namespace ccu {
void dongle_poll() {}
void angle_poll() {}
void projector_poll() {}
}

namespace {

struct Capture {
    std::vector<uint8_t> tx;
};

int capture_write(void* ctx, const uint8_t* data, int n) {
    auto* cap = static_cast<Capture*>(ctx);
    cap->tx.insert(cap->tx.end(), data, data + n);
    return n;
}

std::vector<uint8_t> status_frame(const char* text) {
    const auto n = static_cast<uint8_t>(std::strlen(text));
    std::vector<uint8_t> frame{0xE6, n, 0x00};
    frame.insert(frame.end(), text, text + n);
    return frame;
}

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
    std::printf("H2.4 FAIL\n");
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

}  // namespace

int main() {
    ccu::session_reset();
    Capture cap;
    ccu::dongle_link_attach(capture_write, &cap);
    const std::vector<uint8_t> paired = status_frame("paired");
    ccu::dongle_link_feed(std::span<const uint8_t>(paired.data(), paired.size()));
    if (!ccu::dongle_link_paired()) return fail("paired");
    ccu::session_poll();
    if (cap.tx.empty()) return fail("idle loop produced no bytes");

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

    std::printf("H2.4 PORT %u\n", static_cast<unsigned>(ntohs(bound.sin_port)));
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
    const int n = static_cast<int>(cap.tx.size());
    if (!send_all(client, cap.tx.data(), n) || half_close(client) != 0) {
        rc = fail("send");
    } else {
        std::vector<uint8_t> got(cap.tx.size());
        if (!recv_all(client, got.data(), n)) {
            rc = fail("recv");
        } else if (std::memcmp(got.data(), cap.tx.data(), cap.tx.size()) != 0) {
            rc = fail("echo mismatch");
        } else {
            const auto back = ccu::unpack_device_id(got);
            if (!back || back->device_id != "sm-ccu" || back->display_name != "Smart Marker" ||
                back->session_active || !back->session_id.empty()) {
                rc = fail("idle loop bytes are not an idle DEVICE_ID");
            } else {
                std::printf("H2.4 PASS\n");
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
