#include "dongle.hpp"

#include "dongle_link.hpp"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

namespace ccu {
namespace {

constexpr int kDefaultPort = 8765;
constexpr int kBeaconPort = 8766;
constexpr char kBeaconFind[] = "SMFIND";
constexpr char kBeaconHere[] = "SMHERE";

int listen_fd_ = -1;
int client_fd_ = -1;
int beacon_fd_ = -1;

int set_nonblock(int fd) {
    const int flags = ::fcntl(fd, F_GETFL, 0);
    if (flags < 0) return -1;
    return ::fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

int port_from(const char* path) {
    if (path == nullptr || std::strncmp(path, "tcp:", 4) != 0) return kDefaultPort;
    const int port = std::atoi(path + 4);
    if (port <= 0 || port > 65535) return kDefaultPort;
    return port;
}

int write_client(void* ctx, const uint8_t* data, int n) {
    const int fd = *static_cast<const int*>(ctx);
    if (fd < 0 || data == nullptr || n < 0) return -1;
    int off = 0;
    while (off < n) {
        const ssize_t w = ::send(fd, data + off, static_cast<std::size_t>(n - off), MSG_NOSIGNAL);
        if (w < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        off += static_cast<int>(w);
    }
    return off;
}

void drop_client() {
    if (client_fd_ >= 0) ::close(client_fd_);
    client_fd_ = -1;
    dongle_link_detach();
}

void accept_client() {
    if (listen_fd_ < 0 || client_fd_ >= 0) return;
    const int fd = ::accept(listen_fd_, nullptr, nullptr);
    if (fd < 0) return;
    const int yes = 1;
    ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof yes);
    if (set_nonblock(fd) != 0) {
        ::close(fd);
        return;
    }
    client_fd_ = fd;
    dongle_link_attach(write_client, &client_fd_);
    dongle_link_set_paired(true);
    std::printf("wifi tablet connected\n");
    std::fflush(stdout);
}

void beacon_close() {
    if (beacon_fd_ >= 0) ::close(beacon_fd_);
    beacon_fd_ = -1;
}

bool beacon_open() {
    beacon_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (beacon_fd_ < 0) {
        std::fprintf(stderr, "wifi beacon: socket: %s\n", std::strerror(errno));
        return false;
    }
    const int yes = 1;
    ::setsockopt(beacon_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(kBeaconPort));
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(beacon_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 ||
        set_nonblock(beacon_fd_) != 0) {
        std::fprintf(stderr, "wifi beacon: bind %d: %s\n", kBeaconPort, std::strerror(errno));
        beacon_close();
        return false;
    }
    std::printf("wifi beacon 0.0.0.0:%d\n", kBeaconPort);
    std::fflush(stdout);
    return true;
}

void beacon_poll() {
    if (beacon_fd_ < 0) return;
    for (;;) {
        char buf[32];
        sockaddr_in from{};
        socklen_t len = sizeof from;
        const ssize_t n = ::recvfrom(beacon_fd_, buf, sizeof buf, 0,
                                      reinterpret_cast<sockaddr*>(&from), &len);
        if (n < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) return;
            std::fprintf(stderr, "wifi beacon read: %s\n", std::strerror(errno));
            return;
        }
        if (n == 6 && std::memcmp(buf, kBeaconFind, 6) == 0) {
            ::sendto(beacon_fd_, kBeaconHere, 6, MSG_NOSIGNAL,
                     reinterpret_cast<sockaddr*>(&from), len);
        }
    }
}

}  // namespace

bool dongle_open(const char* path) {
    const int port = port_from(path);
    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        std::fprintf(stderr, "wifi: socket: %s\n", std::strerror(errno));
        return false;
    }
    const int yes = 1;
    ::setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof yes);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0) {
        std::fprintf(stderr, "wifi: bind %d: %s\n", port, std::strerror(errno));
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    if (::listen(listen_fd_, 1) != 0 || set_nonblock(listen_fd_) != 0) {
        std::fprintf(stderr, "wifi: listen: %s\n", std::strerror(errno));
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    if (!beacon_open()) {
        ::close(listen_fd_);
        listen_fd_ = -1;
        return false;
    }
    std::printf("wifi listening 0.0.0.0:%d\n", port);
    std::fflush(stdout);
    return true;
}

void dongle_close() {
    drop_client();
    beacon_close();
    if (listen_fd_ >= 0) ::close(listen_fd_);
    listen_fd_ = -1;
}

void dongle_poll() {
    beacon_poll();
    accept_client();
    if (client_fd_ < 0) return;
    uint8_t buf[256];
    const ssize_t n = ::recv(client_fd_, buf, sizeof buf, 0);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return;
        std::fprintf(stderr, "wifi read: %s\n", std::strerror(errno));
        drop_client();
        return;
    }
    if (n == 0) {
        std::printf("wifi tablet disconnected\n");
        std::fflush(stdout);
        drop_client();
        return;
    }
    dongle_link_feed(std::span<const uint8_t>(buf, static_cast<std::size_t>(n)));
}

}  // namespace ccu
