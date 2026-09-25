#include "lidar.hpp"

#include <cstring>
#include <vector>

namespace ccu {
namespace {

LidarRead read_ = nullptr;
LidarWrite write_ = nullptr;
std::vector<uint8_t> rx_;
std::vector<StatusPayload> statuses_;
std::vector<ControlPayload> controls_;
std::vector<std::vector<uint8_t>> commands_;

void drop_front(std::size_t n) {
    if (n >= rx_.size()) rx_.clear();
    else rx_.erase(rx_.begin(), rx_.begin() + static_cast<std::ptrdiff_t>(n));
}

void feed(const uint8_t* data, int n) {
    if (n <= 0) return;
    rx_.insert(rx_.end(), data, data + n);
    while (!rx_.empty()) {
        const uint8_t header = rx_[0];
        if (header == kControlHeader) {
            if (rx_.size() < kControlPacketSize) return;
            ControlPayload body{};
            std::memcpy(&body, rx_.data() + 3, sizeof(body));
            controls_.push_back(body);
            drop_front(kControlPacketSize);
        } else if (header == kStatusHeader) {
            if (rx_.size() < kStatusPacketSize) return;
            StatusPayload body{};
            std::memcpy(&body, rx_.data() + 3, sizeof(body));
            statuses_.push_back(body);
            drop_front(kStatusPacketSize);
        } else if (header == kDebugHeader) {
            if (rx_.size() < 3) return;
            const uint16_t len = static_cast<uint16_t>(rx_[1] | (static_cast<uint16_t>(rx_[2]) << 8));
            if (rx_.size() < static_cast<std::size_t>(3 + len)) return;
            drop_front(static_cast<std::size_t>(3 + len));
        } else {
            drop_front(1);
        }
    }
}

void send(uint8_t cmd, const uint8_t* payload, uint16_t len) {
    std::vector<uint8_t> frame(static_cast<std::size_t>(3 + len));
    frame[0] = cmd;
    frame[1] = static_cast<uint8_t>(len & 0xFF);
    frame[2] = static_cast<uint8_t>((len >> 8) & 0xFF);
    if (len && payload) std::memcpy(frame.data() + 3, payload, len);
    if (write_) write_(frame.data(), static_cast<int>(frame.size()));
    else commands_.push_back(std::move(frame));
}

}  // namespace

void lidar_bind_port(LidarRead read, LidarWrite write) {
    read_ = read;
    write_ = write;
}

void lidar_poll() {
    if (!read_) return;
    uint8_t buf[256];
    for (;;) {
        const int n = read_(buf, static_cast<int>(sizeof buf));
        if (n <= 0) break;
        feed(buf, n);
    }
}

void lidar_push_status(const StatusPayload& status) { statuses_.push_back(status); }

void lidar_push_control(const ControlPayload& control) { controls_.push_back(control); }

void lidar_test_reset() {
    rx_.clear();
    statuses_.clear();
    controls_.clear();
    commands_.clear();
}

bool lidar_take_status(StatusPayload& out) {
    if (statuses_.empty()) return false;
    out = statuses_.front();
    statuses_.erase(statuses_.begin());
    return true;
}

std::vector<ControlPayload> lidar_take_controls() {
    std::vector<ControlPayload> out;
    out.swap(controls_);
    return out;
}

void lidar_cmd(uint8_t cmd) { send(cmd, nullptr, 0); }

void lidar_cmd_f32(uint8_t cmd, float value) {
    uint8_t payload[4];
    std::memcpy(payload, &value, sizeof(value));
    send(cmd, payload, 4);
}

void lidar_cmd_f32_pair(uint8_t cmd, float x, float y) {
    uint8_t payload[8];
    std::memcpy(payload, &x, 4);
    std::memcpy(payload + 4, &y, 4);
    send(cmd, payload, 8);
}

std::vector<std::vector<uint8_t>> lidar_take_commands() {
    std::vector<std::vector<uint8_t>> out;
    out.swap(commands_);
    return out;
}

}  // namespace ccu
