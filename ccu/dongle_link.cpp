#include "dongle_link.hpp"

#include "protocol.hpp"
#include "session.hpp"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace ccu {
namespace {

constexpr uint8_t kDongleStatus = 0xE6;
constexpr int kMaxPayload = 1024;

enum class Phase { Cmd, LenLo, LenHi, Body };

DongleWriteFn write_ = nullptr;
void* ctx_ = nullptr;
bool paired_ = false;
Phase phase_ = Phase::Cmd;
uint16_t length_ = 0;
int have_ = 0;
uint8_t frame_[3 + kMaxPayload]{};
std::chrono::steady_clock::time_point last_id_{};

void reset_parser() {
    phase_ = Phase::Cmd;
    length_ = 0;
    have_ = 0;
}

bool inbound_cmd(uint8_t b) {
    if (b == kDongleStatus || b == kCmdConnected) return true;
    return b >= kCmdBeginNewSession && b <= kCmdTagAligned;
}

void on_frame() {
    const uint8_t cmd = frame_[0];
    const uint8_t* payload = frame_ + 3;
    if (cmd == kDongleStatus) {
        std::printf("dongle \"%.*s\"\n", static_cast<int>(length_),
                    reinterpret_cast<const char*>(payload));
        std::fflush(stdout);
        if (length_ == 5 && std::memcmp(payload, "ready", 5) == 0) paired_ = false;
        if (length_ == 6 && std::memcmp(payload, "paired", 6) == 0) paired_ = true;
        return;
    }
    if (!paired_) return;
    session_on_frame(std::span<const uint8_t>(frame_, static_cast<std::size_t>(3 + length_)));
}

void feed_byte(uint8_t b) {
    switch (phase_) {
    case Phase::Cmd:
        if (!inbound_cmd(b)) return;
        frame_[0] = b;
        phase_ = Phase::LenLo;
        break;
    case Phase::LenLo:
        frame_[1] = b;
        phase_ = Phase::LenHi;
        break;
    case Phase::LenHi:
        frame_[2] = b;
        length_ = static_cast<uint16_t>(frame_[1] | (static_cast<uint16_t>(frame_[2]) << 8));
        if (length_ > kMaxPayload) {
            reset_parser();
            break;
        }
        if (length_ == 0) {
            on_frame();
            reset_parser();
            break;
        }
        have_ = 0;
        phase_ = Phase::Body;
        break;
    case Phase::Body:
        frame_[3 + have_] = b;
        ++have_;
        if (have_ == static_cast<int>(length_)) {
            on_frame();
            reset_parser();
        }
        break;
    }
}

}  // namespace

void dongle_link_attach(DongleWriteFn write, void* ctx) {
    write_ = write;
    ctx_ = ctx;
    paired_ = false;
    last_id_ = {};
    reset_parser();
}

void dongle_link_detach() {
    write_ = nullptr;
    ctx_ = nullptr;
    paired_ = false;
    reset_parser();
}

void dongle_link_feed(std::span<const uint8_t> bytes) {
    if (!write_) return;
    for (uint8_t b : bytes) feed_byte(b);
}

bool dongle_link_paired() { return paired_; }

void dongle_link_set_paired(bool paired) { paired_ = paired; }

bool dongle_link_write(const uint8_t* data, int n) {
    if (!paired_ || !write_ || data == nullptr || n < 0) return false;
    return write_(ctx_, data, n) == n;
}

void dongle_send_device_id_if_due() {
    if (!paired_ || !write_) return;
    const auto now = std::chrono::steady_clock::now();
    if (now - last_id_ < std::chrono::milliseconds(500)) return;

    DeviceId msg;
    msg.device_id = "sm-ccu";
    msg.display_name = "Smart Marker";
    msg.session_active = session_running();
    msg.session_id = session_id();
    const auto bytes = pack_device_id(msg);
    if (!bytes) return;
    if (write_(ctx_, bytes->data(), static_cast<int>(bytes->size())) < 0) return;
    last_id_ = now;
    std::printf("device_id name=\"%s\" session_active=%d\n",
                msg.display_name.c_str(), msg.session_active ? 1 : 0);
    std::fflush(stdout);
}

}  // namespace ccu
