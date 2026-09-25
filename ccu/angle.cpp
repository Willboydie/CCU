#include "angle.hpp"

#include "serial.hpp"
#include "session.hpp"

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace ccu {
namespace {

constexpr uint8_t kCmdOrientation = 0xA0;
constexpr int kPayload = 16;

enum class Phase { Cmd, LenLo, LenHi, Body };

int fd_ = -1;
Phase phase_ = Phase::Cmd;
uint16_t length_ = 0;
int have_ = 0;
uint8_t frame_[3 + kPayload]{};
bool have_last_ = false;
std::chrono::steady_clock::time_point last_frame_{};

constexpr int kQueueCap = 64;
AngleSample queue_[kQueueCap]{};
int q_head_ = 0;
int q_count_ = 0;

void push_sample(const AngleSample& sample) {
    if (q_count_ == kQueueCap) {
        q_head_ = (q_head_ + 1) % kQueueCap;
        --q_count_;
    }
    const int tail = (q_head_ + q_count_) % kQueueCap;
    queue_[tail] = sample;
    ++q_count_;
}

void clear_queue() {
    q_head_ = 0;
    q_count_ = 0;
}

float read_f32(int offset) {
    float value = 0.f;
    std::memcpy(&value, frame_ + 3 + offset, sizeof value);
    return value;
}

void reset_parser() {
    phase_ = Phase::Cmd;
    length_ = 0;
    have_ = 0;
}

void on_frame() {
    if (length_ != kPayload) return;
    const float roll = read_f32(0);
    const float pitch = read_f32(4);
    const float imu_yaw = read_f32(8);
    const float stage = read_f32(12);
    session_set_stage(stage);

    const auto now = std::chrono::steady_clock::now();
    long long dt_ms = 0;
    if (have_last_) {
        dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_frame_).count();
    }
    have_last_ = true;
    last_frame_ = now;
    AngleSample sample;
    sample.stage = stage;
    sample.roll = roll;
    sample.pitch = pitch;
    sample.imu_yaw = imu_yaw;
    sample.dt_ms = dt_ms;
    push_sample(sample);
}

void feed(uint8_t b) {
    switch (phase_) {
    case Phase::Cmd:
        if (b != kCmdOrientation) return;
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
        if (length_ != kPayload) {
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

bool angle_open(const char* path, int baud, bool flush_rx) {
    fd_ = serial_open(path, baud, false, flush_rx);
    if (fd_ < 0) {
        std::fprintf(stderr, "angle: %s: %s\n", path, std::strerror(errno));
        return false;
    }
    reset_parser();
    clear_queue();
    have_last_ = false;
    std::printf("angle opened %s\n", path);
    std::fflush(stdout);
    return true;
}

void angle_close() { serial_close(fd_); fd_ = -1; }

void angle_poll() {
    if (fd_ < 0) return;
    uint8_t buf[256];
    const int n = serial_read(fd_, buf, static_cast<int>(sizeof buf));
    if (n <= 0) return;
    for (int i = 0; i < n; ++i) feed(buf[i]);
}

bool angle_take(AngleSample& out) {
    if (q_count_ == 0) return false;
    out = queue_[q_head_];
    q_head_ = (q_head_ + 1) % kQueueCap;
    --q_count_;
    return true;
}

}  // namespace ccu
