#pragma once

namespace ccu {

struct AngleSample {
    float stage = 0.f;
    float roll = 0.f;
    float pitch = 0.f;
    float imu_yaw = 0.f;
    long long dt_ms = 0;
};

// baud 115200, RTS clear, and flush_rx false is the open that delivered bytes.
// The default matches the orientation firmware rate and flushes the host buffer.
bool angle_open(const char* path, int baud = 921600, bool flush_rx = true);
void angle_close();
void angle_poll();
bool angle_take(AngleSample& out);

}  // namespace ccu
