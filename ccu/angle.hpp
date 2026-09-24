#pragma once

namespace ccu {

struct AngleSample {
    float stage = 0.f;
    float roll = 0.f;
    float pitch = 0.f;
    float imu_yaw = 0.f;
    long long dt_ms = 0;
};

bool angle_open(const char* path);
void angle_close();
void angle_poll();
bool angle_take(AngleSample& out);

}  // namespace ccu
