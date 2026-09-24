#pragma once

namespace ccu {

inline constexpr float kFloorUpMm = -150.f;

struct LidarAngle {
    float azimuth = 0.f;
    float elevation = 0.f;
};

struct LidarOrigin {
    float x_mm = 0.f;
    float y_mm = 0.f;
    float z_mm = 0.f;
};

LidarAngle lidar_motor_to_angle(float mech_x, float mech_y);
float lidar_up_mm(float range_mm, float elevation);
LidarOrigin lidar_origin_mm();

}  // namespace ccu
