#pragma once

#include <optional>

namespace ccu {

struct LidarAngle {
    float azimuth = 0.f;
    float elevation = 0.f;
};

struct LidarPoint {
    float east_m = 0.f;
    float north_m = 0.f;
    float up_m = 0.f;
};

struct LidarMotor {
    float x = 0.f;
    float y = 0.f;
};

LidarAngle lidar_motor_to_device(float motor_x, float motor_y);
LidarMotor lidar_device_to_motor(float azimuth, float elevation);

// Device-frame metres. No position offset. Distance 0 is ignored.
std::optional<LidarPoint> lidar_device_point(float azimuth, float elevation, float distance_m);

// Plan metres. Yaw is clockwise from due north. plan up is DEVICE_HEIGHT + up_d.
std::optional<LidarPoint> lidar_plan_point(float azimuth, float elevation, float distance_m, float yaw_rad);

}  // namespace ccu
