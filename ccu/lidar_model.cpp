#include "lidar_model.hpp"

#include "device_config.hpp"

#include <cmath>

namespace ccu {

LidarAngle lidar_motor_to_device(float motor_x, float motor_y) {
    return {motor_x - lidar_angle_offset_x(), motor_y - lidar_angle_offset_y()};
}

LidarMotor lidar_device_to_motor(float azimuth, float elevation) {
    return {azimuth + lidar_angle_offset_x(), elevation + lidar_angle_offset_y()};
}

std::optional<LidarPoint> lidar_device_point(float azimuth, float elevation, float distance_m) {
    if (!(distance_m > 0.f) || !std::isfinite(distance_m)) return std::nullopt;
    const float ce = std::cos(elevation);
    LidarPoint point;
    point.north_m = distance_m * std::cos(azimuth) * ce;
    point.east_m = distance_m * std::sin(azimuth) * ce;
    point.up_m = distance_m * std::sin(elevation);
    return point;
}

std::optional<LidarPoint> lidar_plan_point(float azimuth, float elevation, float distance_m, float yaw_rad) {
    const std::optional<LidarPoint> device = lidar_device_point(azimuth, elevation, distance_m);
    if (!device) return std::nullopt;
    LidarPoint plan;
    plan.east_m = device->north_m * std::sin(yaw_rad) + device->east_m * std::cos(yaw_rad);
    plan.north_m = device->north_m * std::cos(yaw_rad) - device->east_m * std::sin(yaw_rad);
    plan.up_m = device_height_m() + device->up_m;
    return plan;
}

}  // namespace ccu
