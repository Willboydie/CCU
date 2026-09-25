#include "bearing.hpp"

#include "device_config.hpp"
#include "yaw.hpp"

#include <cmath>

namespace ccu {
namespace {

float wrap_pi(float radians) {
    constexpr float kPi = 3.14159265358979323846f;
    float wrapped = wrap_2pi(radians);
    if (wrapped > kPi) wrapped -= 2.f * kPi;
    return wrapped;
}

}  // namespace

float bearing_from_north(float east_m, float north_m) {
    return wrap_2pi(std::atan2(east_m, north_m));
}

float device_azimuth(float east_m, float north_m, float effective_yaw) {
    return wrap_pi(bearing_from_north(east_m, north_m) - effective_yaw);
}

bool in_fov(float east_m, float north_m, float effective_yaw) {
    if (std::hypot(east_m, north_m) > d_max_m()) return false;
    const float azimuth = device_azimuth(east_m, north_m, effective_yaw);
    return std::fabs(azimuth) <= angle_max_rad();
}

}  // namespace ccu
