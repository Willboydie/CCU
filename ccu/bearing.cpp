#include "bearing.hpp"

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

float bearing_from_north(float east_mm, float north_mm) {
    return wrap_2pi(std::atan2(east_mm, north_mm));
}

float device_azimuth(float east_mm, float north_mm, float effective_yaw) {
    return wrap_pi(bearing_from_north(east_mm, north_mm) - effective_yaw);
}

bool in_fov(float east_mm, float north_mm, float effective_yaw) {
    if (std::hypot(east_mm, north_mm) > kFovRangeMm) return false;
    const float azimuth = device_azimuth(east_mm, north_mm, effective_yaw);
    return std::fabs(azimuth) <= kFovHalfRad;
}

}  // namespace ccu
