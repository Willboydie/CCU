#include "galvo_model.hpp"

#include "device_config.hpp"

#include <cmath>

namespace ccu {
namespace {

constexpr float kPi = 3.14159265358979323846f;

float arccotangent(float n) {
    if (n == 0.f) return kPi / 2.f;
    return std::atan2(1.f, n);
}

}  // namespace

GalvoMirrors galvo_mirror_angles(float north, float east, float up) {
    const float h = kGalvoMirrorSeparationM;
    const float north_den = std::fabs(north) < 1e-9f ? std::copysign(1e-9f, north == 0.f ? 1.f : north) : north;
    const float y = arccotangent((h - up) / north_den) - kPi / 2.f;
    const float along = std::sin(kPi / 2.f + y);
    const float den = north + h * along;
    const float den_safe = std::fabs(den) < 1e-12f ? std::copysign(1e-12f, den == 0.f ? 1.f : den) : den;
    const float x = (arccotangent((east * along) / den_safe) - kPi / 2.f) / -1.f;
    return {y, x};
}

uint16_t galvo_dac_from_mirror(float mirror_rad) {
    const float scale = galvo_rad_per_dac();
    if (!(scale > 0.f) || !std::isfinite(mirror_rad)) return 0;
    const double quot = static_cast<double>(mirror_rad) / static_cast<double>(scale);
    if (quot >= 40000.0) return 65535;
    if (quot <= -40000.0) return 0;
    const long step = static_cast<long>(quot);
    const long counts = 32768L + step;
    if (counts <= 0) return 0;
    if (counts >= 65535L) return 65535;
    return static_cast<uint16_t>(counts);
}

GalvoDac galvo_plan_to_dac(float plan_east, float plan_north, float plan_up, float yaw_rad) {
    float north = plan_east * std::sin(yaw_rad) + plan_north * std::cos(yaw_rad);
    float east = plan_east * std::cos(yaw_rad) - plan_north * std::sin(yaw_rad);
    float up = plan_up;
    north += lidar_north_offset_m();
    east += lidar_east_offset_m();
    up += -device_height_m() + lidar_up_offset_m();

    const float ox = galvo_mech_offset_x();
    const float oy = galvo_mech_offset_y();
    const float east1 = east * std::cos(ox) - north * std::sin(ox);
    const float north1 = east * std::sin(ox) + north * std::cos(ox);
    const float up1 = up;
    const float east2 = east1 * std::cos(oy) + up1 * std::sin(oy);
    const float north2 = north1;
    const float up2 = -east1 * std::sin(oy) + up1 * std::cos(oy);

    const GalvoMirrors mirrors = galvo_mirror_angles(north2, east2, up2);
    return {galvo_dac_from_mirror(mirrors.x_rad), galvo_dac_from_mirror(mirrors.y_rad)};
}

}  // namespace ccu
