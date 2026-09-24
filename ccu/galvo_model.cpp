#include "galvo_model.hpp"

#include <cmath>

namespace ccu {
namespace {

uint16_t clamp_dac(double counts) {
    if (counts <= static_cast<double>(kGalvoDacMin)) return kGalvoDacMin;
    if (counts >= static_cast<double>(kGalvoDacMax)) return kGalvoDacMax;
    return static_cast<uint16_t>(std::lround(counts));
}

double counts_from_angle(float radians) {
    return static_cast<double>(radians) / static_cast<double>(kGalvoRadPerDac) +
           static_cast<double>(kGalvoDacBoresight);
}

}  // namespace

GalvoAngle galvo_dac_to_angle(uint16_t dac_x, uint16_t dac_y) {
    const float x = (static_cast<float>(dac_x) - static_cast<float>(kGalvoDacBoresight)) * kGalvoRadPerDac;
    const float y = (static_cast<float>(dac_y) - static_cast<float>(kGalvoDacBoresight)) * kGalvoRadPerDac;
    return {x, y};
}

GalvoDac galvo_angle_to_dac(float azimuth, float elevation) {
    return {
        clamp_dac(counts_from_angle(azimuth)),
        clamp_dac(counts_from_angle(elevation)),
    };
}

GalvoAngle galvo_point_to_angle(float x, float y, float z) {
    return {
        std::atan2(y, x),
        std::atan2(z, std::hypot(x, y)),
    };
}

}  // namespace ccu
