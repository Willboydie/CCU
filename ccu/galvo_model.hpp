#pragma once

#include <cstdint>

namespace ccu {

inline constexpr float kGalvoRadPerDac = 1.0e-4f;
inline constexpr uint16_t kGalvoDacBoresight = 32768;
inline constexpr uint16_t kGalvoDacMin = 0;
inline constexpr uint16_t kGalvoDacMax = 65535;

struct GalvoAngle {
    float azimuth = 0.f;
    float elevation = 0.f;
};

struct GalvoDac {
    uint16_t x = kGalvoDacBoresight;
    uint16_t y = kGalvoDacBoresight;
};

GalvoAngle galvo_dac_to_angle(uint16_t dac_x, uint16_t dac_y);
GalvoDac galvo_angle_to_dac(float azimuth, float elevation);

// Device-centre XYZ. Mirror offset is zero. Azimuth is atan2(y, x),
// elevation is atan2(z, horizontal range).
GalvoAngle galvo_point_to_angle(float x, float y, float z);

}  // namespace ccu
