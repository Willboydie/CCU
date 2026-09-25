#pragma once

#include <cstdint>

namespace ccu {

// Mirror separation. A constant in the model, not a config key.
inline constexpr float kGalvoMirrorSeparationM = 0.015f;
inline constexpr uint16_t kGalvoDacCentre = 32768;

struct GalvoDac {
    uint16_t x = kGalvoDacCentre;
    uint16_t y = kGalvoDacCentre;
};

struct GalvoMirrors {
    float y_rad = 0.f;
    float x_rad = 0.f;
};

// Y mirror first. X mirror uses that angle. north/east/up are metres in the
// frame after the lidar-offset shift and the mount rotations.
GalvoMirrors galvo_mirror_angles(float north, float east, float up);

// trunc toward zero, then clamp into 0 … 65535.
uint16_t galvo_dac_from_mirror(float mirror_rad);

// Plan metres and clockwise yaw. Shifts by the lidar offsets, then mount rotations.
GalvoDac galvo_plan_to_dac(float plan_east, float plan_north, float plan_up, float yaw_rad);

}  // namespace ccu
