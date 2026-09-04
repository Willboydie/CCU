#pragma once

#include <cstdint>

// Packed records for projector slope-map calibration (.bin).
// Each sample is one 1 cm surface traverse at a rangefinder distance:
//   Δθ ≈ 0.01 / range_m   (radians, small-angle)
//   f'(start) = Δθ / ΔDAC  (radians per 16-bit DAC count)

inline constexpr uint8_t kCalibAxisX = 0;
inline constexpr uint8_t kCalibAxisY = 1;
inline constexpr float kCalibTraverseM = 0.01f;

#pragma pack(push, 1)
struct ProjectorCalibDatum {
    uint8_t axis;      // kCalibAxisX or kCalibAxisY
    uint16_t start;    // DAC at Begin measure
    float derivative;  // f'(start), rad / DAC count
};
#pragma pack(pop)

static_assert(sizeof(ProjectorCalibDatum) == 7);

inline bool calib_derivative(float range_m, int dac_delta, float& out) {
    if (!(range_m > 0.0f) || dac_delta == 0)
        return false;
    out = kCalibTraverseM / (range_m * static_cast<float>(dac_delta));
    return true;
}

inline const char* calib_axis_name(uint8_t axis) {
    switch (axis) {
        case kCalibAxisX: return "X";
        case kCalibAxisY: return "Y";
        default: return "?";
    }
}
