#pragma once

#include <cstdint>
#include <vector>

namespace ccu {

inline constexpr float kCornerSizeMm = 40.f;

struct AimPoint {
    float east_mm = 0.f;
    float north_mm = 0.f;
    float up_mm = 0.f;
};

struct PathPoint {
    float east_mm = 0.f;
    float north_mm = 0.f;
    float up_mm = 0.f;
    bool laser_on = false;
    uint8_t duty = 0;
};

// For each aim: one laser-off connector from the previous aim, then four
// laser-on corners of a 40 mm square ending back on the aim. No slope.
std::vector<PathPoint> path_engine(const std::vector<AimPoint>& aims, uint8_t duty);

}  // namespace ccu
