#pragma once

#include <cstdint>
#include <vector>

namespace ccu {

struct PathNode {
    float east_m = 0.f;
    float north_m = 0.f;
    float up_m = 0.f;
    float corner_inner_angle_rad = 0.f;
    float corner_orientation_rad = 0.f;
};

struct CartPoint {
    float east_m = 0.f;
    float north_m = 0.f;
    float up_m = 0.f;
};

struct DacPoint {
    uint16_t x = 0;
    uint16_t y = 0;
    bool laser_on = false;
};

// Laser-on arm geometry. The vertex is the single middle point. Up is the stored plan up.
std::vector<CartPoint> path_corner_points(const PathNode& node);

// DAC list. Laser on for corner points. Laser off for every join, including the close.
std::vector<DacPoint> path_engine(const std::vector<PathNode>& nodes, float yaw_rad);

}  // namespace ccu
