#pragma once

#include "Geometry.hpp"

#include "../utils/protocol.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

struct LidarPoint {
    float range = 0;          // metres
    float azimuth = 0;        // theta, rad, boresight
    float elevation = 0;      // phi, rad, boresight
    float x = 0;
    float y = 0;
    float z = 0;
    float mechanical_x = 0;   // device motor angles, rad
    float mechanical_y = 0;
    uint16_t distance_cm = 0; // raw LDM
    bool valid = false;       // false when distance is 0 (no return)
};

inline LidarPoint lidar_point_from_control(const ControlPayload& p) {
    LidarPoint out;
    out.distance_cm = p.distance;
    out.range = static_cast<float>(p.distance) * kCmToMetres;
    out.mechanical_x = p.associated_angle_x;
    out.mechanical_y = p.associated_angle_y;
    out.valid = p.distance != 0;
    mechanical_to_boresight(out.mechanical_x, out.mechanical_y,
                            out.azimuth, out.elevation);
    spherical_to_cartesian(out.range, out.azimuth, out.elevation,
                           out.x, out.y, out.z);
    return out;
}

// Container for general point-cloud data. Cartesian fields are filled
// with the simplified spherical map in Geometry.hpp.
class PointCloud {
public:
    PointCloud() = default;
    explicit PointCloud(std::vector<LidarPoint> pts);

    void clear();
    void reserve(std::size_t n);
    void add(const LidarPoint& p);
    void add_from_control(const ControlPayload& p);
    void append_from_control(const std::vector<ControlPayload>& samples);

    std::size_t size() const;
    bool empty() const;

    const std::vector<LidarPoint>& points() const { return points_; }

    const LidarPoint& operator[](std::size_t i) const { return points_[i]; }

    void recompute_cartesian();

protected:
    std::vector<LidarPoint> points_;
};
