#include "scan/PointCloud.hpp"

PointCloud::PointCloud(std::vector<LidarPoint> pts) : points_(std::move(pts)) {}

void PointCloud::clear() { points_.clear(); }

void PointCloud::reserve(std::size_t n) { points_.reserve(n); }

void PointCloud::add(const LidarPoint& p) { points_.push_back(p); }

void PointCloud::add_from_control(const ControlPayload& p) {
    points_.push_back(lidar_point_from_control(p));
}

void PointCloud::append_from_control(const std::vector<ControlPayload>& samples) {
    points_.reserve(points_.size() + samples.size());
    for (const auto& p : samples)
        points_.push_back(lidar_point_from_control(p));
}

std::size_t PointCloud::size() const { return points_.size(); }

bool PointCloud::empty() const { return points_.empty(); }

void PointCloud::recompute_cartesian() {
    for (auto& p : points_) {
        mechanical_to_boresight(p.mechanical_x, p.mechanical_y, p.azimuth, p.elevation);
        spherical_to_cartesian(p.range, p.azimuth, p.elevation, p.x, p.y, p.z);
    }
}
