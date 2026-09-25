#include "path_engine.hpp"

#include "device_config.hpp"
#include "galvo_model.hpp"

#include <cmath>
#include <vector>

namespace ccu {
namespace {

int arm_count(float length, float density_vertex, float density_end) {
    if (!(length > 0.f)) return 2;
    const double n = static_cast<double>(length) *
                     (static_cast<double>(density_vertex) + static_cast<double>(density_end)) / 2.0;
    int count = static_cast<int>(std::lround(n));
    if (count < 2) count = 2;
    return count;
}

float distance_at(int index, int count, float length, float density_vertex, float density_end) {
    if (count <= 1 || !(length > 0.f)) return 0.f;
    const double total = static_cast<double>(length) *
                         (static_cast<double>(density_vertex) + static_cast<double>(density_end)) / 2.0;
    const double target = total * static_cast<double>(index) / static_cast<double>(count - 1);
    const double a = (static_cast<double>(density_end) - static_cast<double>(density_vertex)) / (2.0 * length);
    const double b = density_vertex;
    if (std::fabs(a) < 1e-12) {
        if (b == 0.0) return 0.f;
        return static_cast<float>(target / b);
    }
    const double disc = b * b + 4.0 * a * target;
    const double root = std::sqrt(disc > 0.0 ? disc : 0.0);
    return static_cast<float>((-b + root) / (2.0 * a));
}

CartPoint point_along(const PathNode& node, float bearing, float distance) {
    return {
        node.east_m + std::sin(bearing) * distance,
        node.north_m + std::cos(bearing) * distance,
        node.up_m,
    };
}

struct Step {
    double x = 0;
    double y = 0;
    bool found = false;
};

Step outgoing_step(const std::vector<DacPoint>& corner) {
    if (corner.size() < 2) return {};
    for (std::size_t i = corner.size() - 1; i > 0; --i) {
        const double dx = static_cast<int>(corner[i].x) - static_cast<int>(corner[i - 1].x);
        const double dy = static_cast<int>(corner[i].y) - static_cast<int>(corner[i - 1].y);
        if (dx != 0.0 || dy != 0.0) return {dx, dy, true};
    }
    return {};
}

Step incoming_step(const std::vector<DacPoint>& corner) {
    if (corner.size() < 2) return {};
    for (std::size_t i = 1; i < corner.size(); ++i) {
        const double dx = static_cast<int>(corner[i].x) - static_cast<int>(corner[i - 1].x);
        const double dy = static_cast<int>(corner[i].y) - static_cast<int>(corner[i - 1].y);
        if (dx != 0.0 || dy != 0.0) return {dx, dy, true};
    }
    return {};
}

struct Vec2 {
    double x = 0;
    double y = 0;
};

Vec2 hermite(double t, Vec2 p0, Vec2 m0, Vec2 p1, Vec2 m1) {
    const double t2 = t * t;
    const double t3 = t2 * t;
    const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    const double h10 = t3 - 2.0 * t2 + t;
    const double h01 = -2.0 * t3 + 3.0 * t2;
    const double h11 = t3 - t2;
    return {
        h00 * p0.x + h10 * m0.x + h01 * p1.x + h11 * m1.x,
        h00 * p0.y + h10 * m0.y + h01 * p1.y + h11 * m1.y,
    };
}

uint16_t clamp_round(double value) {
    const long rounded = std::lround(value);
    if (rounded <= 0) return 0;
    if (rounded >= 65535L) return 65535;
    return static_cast<uint16_t>(rounded);
}

void append_join(std::vector<DacPoint>& path, const std::vector<DacPoint>& from, const std::vector<DacPoint>& to) {
    if (from.empty() || to.empty()) return;
    const Step out = outgoing_step(from);
    const Step in = incoming_step(to);
    if (!out.found || !in.found) {
        path.push_back({from.back().x, from.back().y, false});
        path.push_back({to.front().x, to.front().y, false});
        return;
    }

    const Vec2 p0{static_cast<double>(from.back().x), static_cast<double>(from.back().y)};
    const Vec2 p1{static_cast<double>(to.front().x), static_cast<double>(to.front().y)};
    const Vec2 m0{out.x, out.y};
    const Vec2 m1{in.x, in.y};
    constexpr int kSamples = 64;
    double cum[kSamples + 1];
    Vec2 poly[kSamples + 1];
    poly[0] = hermite(0.0, p0, m0, p1, m1);
    cum[0] = 0.0;
    for (int i = 1; i <= kSamples; ++i) {
        poly[i] = hermite(static_cast<double>(i) / kSamples, p0, m0, p1, m1);
        const double dx = poly[i].x - poly[i - 1].x;
        const double dy = poly[i].y - poly[i - 1].y;
        cum[i] = cum[i - 1] + std::hypot(dx, dy);
    }
    const double length = cum[kSamples];
    const double step = std::hypot(m0.x, m0.y);
    if (!(step > 0.0) || !(length > step)) return;

    for (double dist = step; dist < length - 1e-3; dist += step) {
        int i = 0;
        while (i < kSamples - 1 && cum[i + 1] < dist) ++i;
        const double span = cum[i + 1] - cum[i];
        const double u = span > 1e-12 ? (dist - cum[i]) / span : 0.0;
        const double x = poly[i].x + (poly[i + 1].x - poly[i].x) * u;
        const double y = poly[i].y + (poly[i + 1].y - poly[i].y) * u;
        const uint16_t dx = clamp_round(x);
        const uint16_t dy = clamp_round(y);
        if (dx == from.back().x && dy == from.back().y) continue;
        if (dx == to.front().x && dy == to.front().y) continue;
        path.push_back({dx, dy, false});
    }
}

std::vector<DacPoint> corner_dac(const PathNode& node, float yaw_rad) {
    const std::vector<CartPoint> cart = path_corner_points(node);
    std::vector<DacPoint> dac;
    dac.reserve(cart.size());
    for (const CartPoint& point : cart) {
        const GalvoDac sample = galvo_plan_to_dac(point.east_m, point.north_m, point.up_m, yaw_rad);
        dac.push_back({sample.x, sample.y, true});
    }
    return dac;
}

}  // namespace

std::vector<CartPoint> path_corner_points(const PathNode& node) {
    const float length = corner_len_m();
    const float density_vertex = corner_density();
    const float density_end = base_path_density();
    const int count = arm_count(length, density_vertex, density_end);
    std::vector<CartPoint> points;
    points.reserve(static_cast<std::size_t>(count * 2 - 1));
    for (int i = count - 1; i >= 0; --i) {
        const float distance = distance_at(i, count, length, density_vertex, density_end);
        points.push_back(point_along(node, node.corner_orientation_rad, distance));
    }
    const float exit = node.corner_orientation_rad + node.corner_inner_angle_rad;
    for (int i = 1; i < count; ++i) {
        const float distance = distance_at(i, count, length, density_vertex, density_end);
        points.push_back(point_along(node, exit, distance));
    }
    return points;
}

std::vector<DacPoint> path_engine(const std::vector<PathNode>& nodes, float yaw_rad) {
    std::vector<DacPoint> path;
    if (nodes.empty()) return path;
    std::vector<std::vector<DacPoint>> corners;
    corners.reserve(nodes.size());
    for (const PathNode& node : nodes) corners.push_back(corner_dac(node, yaw_rad));
    for (std::size_t i = 0; i < corners.size(); ++i) {
        path.insert(path.end(), corners[i].begin(), corners[i].end());
        append_join(path, corners[i], corners[(i + 1) % corners.size()]);
    }
    return path;
}

}  // namespace ccu
