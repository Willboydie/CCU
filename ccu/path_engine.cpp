#include "path_engine.hpp"

namespace ccu {

std::vector<PathPoint> path_engine(const std::vector<AimPoint>& aims, uint8_t duty) {
    std::vector<PathPoint> path;
    if (aims.empty()) return path;
    path.reserve(aims.size() * 5);
    for (std::size_t i = 0; i < aims.size(); ++i) {
        const AimPoint& aim = aims[i];
        const AimPoint& from = i == 0 ? aim : aims[i - 1];
        path.push_back({from.east_mm, from.north_mm, from.up_mm, false, 0});
        path.push_back({aim.east_mm + kCornerSizeMm, aim.north_mm, aim.up_mm, true, duty});
        path.push_back({aim.east_mm + kCornerSizeMm, aim.north_mm + kCornerSizeMm, aim.up_mm, true, duty});
        path.push_back({aim.east_mm, aim.north_mm + kCornerSizeMm, aim.up_mm, true, duty});
        path.push_back({aim.east_mm, aim.north_mm, aim.up_mm, true, duty});
    }
    return path;
}

}  // namespace ccu
