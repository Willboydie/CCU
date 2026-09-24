#include "lidar.hpp"

namespace ccu {

void lidar_set_aim(float /*azimuth*/, float /*elevation*/) {}

std::optional<float> lidar_read_range() { return std::nullopt; }

}  // namespace ccu
