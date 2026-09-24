#pragma once

#include <optional>

namespace ccu {

void lidar_set_aim(float azimuth, float elevation);
std::optional<float> lidar_read_range();
bool lidar_open(const char* path);
void lidar_close();

}  // namespace ccu
