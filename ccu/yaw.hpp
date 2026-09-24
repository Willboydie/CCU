#pragma once

namespace ccu {

// Radians into [0, 2π).
float wrap_2pi(float radians);

// Clockwise from due north. Stage angle increases anticlockwise.
float effective_yaw(float north_offset, float stage);

}  // namespace ccu
