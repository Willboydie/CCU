#pragma once

namespace ccu {

// Clockwise from due north, in [0, 2π). Distances are metres.
float bearing_from_north(float east_m, float north_m);

// Signed azimuth of a plan point relative to effective yaw, in (-π, π].
float device_azimuth(float east_m, float north_m, float effective_yaw);

// Inside ±ANGLE_MAX of effective yaw and at most D_MAX from the origin.
bool in_fov(float east_m, float north_m, float effective_yaw);

}  // namespace ccu
