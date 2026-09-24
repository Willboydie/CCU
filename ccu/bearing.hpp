#pragma once

namespace ccu {

inline constexpr float kFovHalfRad = 0.6108652381980153f;  // 35 degrees
inline constexpr float kFovRangeMm = 13000.f;

// Clockwise from due north, in [0, 2π).
float bearing_from_north(float east_mm, float north_mm);

// Signed azimuth of a plan point relative to effective yaw, in (-π, π].
float device_azimuth(float east_mm, float north_mm, float effective_yaw);

// Inside ±35° of effective yaw and at most 13 m from the origin.
bool in_fov(float east_mm, float north_mm, float effective_yaw);

}  // namespace ccu
