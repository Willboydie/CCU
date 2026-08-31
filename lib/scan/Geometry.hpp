#pragma once

#include <cmath>

// Mechanical frame: encoder/motor radians, same limits as HOST_PROTOCOL §6.
inline constexpr float kMechXMin = 2.5f;
inline constexpr float kMechXMax = 4.2f;
inline constexpr float kMechYMin = 2.6f;
inline constexpr float kMechYMax = 3.9f;
inline constexpr float kMechNeutralX = 3.54f;
inline constexpr float kMechNeutralY = 3.39f;
inline constexpr float kMechAmpMax = 0.6f;

// Angular pixel size for strip search (notes: 10 mrad bins).
inline constexpr float kPixelBinRad = 0.010f;

// LDM ControlPayload.distance is centimetres.
inline constexpr float kCmToMetres = 0.01f;

// Simplified pointing map (placeholder). Motor (x, y) is treated as a
// boresight spherical pair offset from the mechanical neutral:
//   theta (azimuth)   = mechanical_x - kMechNeutralX
//   phi   (elevation) = mechanical_y - kMechNeutralY
// Origin, pointing straight ahead [1, 0, 0], is (theta = 0, phi = 0):
//   x = r * cos(theta) * cos(phi)
//   y = r * sin(theta) * cos(phi)
//   z = r * sin(phi)
// A later map can replace these without changing callers.

inline void mechanical_to_boresight(float mech_x, float mech_y,
                                    float& azimuth, float& elevation) {
    azimuth = mech_x - kMechNeutralX;
    elevation = mech_y - kMechNeutralY;
}

inline void boresight_to_mechanical(float azimuth, float elevation,
                                    float& mech_x, float& mech_y) {
    mech_x = azimuth + kMechNeutralX;
    mech_y = elevation + kMechNeutralY;
}

inline void spherical_to_cartesian(float range, float azimuth, float elevation,
                                   float& x, float& y, float& z) {
    const float cphi = std::cos(elevation);
    x = range * std::cos(azimuth) * cphi;
    y = range * std::sin(azimuth) * cphi;
    z = range * std::sin(elevation);
}

inline float azimuth_of_xy(float x, float y) {
    return std::atan2(y, x);
}

inline float horizontal_range_xy(float x, float y) {
    return std::hypot(x, y);
}

// Measured range that would place a return at horizontal distance |xy|
// for a given elevation. Notes wrote r = cos(phi) * |xy|; the spherical
// map above implies |xy| = r * cos(phi), so r = |xy| / cos(phi).
inline float r_target_at_elevation(float horiz_range, float elevation) {
    const float cphi = std::cos(elevation);
    if (std::fabs(cphi) < 1e-6f)
        return 0.0f;
    return horiz_range / cphi;
}
