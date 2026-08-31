#pragma once

#include "PointCloud.hpp"

#include <optional>
#include <string>

struct DwellParams {
    float amp_x = 0.1f;       // rad, Lissajous half-width about the pixel
    float amp_y = 0.1f;
    float freq_x = 2.0f;      // Hz; fy = 2 * fx is the infinity figure-8
    float freq_y = 4.0f;
    float phase_x = 0.0f;
    float phase_y = 0.0f;
    float duration_s = 1.0f;  // how long to collect after coarse scan is up
};

// Point cloud collected while dwelling a small Lissajous on one pixel.
// Owns Z extraction; estimate_z() is a stub until that method is defined.
//
// Binary file (little-endian), for offline analysis of raw module reports:
//   header 64 bytes
//     magic[4]     "WDL1"
//     version      u16 = 1
//     rec_size     u16 = 12
//     count        u32
//     target_x     f32   query XY (m)
//     target_y     f32
//     centre_az    f32   boresight (rad)
//     centre_el    f32
//     centre_mx    f32   mechanical centre (rad)
//     centre_my    f32
//     amp_x        f32
//     amp_y        f32
//     freq_x       f32
//     freq_y       f32
//     phase_x      f32
//     phase_y      f32
//     duration_s   f32
//   then `count` records of 12 bytes:
//     angle_x      f32   associated (LDM) angle, rad
//     angle_y      f32
//     distance_cm  u16   raw LDM
//     flags        u16   bit0 = valid (distance != 0)
class DwellPointCloud : public PointCloud {
public:
    DwellPointCloud() = default;

    void set_centre(float azimuth, float elevation);
    void set_centre_mechanical(float mx, float my);
    void set_target_xy(float x, float y);
    void set_params(const DwellParams& p);

    float centre_azimuth() const { return centre_az_; }
    float centre_elevation() const { return centre_el_; }
    float centre_mechanical_x() const { return centre_mx_; }
    float centre_mechanical_y() const { return centre_my_; }
    float target_x() const { return target_x_; }
    float target_y() const { return target_y_; }
    bool has_centre() const { return have_centre_; }
    const DwellParams& params() const { return params_; }

    // Returns nullopt until a Z estimator exists.
    std::optional<float> estimate_z() const;

    bool save_binary(const std::string& path, std::string& err) const;
    static bool load_binary(const std::string& path, DwellPointCloud& out, std::string& err);

private:
    float centre_az_ = 0;
    float centre_el_ = 0;
    float centre_mx_ = 0;
    float centre_my_ = 0;
    float target_x_ = 0;
    float target_y_ = 0;
    DwellParams params_{};
    bool have_centre_ = false;
};
