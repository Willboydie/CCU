#pragma once

namespace ccu {

inline constexpr float kDeviceHeightDefaultM = 1.365f;
inline constexpr float kDMaxDefaultM = 20.f;
inline constexpr float kAngleMaxDefaultRad = 0.40f;
inline constexpr float kSurveyTolDefaultM = 0.10f;
inline constexpr float kBasePathDensityDefault = 50.f;
inline constexpr float kCornerDensityDefault = 400.f;
inline constexpr float kCornerLenDefaultM = 0.20f;
inline constexpr float kGalvoMechOffsetXDefault = 0.1283f;
inline constexpr float kGalvoMechOffsetYDefault = 0.0136f;
inline constexpr float kGalvoRadPerDacDefault = 0.0015466f;
inline constexpr float kLidarAngleOffsetXDefault = 3.540f;
inline constexpr float kLidarAngleOffsetYDefault = 3.390f;
inline constexpr float kLidarEastOffsetDefault = 0.020f;
inline constexpr float kLidarNorthOffsetDefault = 0.f;
inline constexpr float kLidarUpOffsetDefault = 0.062f;
inline constexpr float kDwellTimeDefaultS = 1.0f;
inline constexpr float kCoarseAmpXDefaultRad = 0.10f;
inline constexpr float kCoarseAmpYDefaultRad = 0.10f;
inline constexpr float kCoarseFreqXDefaultHz = 2.f;
inline constexpr float kCoarseFreqYDefaultHz = 4.f;
inline constexpr float kArriveTolDefaultRad = 0.02f;
inline constexpr float kMoveTimeoutDefaultS = 8.f;

struct DeviceConfigStatus {
    bool opened = false;
    bool height_set = false;
};

float device_height_m();
float d_max_m();
float angle_max_rad();
float survey_tol_m();
float base_path_density();
float corner_density();
float corner_len_m();
float galvo_mech_offset_x();
float galvo_mech_offset_y();
float galvo_rad_per_dac();
float lidar_angle_offset_x();
float lidar_angle_offset_y();
float lidar_east_offset_m();
float lidar_north_offset_m();
float lidar_up_offset_m();
float dwell_time_s();
float coarse_amp_x_rad();
float coarse_amp_y_rad();
float coarse_freq_x_hz();
float coarse_freq_y_hz();
float arrive_tol_rad();
float move_timeout_s();

// Plan up of a return. range_m and the result are metres. No position offset.
float plan_up_m(float range_m, float elevation_rad);

// Device-centre up of a plan point. East and north are already device-centre.
float device_up_m(float plan_up_m);

void device_config_reset();

// Read ccu_survey.cfg. The program does not write it. A missing file keeps every
// default. A missing or bad key keeps that key's default.
DeviceConfigStatus device_config_load(const char* path);

}  // namespace ccu
