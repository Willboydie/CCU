#include "device_config.hpp"

#include <cctype>
#include <charconv>
#include <cmath>
#include <fstream>
#include <string>

namespace ccu {
namespace {

struct Values {
    float device_height_m = kDeviceHeightDefaultM;
    float d_max_m = kDMaxDefaultM;
    float angle_max_rad = kAngleMaxDefaultRad;
    float survey_tol_m = kSurveyTolDefaultM;
    float base_path_density = kBasePathDensityDefault;
    float corner_density = kCornerDensityDefault;
    float corner_len_m = kCornerLenDefaultM;
    float galvo_mech_offset_x = kGalvoMechOffsetXDefault;
    float galvo_mech_offset_y = kGalvoMechOffsetYDefault;
    float galvo_rad_per_dac = kGalvoRadPerDacDefault;
    float lidar_angle_offset_x = kLidarAngleOffsetXDefault;
    float lidar_angle_offset_y = kLidarAngleOffsetYDefault;
    float lidar_east_offset_m = kLidarEastOffsetDefault;
    float lidar_north_offset_m = kLidarNorthOffsetDefault;
    float lidar_up_offset_m = kLidarUpOffsetDefault;
    float dwell_time_s = kDwellTimeDefaultS;
    float coarse_amp_x_rad = kCoarseAmpXDefaultRad;
    float coarse_amp_y_rad = kCoarseAmpYDefaultRad;
    float coarse_freq_x_hz = kCoarseFreqXDefaultHz;
    float coarse_freq_y_hz = kCoarseFreqYDefaultHz;
    float arrive_tol_rad = kArriveTolDefaultRad;
    float move_timeout_s = kMoveTimeoutDefaultS;
};

Values cfg_;

std::string trim(std::string s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    std::size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    return s.substr(i);
}

bool parse_finite(const std::string& text, float& out) {
    const char* begin = text.data();
    const char* end = begin + text.size();
    float value = 0.f;
    const std::from_chars_result parsed = std::from_chars(begin, end, value);
    if (parsed.ec != std::errc{} || parsed.ptr != end) return false;
    if (!std::isfinite(value)) return false;
    out = value;
    return true;
}

enum class Bound { Any, NonNegative, Positive };

struct Key {
    const char* name;
    float Values::* slot;
    float fallback;
    Bound bound;
};

const Key kKeys[] = {
    {"DEVICE_HEIGHT", &Values::device_height_m, kDeviceHeightDefaultM, Bound::Positive},
    {"D_MAX", &Values::d_max_m, kDMaxDefaultM, Bound::Positive},
    {"ANGLE_MAX", &Values::angle_max_rad, kAngleMaxDefaultRad, Bound::Positive},
    {"SURVEY_TOL", &Values::survey_tol_m, kSurveyTolDefaultM, Bound::Positive},
    {"BASE_PATH_DENSITY", &Values::base_path_density, kBasePathDensityDefault, Bound::Positive},
    {"CORNER_DENSITY", &Values::corner_density, kCornerDensityDefault, Bound::Positive},
    {"CORNER_LEN", &Values::corner_len_m, kCornerLenDefaultM, Bound::Positive},
    {"GALVO_MECH_OFFSET_X", &Values::galvo_mech_offset_x, kGalvoMechOffsetXDefault, Bound::Any},
    {"GALVO_MECH_OFFSET_Y", &Values::galvo_mech_offset_y, kGalvoMechOffsetYDefault, Bound::Any},
    {"GALVO_RAD_PER_DAC", &Values::galvo_rad_per_dac, kGalvoRadPerDacDefault, Bound::Positive},
    {"LIDAR_ANGLE_OFFSETX", &Values::lidar_angle_offset_x, kLidarAngleOffsetXDefault, Bound::Any},
    {"LIDAR_ANGLE_OFFSETY", &Values::lidar_angle_offset_y, kLidarAngleOffsetYDefault, Bound::Any},
    {"LIDAR_EAST_OFFSET", &Values::lidar_east_offset_m, kLidarEastOffsetDefault, Bound::Any},
    {"LIDAR_NORTH_OFFSET", &Values::lidar_north_offset_m, kLidarNorthOffsetDefault, Bound::Any},
    {"LIDAR_UP_OFFSET", &Values::lidar_up_offset_m, kLidarUpOffsetDefault, Bound::Any},
    {"DWELL_TIME_S", &Values::dwell_time_s, kDwellTimeDefaultS, Bound::NonNegative},
    {"COARSE_AMP_X_RAD", &Values::coarse_amp_x_rad, kCoarseAmpXDefaultRad, Bound::NonNegative},
    {"COARSE_AMP_Y_RAD", &Values::coarse_amp_y_rad, kCoarseAmpYDefaultRad, Bound::NonNegative},
    {"COARSE_FREQ_X_HZ", &Values::coarse_freq_x_hz, kCoarseFreqXDefaultHz, Bound::NonNegative},
    {"COARSE_FREQ_Y_HZ", &Values::coarse_freq_y_hz, kCoarseFreqYDefaultHz, Bound::NonNegative},
    {"ARRIVE_TOL_RAD", &Values::arrive_tol_rad, kArriveTolDefaultRad, Bound::Positive},
    {"MOVE_TIMEOUT_S", &Values::move_timeout_s, kMoveTimeoutDefaultS, Bound::Positive},
};

bool in_bound(float value, Bound bound) {
    if (bound == Bound::Positive) return value > 0.f;
    if (bound == Bound::NonNegative) return value >= 0.f;
    return true;
}

}  // namespace

float device_height_m() { return cfg_.device_height_m; }
float d_max_m() { return cfg_.d_max_m; }
float angle_max_rad() { return cfg_.angle_max_rad; }
float survey_tol_m() { return cfg_.survey_tol_m; }
float base_path_density() { return cfg_.base_path_density; }
float corner_density() { return cfg_.corner_density; }
float corner_len_m() { return cfg_.corner_len_m; }
float galvo_mech_offset_x() { return cfg_.galvo_mech_offset_x; }
float galvo_mech_offset_y() { return cfg_.galvo_mech_offset_y; }
float galvo_rad_per_dac() { return cfg_.galvo_rad_per_dac; }
float lidar_angle_offset_x() { return cfg_.lidar_angle_offset_x; }
float lidar_angle_offset_y() { return cfg_.lidar_angle_offset_y; }
float lidar_east_offset_m() { return cfg_.lidar_east_offset_m; }
float lidar_north_offset_m() { return cfg_.lidar_north_offset_m; }
float lidar_up_offset_m() { return cfg_.lidar_up_offset_m; }
float dwell_time_s() { return cfg_.dwell_time_s; }
float coarse_amp_x_rad() { return cfg_.coarse_amp_x_rad; }
float coarse_amp_y_rad() { return cfg_.coarse_amp_y_rad; }
float coarse_freq_x_hz() { return cfg_.coarse_freq_x_hz; }
float coarse_freq_y_hz() { return cfg_.coarse_freq_y_hz; }
float arrive_tol_rad() { return cfg_.arrive_tol_rad; }
float move_timeout_s() { return cfg_.move_timeout_s; }

float plan_up_m(float range_m, float elevation_rad) {
    return device_height_m() + range_m * std::sin(elevation_rad);
}

float device_up_m(float plan_up) { return plan_up - device_height_m(); }

void device_config_reset() { cfg_ = {}; }

DeviceConfigStatus device_config_load(const char* path) {
    cfg_ = {};
    DeviceConfigStatus status;
    std::ifstream in(path);
    if (!in) return status;
    status.opened = true;

    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = trim(line.substr(0, eq));
        const std::string value = trim(line.substr(eq + 1));
        const Key* found = nullptr;
        for (const Key& item : kKeys) {
            if (key == item.name) {
                found = &item;
                break;
            }
        }
        if (!found) continue;
        float parsed = 0.f;
        if (!parse_finite(value, parsed) || !in_bound(parsed, found->bound)) {
            cfg_.*(found->slot) = found->fallback;
            if (key == "DEVICE_HEIGHT") status.height_set = false;
            continue;
        }
        cfg_.*(found->slot) = parsed;
        if (key == "DEVICE_HEIGHT") status.height_set = true;
    }
    return status;
}

}  // namespace ccu
