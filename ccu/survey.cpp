#include "survey.hpp"

#include "bearing.hpp"
#include "device_config.hpp"
#include "lidar_model.hpp"
#include "session.hpp"
#include "utils/cmd_refs.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

namespace ccu {
namespace {

constexpr float kMotorXMin = 2.5f;
constexpr float kMotorXMax = 4.2f;
constexpr float kMotorYMin = 2.6f;
constexpr float kMotorYMax = 3.9f;

enum class Phase {
    Idle,
    Begin,
    AwaitEnable,
    AwaitScan,
    AwaitLow,
    Sweep,
    AwaitReturn,
    AwaitCoarse,
    Dwell,
};

struct Sample {
    float motor_x = 0.f;
    float motor_y = 0.f;
    float plan_east = 0.f;
    float plan_north = 0.f;
    float plan_up = 0.f;
    float horiz = 0.f;
};

std::vector<PlanNode> nodes_;
std::vector<SurveyNode> stored_;
std::size_t index_ = 0;
bool active_ = false;
bool motors_on_ = false;
bool have_status_ = false;
bool logged_wait_ = false;
Phase phase_ = Phase::Idle;
float frozen_yaw_ = 0.f;
StatusPayload status_{};
LidarMotor low_{};
LidarMotor high_{};
LidarMotor back_{};
Sample best_{};
bool have_best_ = false;
std::vector<Sample> dwell_;
std::chrono::steady_clock::time_point move_start_{};
std::chrono::steady_clock::time_point dwell_start_{};

bool motor_ok(float x, float y) {
    return x >= kMotorXMin && x <= kMotorXMax && y >= kMotorYMin && y <= kMotorYMax;
}

float limited_amp(float centre, float amp, float lo, float hi) {
    const float below = centre - lo;
    const float above = hi - centre;
    float room = below < above ? below : above;
    if (room < 0.f) room = 0.f;
    if (amp > room) amp = room;
    if (amp < 0.f) return 0.f;
    return amp;
}

bool arrived(const ControlPayload& sample, float x, float y) {
    const float tol = arrive_tol_rad();
    return std::fabs(sample.mechanical_angle_x - x) <= tol &&
           std::fabs(sample.mechanical_angle_y - y) <= tol;
}

bool move_expired() {
    const auto limit = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(move_timeout_s()));
    return std::chrono::steady_clock::now() - move_start_ >= limit;
}

bool dwell_expired() {
    const auto limit = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(dwell_time_s()));
    return std::chrono::steady_clock::now() - dwell_start_ >= limit;
}

void disable_motors() {
    if (!motors_on_) return;
    lidar_cmd(kCmdDisableEnable);
    motors_on_ = false;
}

void reset_job() {
    nodes_.clear();
    index_ = 0;
    active_ = false;
    motors_on_ = false;
    have_status_ = false;
    logged_wait_ = false;
    phase_ = Phase::Idle;
    frozen_yaw_ = 0.f;
    status_ = {};
    have_best_ = false;
    dwell_.clear();
}

SurveyResult from_node(const PlanNode& node) {
    SurveyResult result;
    result.id = node.id;
    result.east_m = node.east_m;
    result.north_m = node.north_m;
    result.corner_inner_angle_rad = node.corner_inner_angle_rad;
    result.corner_orientation_rad = node.corner_orientation_rad;
    return result;
}

void remember(const SurveyNode& node) {
    for (SurveyNode& kept : stored_) {
        if (kept.id == node.id) {
            kept = node;
            return;
        }
    }
    stored_.push_back(node);
}

SurveyResult finish(SurveyResult result) {
    ++index_;
    result.survey_finished = index_ >= nodes_.size();
    if (result.survey_finished) {
        active_ = false;
        phase_ = Phase::Idle;
        disable_motors();
    } else {
        phase_ = Phase::Begin;
        have_best_ = false;
        dwell_.clear();
    }
    return result;
}

SurveyResult fail_node(SurveyError error) {
    SurveyResult result = from_node(nodes_[index_]);
    result.up_m = 0.f;
    result.surveyed = false;
    result.success = false;
    result.error_code = error;
    return finish(result);
}

bool sample_from_control(const ControlPayload& control, Sample& out) {
    if (control.distance == 0) return false;
    const float distance_m = static_cast<float>(control.distance) * 0.01f;
    const LidarAngle device = lidar_motor_to_device(control.associated_angle_x, control.associated_angle_y);
    const std::optional<LidarPoint> plan =
        lidar_plan_point(device.azimuth, device.elevation, distance_m, frozen_yaw_);
    if (!plan) return false;
    const PlanNode& node = nodes_[index_];
    out.motor_x = control.associated_angle_x;
    out.motor_y = control.associated_angle_y;
    out.plan_east = plan->east_m;
    out.plan_north = plan->north_m;
    out.plan_up = plan->up_m;
    out.horiz = std::hypot(plan->east_m - node.east_m, plan->north_m - node.north_m);
    return true;
}

bool solve3(double m[3][4], double out[3]) {
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        for (int row = col + 1; row < 3; ++row) {
            if (std::fabs(m[row][col]) > std::fabs(m[pivot][col])) pivot = row;
        }
        if (std::fabs(m[pivot][col]) < 1e-8) return false;
        if (pivot != col) {
            for (int c = col; c < 4; ++c) std::swap(m[col][c], m[pivot][c]);
        }
        const double div = m[col][col];
        for (int c = col; c < 4; ++c) m[col][c] /= div;
        for (int row = 0; row < 3; ++row) {
            if (row == col) continue;
            const double factor = m[row][col];
            for (int c = col; c < 4; ++c) m[row][c] -= factor * m[col][c];
        }
    }
    out[0] = m[0][3];
    out[1] = m[1][3];
    out[2] = m[2][3];
    return true;
}

std::optional<SurveyResult> fit_node() {
    if (dwell_.size() < 3 || !have_best_) return fail_node(SurveyError::NotVisible);
    const float e0 = best_.plan_east;
    const float n0 = best_.plan_north;
    double xx = 0, xy = 0, yy = 0, x = 0, y = 0, n = 0, xz = 0, yz = 0, z = 0;
    for (const Sample& sample : dwell_) {
        const double dx = sample.plan_east - e0;
        const double dy = sample.plan_north - n0;
        const double dz = sample.plan_up;
        xx += dx * dx;
        xy += dx * dy;
        yy += dy * dy;
        x += dx;
        y += dy;
        n += 1.0;
        xz += dx * dz;
        yz += dy * dz;
        z += dz;
    }
    double rows[3][4] = {
        {xx, xy, x, xz},
        {xy, yy, y, yz},
        {x, y, n, z},
    };
    double solved[3];
    if (!solve3(rows, solved)) return fail_node(SurveyError::NotVisible);
    const float a = static_cast<float>(solved[0]);
    const float b = static_cast<float>(solved[1]);
    const float c = static_cast<float>(solved[2]);
    float ne = -a;
    float nn = -b;
    float nu = 1.f;
    const float norm = std::sqrt(ne * ne + nn * nn + nu * nu);
    if (!(norm > 0.f)) return fail_node(SurveyError::NotVisible);
    ne /= norm;
    nn /= norm;
    nu /= norm;
    if (nu < 0.f) {
        ne = -ne;
        nn = -nn;
        nu = -nu;
    }
    if (nu < 0.05f) return fail_node(SurveyError::NotVisible);

    const PlanNode& node = nodes_[index_];
    SurveyNode kept;
    kept.id = node.id;
    kept.east_m = node.east_m;
    kept.north_m = node.north_m;
    kept.up_m = a * (node.east_m - e0) + b * (node.north_m - n0) + c;
    kept.corner_inner_angle_rad = node.corner_inner_angle_rad;
    kept.corner_orientation_rad = node.corner_orientation_rad;
    kept.normal_east = ne;
    kept.normal_north = nn;
    kept.normal_up = nu;
    remember(kept);

    SurveyResult result = from_node(node);
    result.up_m = kept.up_m;
    result.surveyed = true;
    result.success = true;
    result.error_code = SurveyError::None;
    return finish(result);
}

void send_coarse(float centre_x, float centre_y) {
    lidar_cmd_f32(kCmdXAmplitudeSet, limited_amp(centre_x, coarse_amp_x_rad(), kMotorXMin, kMotorXMax));
    lidar_cmd_f32(kCmdYAmplitudeSet, limited_amp(centre_y, coarse_amp_y_rad(), kMotorYMin, kMotorYMax));
    lidar_cmd_f32(kCmdXFrequencySet, coarse_freq_x_hz());
    lidar_cmd_f32(kCmdYFrequencySet, coarse_freq_y_hz());
    lidar_cmd_f32(kCmdXPhaseSet, 0.f);
    lidar_cmd_f32(kCmdYPhaseSet, 0.f);
    lidar_cmd(kCmdBeginCoarseScanning);
}

std::optional<SurveyResult> on_begin() {
    if (index_ >= nodes_.size()) {
        active_ = false;
        phase_ = Phase::Idle;
        return std::nullopt;
    }
    const PlanNode& node = nodes_[index_];
    if (!in_fov(node.east_m, node.north_m, frozen_yaw_)) {
        const float dist = std::hypot(node.east_m, node.north_m);
        const float azimuth = device_azimuth(node.east_m, node.north_m, frozen_yaw_);
        std::printf(
            "survey out_of_fov id=%u east=%.3f north=%.3f dist=%.2f yaw=%.3f az=%.3f "
            "angle_max=%.2f d_max=%.2f\n",
            static_cast<unsigned>(node.id), static_cast<double>(node.east_m),
            static_cast<double>(node.north_m), static_cast<double>(dist),
            static_cast<double>(frozen_yaw_), static_cast<double>(azimuth),
            static_cast<double>(angle_max_rad()), static_cast<double>(d_max_m()));
        std::fflush(stdout);
        return fail_node(SurveyError::OutOfFov);
    }
    const float azimuth = device_azimuth(node.east_m, node.north_m, frozen_yaw_);
    low_ = lidar_device_to_motor(azimuth, -angle_max_rad());
    high_ = lidar_device_to_motor(azimuth, angle_max_rad());
    if (!motor_ok(low_.x, low_.y) || !motor_ok(high_.x, high_.y)) return fail_node(SurveyError::NotVisible);
    if (!have_status_) {
        if (!logged_wait_) {
            std::printf("survey waiting for lidar status\n");
            std::fflush(stdout);
            logged_wait_ = true;
        }
        return std::nullopt;
    }
    std::printf("survey status enabled=%u scan=%u\n",
                static_cast<unsigned>(status_.enabled),
                static_cast<unsigned>(status_.scan_mode));
    std::fflush(stdout);
    if (!status_.enabled) {
        std::printf("survey cmd 0x01 enable\n");
        std::fflush(stdout);
        lidar_cmd(kCmdDisableEnable);
        motors_on_ = true;
        phase_ = Phase::AwaitEnable;
        return std::nullopt;
    }
    std::printf("survey cmd 0x2b scan on\n");
    std::fflush(stdout);
    lidar_cmd(kCmdScanEngineOn);
    phase_ = Phase::AwaitScan;
    return std::nullopt;
}

std::optional<SurveyResult> on_dwell(const std::vector<ControlPayload>& controls) {
    for (const ControlPayload& control : controls) {
        Sample sample;
        if (sample_from_control(control, sample)) dwell_.push_back(sample);
    }
    if (!dwell_expired()) return std::nullopt;
    lidar_cmd(kCmdStopCoarseScanning);
    return fit_node();
}

}  // namespace

std::vector<SurveyResult> abort_remaining() {
    std::vector<SurveyResult> out;
    if (!active_) return out;
    for (std::size_t i = index_; i < nodes_.size(); ++i) {
        SurveyResult result = from_node(nodes_[i]);
        result.up_m = 0.f;
        result.surveyed = false;
        result.success = false;
        result.error_code = SurveyError::Aborted;
        result.survey_finished = i + 1 == nodes_.size();
        out.push_back(result);
    }
    disable_motors();
    active_ = false;
    phase_ = Phase::Idle;
    return out;
}

void survey_reset() {
    reset_job();
    stored_.clear();
}

void survey_begin(const std::vector<PlanNode>& nodes) {
    reset_job();
    nodes_ = nodes;
    frozen_yaw_ = session_effective_yaw();
    active_ = !nodes_.empty();
    phase_ = active_ ? Phase::Begin : Phase::Idle;
    if (!nodes_.empty()) {
        const PlanNode& node = nodes_.front();
        std::printf("survey begin count=%zu id=%u east=%.3f north=%.3f yaw=%.3f\n",
                    nodes_.size(), static_cast<unsigned>(node.id),
                    static_cast<double>(node.east_m), static_cast<double>(node.north_m),
                    static_cast<double>(frozen_yaw_));
        std::fflush(stdout);
    }
}

std::optional<SurveyResult> survey_poll() {
    if (!active_) return std::nullopt;
    lidar_poll();
    StatusPayload status{};
    while (lidar_take_status(status)) {
        status_ = status;
        have_status_ = true;
    }
    const std::vector<ControlPayload> controls = lidar_take_controls();
    const ControlPayload* latest = controls.empty() ? nullptr : &controls.back();

    switch (phase_) {
    case Phase::Begin:
        return on_begin();
    case Phase::AwaitEnable:
        if (have_status_ && status_.enabled) {
            std::printf("survey cmd 0x2b scan on\n");
            std::fflush(stdout);
            lidar_cmd(kCmdScanEngineOn);
            phase_ = Phase::AwaitScan;
        }
        return std::nullopt;
    case Phase::AwaitScan:
        if (have_status_ && status_.scan_mode == kScanModePoint2Point) {
            std::printf("survey cmd 0x2d low %.3f %.3f\n",
                        static_cast<double>(low_.x), static_cast<double>(low_.y));
            std::fflush(stdout);
            lidar_cmd_f32_pair(kCmdSetPoint2PointTarget, low_.x, low_.y);
            move_start_ = std::chrono::steady_clock::now();
            phase_ = Phase::AwaitLow;
        }
        return std::nullopt;
    case Phase::AwaitLow:
        if (latest && arrived(*latest, low_.x, low_.y)) {
            lidar_cmd_f32_pair(kCmdSetPoint2PointTarget, high_.x, high_.y);
            move_start_ = std::chrono::steady_clock::now();
            phase_ = Phase::Sweep;
            have_best_ = false;
        } else if (move_expired()) {
            return fail_node(SurveyError::NotVisible);
        }
        return std::nullopt;
    case Phase::Sweep:
        for (const ControlPayload& control : controls) {
            Sample sample;
            if (!sample_from_control(control, sample)) continue;
            if (!have_best_ || sample.horiz < best_.horiz) {
                best_ = sample;
                have_best_ = true;
            }
        }
        if (latest && arrived(*latest, high_.x, high_.y)) {
            if (!have_best_ || best_.horiz > survey_tol_m()) return fail_node(SurveyError::NotVisible);
            back_ = {best_.motor_x, best_.motor_y};
            lidar_cmd_f32_pair(kCmdSetPoint2PointTarget, back_.x, back_.y);
            move_start_ = std::chrono::steady_clock::now();
            phase_ = Phase::AwaitReturn;
        } else if (move_expired()) {
            return fail_node(SurveyError::NotVisible);
        }
        return std::nullopt;
    case Phase::AwaitReturn:
        if (latest && arrived(*latest, back_.x, back_.y)) {
            send_coarse(back_.x, back_.y);
            phase_ = Phase::AwaitCoarse;
        } else if (move_expired()) {
            return fail_node(SurveyError::NotVisible);
        }
        return std::nullopt;
    case Phase::AwaitCoarse:
        if (!(have_status_ && status_.scan_mode == kScanModeCoarse && status_.dynamic_routine == kRoutineNone))
            return std::nullopt;
        dwell_.clear();
        dwell_start_ = std::chrono::steady_clock::now();
        phase_ = Phase::Dwell;
        return on_dwell(controls);
    case Phase::Dwell:
        return on_dwell(controls);
    case Phase::Idle:
        return std::nullopt;
    }
    return std::nullopt;
}

std::vector<SurveyResult> survey_abort() { return abort_remaining(); }

std::optional<SurveyResult> survey_result(uint16_t id) {
    for (const SurveyNode& node : stored_) {
        if (node.id != id) continue;
        SurveyResult result;
        result.id = node.id;
        result.east_m = node.east_m;
        result.north_m = node.north_m;
        result.up_m = node.up_m;
        result.corner_inner_angle_rad = node.corner_inner_angle_rad;
        result.corner_orientation_rad = node.corner_orientation_rad;
        result.surveyed = true;
        result.success = true;
        result.error_code = SurveyError::None;
        return result;
    }
    return std::nullopt;
}

std::optional<SurveyNode> survey_node(uint16_t id) {
    for (const SurveyNode& node : stored_) {
        if (node.id == id) return node;
    }
    return std::nullopt;
}

}  // namespace ccu
