#include "survey.hpp"

#include "bearing.hpp"
#include "lidar_model.hpp"
#include "session.hpp"

#include <chrono>
#include <cmath>

namespace ccu {
namespace {

constexpr int kSurveySamples = 20;
constexpr std::chrono::milliseconds kSurveyDwell{200};

std::vector<PlanNode> nodes_;
std::size_t index_ = 0;
RangeRead read_ = lidar_read_range;
bool active_ = false;
int samples_ = 0;
std::optional<float> latest_;
float aim_elevation_ = 0.f;
std::chrono::steady_clock::time_point dwell_start_{};

enum class Step { Idle, Consider, Dwell };
Step step_ = Step::Idle;
std::vector<SurveyResult> stored_;

void remember(const SurveyResult& result) {
    if (!result.success) return;
    for (SurveyResult& kept : stored_) {
        if (kept.id == result.id) {
            kept = result;
            return;
        }
    }
    stored_.push_back(result);
}

void reset_job() {
    nodes_.clear();
    index_ = 0;
    read_ = lidar_read_range;
    active_ = false;
    samples_ = 0;
    latest_.reset();
    aim_elevation_ = 0.f;
    step_ = Step::Idle;
}

SurveyResult from_node(const PlanNode& node) {
    SurveyResult result;
    result.id = node.id;
    result.east_mm = node.east_mm;
    result.north_mm = node.north_mm;
    result.corner_inner_angle_rad = node.corner_inner_angle_rad;
    result.corner_orientation_rad = node.corner_orientation_rad;
    return result;
}

void finish_node(SurveyResult& result) {
    ++index_;
    result.survey_finished = index_ >= nodes_.size();
    if (result.survey_finished) {
        active_ = false;
        step_ = Step::Idle;
    } else {
        step_ = Step::Consider;
    }
}

std::optional<SurveyResult> dwell() {
    if (read_) {
        if (const std::optional<float> sample = read_()) {
            latest_ = sample;
            ++samples_;
        }
    }
    const bool timed_out = std::chrono::steady_clock::now() - dwell_start_ >= kSurveyDwell;
    if (samples_ < kSurveySamples && !timed_out) return std::nullopt;

    const PlanNode& node = nodes_[index_];
    SurveyResult result = from_node(node);
    if (!latest_) {
        result.up_mm = 0.f;
        result.surveyed = false;
        result.success = false;
        result.error_code = SurveyError::NotVisible;
    } else {
        result.up_mm = lidar_up_mm(*latest_, aim_elevation_);
        result.surveyed = true;
        result.success = true;
        result.error_code = SurveyError::None;
        remember(result);
    }
    finish_node(result);
    return result;
}

std::optional<SurveyResult> consider() {
    if (index_ >= nodes_.size()) {
        active_ = false;
        step_ = Step::Idle;
        return std::nullopt;
    }
    const PlanNode& node = nodes_[index_];
    const float yaw = session_effective_yaw();
    if (!in_fov(node.east_mm, node.north_mm, yaw)) {
        SurveyResult result = from_node(node);
        result.up_mm = 0.f;
        result.surveyed = false;
        result.success = false;
        result.error_code = SurveyError::OutOfFov;
        finish_node(result);
        return result;
    }

    const float horiz = std::hypot(node.east_mm, node.north_mm);
    aim_elevation_ = std::atan2(kFloorUpMm, horiz);
    lidar_set_aim(device_azimuth(node.east_mm, node.north_mm, yaw), aim_elevation_);
    samples_ = 0;
    latest_.reset();
    dwell_start_ = std::chrono::steady_clock::now();
    step_ = Step::Dwell;
    return dwell();
}

}  // namespace

std::vector<SurveyResult> abort_remaining() {
    std::vector<SurveyResult> out;
    if (!active_) return out;
    for (std::size_t i = index_; i < nodes_.size(); ++i) {
        SurveyResult result = from_node(nodes_[i]);
        result.up_mm = 0.f;
        result.surveyed = false;
        result.success = false;
        result.error_code = SurveyError::Aborted;
        result.survey_finished = i + 1 == nodes_.size();
        out.push_back(result);
    }
    active_ = false;
    step_ = Step::Idle;
    return out;
}

std::optional<SurveyResult> find_stored(uint16_t id) {
    for (const SurveyResult& result : stored_) {
        if (result.id == id && result.success) return result;
    }
    return std::nullopt;
}

void survey_reset() {
    reset_job();
    stored_.clear();
}

void survey_begin(const std::vector<PlanNode>& nodes, RangeRead read) {
    reset_job();
    nodes_ = nodes;
    read_ = read ? read : lidar_read_range;
    active_ = !nodes_.empty();
    step_ = active_ ? Step::Consider : Step::Idle;
}

std::optional<SurveyResult> survey_poll() {
    if (!active_) return std::nullopt;
    if (step_ == Step::Consider) return consider();
    if (step_ == Step::Dwell) return dwell();
    return std::nullopt;
}

std::vector<SurveyResult> survey_abort() { return abort_remaining(); }

std::optional<SurveyResult> survey_result(uint16_t id) { return find_stored(id); }

}  // namespace ccu
