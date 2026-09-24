#include "project.hpp"

#include "bearing.hpp"
#include "session.hpp"
#include "survey.hpp"

namespace ccu {
namespace {

constexpr uint8_t kNotSurveyed = 4;
constexpr uint8_t kOutOfFov = 2;

std::vector<PathPoint> path_;
bool running_ = false;

}  // namespace

ProjectionReady project_begin(const std::vector<PlanNode>& nodes) {
    ProjectionReady ready;
    std::vector<AimPoint> aims;
    const float yaw = session_effective_yaw();
    for (const PlanNode& node : nodes) {
        const std::optional<SurveyResult> stored = survey_result(node.id);
        if (!stored) {
            ready.errors.push_back({node.id, kNotSurveyed});
            continue;
        }
        if (!in_fov(node.east_mm, node.north_mm, yaw)) {
            ready.errors.push_back({node.id, kOutOfFov});
            continue;
        }
        ready.nodes.push_back(node);
        aims.push_back({node.east_mm, node.north_mm, stored->up_mm});
    }
    path_ = path_engine(aims, kLaserDutyDefault);
    running_ = !path_.empty();
    return ready;
}

const std::vector<PathPoint>& project_path() { return path_; }

void project_poll() {}

void project_pause() { running_ = false; }

void project_play() {
    if (!path_.empty()) running_ = true;
}

void project_end() {
    path_.clear();
    running_ = false;
}

}  // namespace ccu
