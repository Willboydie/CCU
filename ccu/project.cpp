#include "project.hpp"

#include "bearing.hpp"
#include "projector.hpp"
#include "protocol.hpp"
#include "session.hpp"
#include "survey.hpp"
#include "utils/projector_protocol.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace ccu {
namespace {

constexpr uint8_t kNotSurveyed = 4;
constexpr uint8_t kOutOfFov = 2;
constexpr std::size_t kStreamRecords = (1024 - 1) / 5;

std::vector<DacPoint> path_;
float frozen_yaw_ = 0.f;
bool primed_ = false;
bool paused_ = false;
std::size_t cursor_ = 0;
uint8_t packet_ = 1;
uint8_t duty_ = kLaserDutyDefault;

void enqueue(uint8_t cmd, const uint8_t* payload, int n) {
    std::vector<uint8_t> frame(static_cast<std::size_t>(3 + n));
    frame[0] = cmd;
    frame[1] = static_cast<uint8_t>(n & 0xFF);
    frame[2] = static_cast<uint8_t>((n >> 8) & 0xFF);
    if (n > 0 && payload) std::memcpy(frame.data() + 3, payload, static_cast<std::size_t>(n));
    projector_enqueue(std::move(frame));
}

void enqueue_chunk() {
    if (cursor_ >= path_.size()) {
        cursor_ = 0;
        packet_ = 1;
    }
    const std::size_t count = std::min(kStreamRecords, path_.size() - cursor_);
    if (count == 0) return;
    std::vector<uint8_t> payload(1 + count * 5);
    payload[0] = packet_;
    for (std::size_t i = 0; i < count; ++i) {
        const DacPoint& point = path_[cursor_ + i];
        uint8_t* rec = payload.data() + 1 + i * 5;
        rec[0] = static_cast<uint8_t>((point.x >> 8) & 0xFF);
        rec[1] = static_cast<uint8_t>(point.x & 0xFF);
        rec[2] = static_cast<uint8_t>((point.y >> 8) & 0xFF);
        rec[3] = static_cast<uint8_t>(point.y & 0xFF);
        rec[4] = point.laser_on ? 1 : 0;
    }
    enqueue(kProjCmdPointStream, payload.data(), static_cast<int>(payload.size()));
    cursor_ += count;
    packet_ = static_cast<uint8_t>(packet_ + 1);
}

}  // namespace

ProjectionReady project_begin(const std::vector<PlanNode>& nodes) {
    ProjectionReady ready;
    std::vector<PathNode> draw;
    frozen_yaw_ = session_effective_yaw();
    for (const PlanNode& node : nodes) {
        const std::optional<SurveyResult> stored = survey_result(node.id);
        if (!stored) {
            ready.errors.push_back({node.id, kNotSurveyed});
            continue;
        }
        if (!in_fov(node.east_m, node.north_m, frozen_yaw_)) {
            ready.errors.push_back({node.id, kOutOfFov});
            continue;
        }
        ready.nodes.push_back(node);
        draw.push_back({
            stored->east_m,
            stored->north_m,
            stored->up_m,
            stored->corner_inner_angle_rad,
            stored->corner_orientation_rad,
        });
    }
    path_ = path_engine(draw, frozen_yaw_);
    primed_ = false;
    paused_ = false;
    cursor_ = 0;
    packet_ = 1;
    return ready;
}

const std::vector<DacPoint>& project_path() { return path_; }

void project_poll() {
    if (path_.empty() || paused_) return;
    if (!primed_) {
        enqueue(kProjCmdResetStream, nullptr, 0);
        enqueue(kProjCmdSetLaserDuty, &duty_, 1);
        enqueue(kProjCmdPlay, nullptr, 0);
        primed_ = true;
        cursor_ = 0;
        packet_ = 1;
    }
    enqueue_chunk();
}

void project_pause() { paused_ = true; }

void project_play() {
    if (path_.empty()) return;
    paused_ = false;
    primed_ = false;
    cursor_ = 0;
    packet_ = 1;
}

void project_end() {
    if (primed_) enqueue(kProjCmdStop, nullptr, 0);
    path_.clear();
    primed_ = false;
    paused_ = false;
    cursor_ = 0;
}

}  // namespace ccu
