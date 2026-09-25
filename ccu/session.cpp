#include "session.hpp"

#include "project.hpp"
#include "protocol.hpp"
#include "survey.hpp"
#include "yaw.hpp"

#include <chrono>
#include <cstdio>

namespace ccu {
namespace {

bool running_ = false;
bool pairing_unsent_ = false;
bool origin_is_set_ = false;
float stage_ = 0.f;
float north_offset_ = 0.f;
WorkMode work_ = WorkMode::Idle;
std::string session_id_;
uint32_t session_counter_ = 0;
std::vector<SurveyResult> taken_survey_;
std::optional<ProjectionReady> taken_ready_;
std::chrono::steady_clock::time_point last_connected_{};

constexpr auto kLinkSilence = std::chrono::seconds(5);

bool link_up_now() {
    if (!running_ || last_connected_.time_since_epoch().count() == 0) return false;
    return std::chrono::steady_clock::now() - last_connected_ < kLinkSilence;
}

void latch_link() { last_connected_ = std::chrono::steady_clock::now(); }

std::string make_session_id() {
    ++session_counter_;
    const auto ticks = static_cast<uint32_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    const uint32_t mixed = (session_counter_ * 0x9E3779B9u) ^ ticks;
    char buf[9];
    std::snprintf(buf, sizeof(buf), "%08x", mixed);
    return buf;
}

void clear_session() {
    if (work_ == WorkMode::Surveying) {
        const std::vector<SurveyResult> aborted = survey_abort();
        taken_survey_.insert(taken_survey_.end(), aborted.begin(), aborted.end());
    }
    if (work_ == WorkMode::Projecting) project_end();
    survey_reset();
    project_end();
    running_ = false;
    pairing_unsent_ = false;
    origin_is_set_ = false;
    north_offset_ = 0.f;
    work_ = WorkMode::Idle;
    session_id_.clear();
    last_connected_ = {};
}

void begin_new_session() {
    running_ = true;
    pairing_unsent_ = true;
    origin_is_set_ = false;
    north_offset_ = 0.f;
    work_ = WorkMode::Idle;
    session_id_ = make_session_id();
    latch_link();
}

void join_session(const std::string& id) {
    if (!running_ || link_up_now() || id != session_id_) return;
    pairing_unsent_ = true;
    latch_link();
}

void begin_origin_set() {
    north_offset_ = stage_;
    origin_is_set_ = true;
}

void end_session(const std::string& id) {
    if (id != session_id_) return;
    clear_session();
}

}  // namespace

void session_reset() {
    stage_ = 0.f;
    clear_session();
    taken_survey_.clear();
    taken_ready_.reset();
}

void session_set_stage(float stage_rad) { stage_ = stage_rad; }

bool session_running() { return running_; }

bool session_pairing_unsent() { return pairing_unsent_; }

void session_mark_pairing_sent() { pairing_unsent_ = false; }

bool session_origin_is_set() { return origin_is_set_; }

std::string session_id() { return session_id_; }

float session_stage() { return stage_; }

float session_north_offset() { return north_offset_; }

float session_effective_yaw() { return effective_yaw(north_offset_, stage_); }

WorkMode session_work() { return work_; }

AcceptMode session_mode() { return {running_, work_, origin_is_set_}; }

bool session_link_up() { return link_up_now(); }

void session_expire_link() {
    if (last_connected_.time_since_epoch().count() == 0) return;
    last_connected_ -= std::chrono::seconds(6);
}

void session_on_frame(std::span<const uint8_t> frame) {
    const auto raw = unpack_frame(frame);
    if (!raw) return;
    // JOIN is dropped while the link is up. A dropped link keeps the session,
    // and a matching id latches the link again without clearing origin.
    if (raw->cmd == kCmdJoinSession && running_ && !link_up_now()) {
        if (const auto id = unpack_join_session(frame)) join_session(*id);
        return;
    }
    if (!command_allowed(session_mode(), raw->cmd)) return;

    switch (raw->cmd) {
    case kCmdConnected:
        if (unpack_connected(frame) && link_up_now()) latch_link();
        break;
    case kCmdBeginNewSession:
        if (unpack_begin_new_session(frame)) begin_new_session();
        break;
    case kCmdBeginOriginSet:
        if (unpack_begin_origin_set(frame)) begin_origin_set();
        break;
    case kCmdEndSession:
        if (const auto id = unpack_end_session(frame)) end_session(*id);
        break;
    case kCmdPerformSurvey:
        if (const auto nodes = unpack_perform_survey(frame)) {
            work_ = WorkMode::Surveying;
            survey_begin(*nodes);
        }
        break;
    case kCmdAbortSurvey:
        for (const SurveyResult& result : survey_abort()) session_queue_survey_result(result);
        work_ = WorkMode::Idle;
        break;
    case kCmdPerformProjection:
        if (const auto nodes = unpack_perform_projection(frame)) {
            work_ = WorkMode::Projecting;
            taken_ready_ = project_begin(*nodes);
        }
        break;
    case kCmdProjectorPause:
        project_pause();
        break;
    case kCmdProjectorPlay:
        project_play();
        break;
    case kCmdProjectionEnd:
        project_end();
        work_ = WorkMode::Idle;
        break;
    default:
        break;
    }
}

std::vector<SurveyResult> session_take_survey_results() {
    std::vector<SurveyResult> out;
    out.swap(taken_survey_);
    return out;
}

void session_queue_survey_result(const SurveyResult& result) { taken_survey_.push_back(result); }

void session_restore_survey_results(std::vector<SurveyResult> pending) {
    if (pending.empty()) return;
    pending.insert(pending.end(), taken_survey_.begin(), taken_survey_.end());
    taken_survey_.swap(pending);
}

void session_set_work(WorkMode mode) { work_ = mode; }

std::optional<ProjectionReady> session_take_projection_ready() {
    std::optional<ProjectionReady> out;
    out.swap(taken_ready_);
    return out;
}

}  // namespace ccu
