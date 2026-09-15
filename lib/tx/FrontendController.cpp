#include "FrontendController.hpp"

#include "../rx/serial/SerialDevice.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <ctime>

namespace {
constexpr int kLogMax = 2000;

std::string now_stamp() {
    using clock = std::chrono::system_clock;
    const auto now = clock::now();
    const std::time_t t = clock::to_time_t(now);
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        now.time_since_epoch()) %
                    1000;
    char buf[24];
    std::snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%03d  ",
                  tm.tm_hour, tm.tm_min, tm.tm_sec, static_cast<int>(ms.count()));
    return buf;
}
}

FrontendController::FrontendController(SerialDevice& device)
    : device_(device)
    , telemetry_(make_default_telemetry()) {}

FrontendController::~FrontendController() {
    set_streaming(false);
}

void FrontendController::set_telemetry(const FrontendTelemetry& t) {
    std::lock_guard<std::mutex> lock(mu_);
    telemetry_ = t;
    clamp_telemetry(telemetry_);
}

FrontendTelemetry FrontendController::telemetry() const {
    std::lock_guard<std::mutex> lock(mu_);
    return telemetry_;
}

bool FrontendController::write_frame(const std::vector<uint8_t>& frame) {
    if (frame.empty())
        return false;
    return device_.write(frame);
}

void FrontendController::log_line(std::string line) {
    const std::string stamp = now_stamp();
    if (line.size() >= 3 && (line.compare(0, 3, "TX ") == 0 ||
                             line.compare(0, 3, "RX ") == 0))
        line.insert(3, stamp);
    else
        line.insert(0, stamp);

    std::lock_guard<std::mutex> lock(mu_);
    log_.push_back(std::move(line));
    if (static_cast<int>(log_.size()) > kLogMax)
        log_.erase(log_.begin(), log_.begin() + (log_.size() - kLogMax));
}

bool FrontendController::send_firmware_version(const std::string& firmware_version) {
    auto frame = pack_firmware_version(firmware_version);
    if (!frame || !write_frame(*frame))
        return false;
    log_line(format_firmware_tx(firmware_version));
    return true;
}

bool FrontendController::send_telemetry_once() {
    FrontendTelemetry t;
    {
        std::lock_guard<std::mutex> lock(mu_);
        t = telemetry_;
    }
    auto frame = pack_telemetry(t);
    if (!frame || !write_frame(*frame))
        return false;
    telemetry_n_.fetch_add(1, std::memory_order_relaxed);
    return true;
}

bool FrontendController::send_survey_point(const FrontendSurveyPoint& point) {
    auto frame = pack_survey_point(point);
    if (!frame || !write_frame(*frame))
        return false;
    log_line(format_survey_tx(point));
    return true;
}

bool FrontendController::send_projection_start(const std::vector<FrontendNode>& nodes) {
    auto frame = pack_projection_start(nodes);
    if (!frame || !write_frame(*frame))
        return false;
    log_line(format_projection_tx(nodes));
    return true;
}

bool FrontendController::begin_session(const std::string& firmware_version) {
    if (!send_firmware_version(firmware_version))
        return false;
    send_telemetry_once();
    set_streaming(true);
    return true;
}

void FrontendController::set_streaming(bool on) {
    if (on) {
        bool expected = false;
        if (!streaming_.compare_exchange_strong(expected, true))
            return;
        ticker_ = std::thread([this] { ticker_main(); });
        return;
    }
    if (!streaming_.exchange(false))
        return;
    if (ticker_.joinable())
        ticker_.join();
}

bool FrontendController::streaming() const {
    return streaming_.load();
}

void FrontendController::set_apply_rx(bool on) {
    apply_rx_.store(on);
}

bool FrontendController::apply_rx() const {
    return apply_rx_.load();
}

void FrontendController::apply_rx_locked(uint8_t cmd, std::span<const uint8_t> payload) {
    switch (cmd) {
        case kFeCmdSetLaserPower:
            if (payload.size() == 1)
                telemetry_.laserPower = std::min<uint8_t>(payload[0], 100);
            break;
        case kFeCmdSetProjectionMode:
            if (payload.size() == 1) {
                telemetry_.projectionMode =
                    payload[0] > kFeModeProject ? kFeModeProject : payload[0];
                if (payload[0] == kFeModeOff) {
                    telemetry_.isSurveying = 0;
                    telemetry_.isProjecting = 0;
                    telemetry_.laserOn = 0;
                    telemetry_.surveyReady = telemetry_.healthOK && telemetry_.originIsSet &&
                                             !telemetry_.moved && telemetry_.poseValid &&
                                             telemetry_.connectionStatus == kFeConnLinked;
                }
            }
            break;
        case kFeCmdBeginTagSearch:
            telemetry_.laserOn = 1;
            telemetry_.isSurveying = 0;
            telemetry_.isProjecting = 0;
            break;
        case kFeCmdBeginSurvey:
            telemetry_.isSurveying = 1;
            telemetry_.surveyReady = 0;
            telemetry_.isProjecting = 0;
            telemetry_.projectionMode = kFeModeSurvey;
            telemetry_.laserOn = 1;
            break;
        case kFeCmdBeginProjection:
            telemetry_.isProjecting = 1;
            telemetry_.isSurveying = 0;
            telemetry_.projectionMode = kFeModeProject;
            telemetry_.laserOn = 1;
            break;
        case kFeCmdTagFound:
            telemetry_.laserOn = 0;
            break;
        case kFeCmdSetOrigin:
            if (payload.size() == sizeof(FrontendPose)) {
                FrontendPose pose{};
                if (unpack_pose(payload, pose)) {
                    pose.roll_rad = wrap_2pi(pose.roll_rad);
                    pose.pitch_rad = wrap_2pi(pose.pitch_rad);
                    pose.yaw_rad = wrap_2pi(pose.yaw_rad);
                    telemetry_.lastKnownPose = pose;
                    telemetry_.effectiveYaw = pose.yaw_rad;
                    telemetry_.originIsSet = 1;
                    telemetry_.poseValid = 1;
                    telemetry_.moved = 0;
                }
            }
            break;
        default:
            break;
    }
}

void FrontendController::on_rx(uint8_t cmd, std::span<const uint8_t> payload) {
    rx_n_.fetch_add(1, std::memory_order_relaxed);

    if (cmd == kFeCmdTelemetry)
        return;

    log_line(std::string("RX ") + format_frontend_rx(cmd, payload));

    if (!apply_rx_.load())
        return;

    std::lock_guard<std::mutex> lock(mu_);
    apply_rx_locked(cmd, payload);
    clamp_telemetry(telemetry_);
}

std::vector<std::string> FrontendController::take_log() {
    std::lock_guard<std::mutex> lock(mu_);
    std::vector<std::string> out;
    out.swap(log_);
    return out;
}

uint64_t FrontendController::telemetry_count() const {
    return telemetry_n_.load(std::memory_order_relaxed);
}

uint64_t FrontendController::rx_count() const {
    return rx_n_.load(std::memory_order_relaxed);
}

void FrontendController::ticker_main() {
    using clock = std::chrono::steady_clock;
    auto next = clock::now() + std::chrono::milliseconds(1000 / kFrontendTelemetryHz);
    while (streaming_.load()) {
        std::this_thread::sleep_until(next);
        if (!streaming_.load())
            break;
        send_telemetry_once();
        next += std::chrono::milliseconds(1000 / kFrontendTelemetryHz);
        const auto now = clock::now();
        if (next < now)
            next = now;
    }
}
