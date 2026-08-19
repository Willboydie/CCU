#include "LidarController.hpp"

#include "CommandEncoder.hpp"
#include "../rx/serial/SerialDevice.hpp"
#include "../utils/cmd_refs.hpp"

#include <utility>

LidarController::LidarController(SerialDevice& device)
    : device_(device) {}

void LidarController::on_status(const StatusPayload& status) {
    bool fetch_errors = false;
    {
        std::lock_guard<std::mutex> lock(mu_);
        fetch_errors = status.has_fault && (!have_status_ || !status_.has_fault);
        status_ = status;
        have_status_ = true;
    }
    cv_.notify_all();
    if (fetch_errors)
        get_error_msg();
}

void LidarController::on_control(const ControlPayload& control) {
    {
        std::lock_guard<std::mutex> lock(mu_);
        control_ = control;
        have_control_ = true;
    }
    cv_.notify_all();
}

void LidarController::on_text(uint8_t kind, std::span<const uint8_t> text) {
    std::string decoded = decode_debug_ascii(text);
    while (!decoded.empty() && (decoded.back() == '\n' || decoded.back() == '\r'))
        decoded.pop_back();

    std::string line = (kind == kDebugHeader) ? "[debug] " : "[text] ";
    line += decoded;
    {
        std::lock_guard<std::mutex> lock(mu_);
        text_lines_.push_back(std::move(line));
        if (text_lines_.size() > 64)
            text_lines_.erase(text_lines_.begin());
    }
    cv_.notify_all();
}

bool LidarController::has_status() const {
    std::lock_guard<std::mutex> lock(mu_);
    return have_status_;
}

StatusPayload LidarController::status() const {
    std::lock_guard<std::mutex> lock(mu_);
    return status_;
}

bool LidarController::has_control() const {
    std::lock_guard<std::mutex> lock(mu_);
    return have_control_;
}

ControlPayload LidarController::control() const {
    std::lock_guard<std::mutex> lock(mu_);
    return control_;
}

std::vector<std::string> LidarController::take_text() {
    std::lock_guard<std::mutex> lock(mu_);
    std::vector<std::string> out;
    out.swap(text_lines_);
    return out;
}

bool LidarController::wait_for_status(StatusPred pred, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mu_);
    return cv_.wait_for(lock, timeout, [&] {
        return have_status_ && pred(status_);
    });
}

bool LidarController::wait_until_idle(std::chrono::milliseconds timeout) {
    return wait_for_status(
        [](const StatusPayload& s) { return s.dynamic_routine == kRoutineNone; },
        timeout);
}

bool LidarController::wait_enabled(bool enabled, std::chrono::milliseconds timeout) {
    return wait_for_status(
        [enabled](const StatusPayload& s) {
            return (s.enabled != 0) == enabled;
        },
        timeout);
}

bool LidarController::wait_scan_mode(uint8_t mode, std::chrono::milliseconds timeout) {
    return wait_for_status(
        [mode](const StatusPayload& s) { return s.scan_mode == mode; },
        timeout);
}

bool LidarController::toggle_flag(uint8_t current, bool want, uint8_t cmd, bool have) {
    if (!have)
        return false;
    if ((current != 0) == want)
        return true;
    return send(cmd);
}

bool LidarController::set_enabled(bool enabled) {
    uint8_t current = 0;
    bool have = false;
    {
        std::lock_guard<std::mutex> lock(mu_);
        have = have_status_;
        current = status_.enabled;
    }
    return toggle_flag(current, enabled, kCmdDisableEnable, have);
}

bool LidarController::set_debug_mode(bool on) {
    uint8_t current = 0;
    bool have = false;
    {
        std::lock_guard<std::mutex> lock(mu_);
        have = have_status_;
        current = status_.debug_mode;
    }
    return toggle_flag(current, on, kCmdSetDebugMode, have);
}

bool LidarController::toggle_control_mode() { return send(kCmdToggleControlMode); }
bool LidarController::sleep() { return send(kCmdSleep); }
bool LidarController::wake() { return send(kCmdWake); }
bool LidarController::restart() { return send(kCmdRestart); }
bool LidarController::enter_bootloader() { return send(kCmdEnterBootloader); }
bool LidarController::get_error_msg() { return send(kCmdGetErrorMsg); }

bool LidarController::reconfigure_ldm(float preset) {
    return send_f32(kCmdReconfigureLdm, {preset});
}

bool LidarController::set_target_x(float x) { return send_f32(kCmdTargetXSet, {x}); }
bool LidarController::set_target_y(float y) { return send_f32(kCmdTargetYSet, {y}); }

bool LidarController::scan_engine_on() { return send(kCmdScanEngineOn); }
bool LidarController::scan_engine_off() { return send(kCmdScanEngineOff); }
bool LidarController::seek_neutral() { return send(kCmdSeekNeutral); }

bool LidarController::set_point_to_point_target(float x, float y) {
    return send_f32(kCmdSetPoint2PointTarget, {x, y});
}

bool LidarController::begin_coarse_scanning() { return send(kCmdBeginCoarseScanning); }
bool LidarController::stop_coarse_scanning() { return send(kCmdStopCoarseScanning); }

bool LidarController::update_coarse_scan_centre(float x, float y) {
    return send_f32(kCmdUpdateCoarseScanCentre, {x, y});
}

bool LidarController::set_x_amplitude(float v) { return send_f32(kCmdXAmplitudeSet, {v}); }
bool LidarController::set_y_amplitude(float v) { return send_f32(kCmdYAmplitudeSet, {v}); }
bool LidarController::set_x_frequency(float v) { return send_f32(kCmdXFrequencySet, {v}); }
bool LidarController::set_y_frequency(float v) { return send_f32(kCmdYFrequencySet, {v}); }
bool LidarController::set_x_phase(float v) { return send_f32(kCmdXPhaseSet, {v}); }
bool LidarController::set_y_phase(float v) { return send_f32(kCmdYPhaseSet, {v}); }

bool LidarController::send(uint8_t cmd) {
    return send_payload(cmd, {});
}

bool LidarController::send_f32(uint8_t cmd, std::initializer_list<float> args) {
    auto frame = CommandEncoder::pack_f32(cmd, args);
    if (!frame)
        return false;
    return device_.write(*frame);
}

bool LidarController::send_payload(uint8_t cmd, std::span<const uint8_t> payload) {
    auto frame = CommandEncoder::pack(cmd, payload);
    if (!frame)
        return false;
    return device_.write(*frame);
}
