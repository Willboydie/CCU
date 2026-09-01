#include "ProjectorController.hpp"

#include "CommandEncoder.hpp"
#include "../rx/serial/SerialDevice.hpp"
#include "../utils/protocol.hpp"

#include <utility>

ProjectorController::ProjectorController(SerialDevice& device)
    : device_(device) {}

void ProjectorController::on_status(const ProjectorStatus& status) {
    bool fetch_errors = false;
    {
        std::lock_guard<std::mutex> lock(mu_);
        fetch_errors = status.has_fault && (!have_status_ || !status_.has_fault);
        status_ = status;
        have_status_ = true;
    }
    cv_.notify_all();
    if (fetch_errors)
        get_fault_msg();
}

void ProjectorController::on_text(uint8_t kind, std::span<const uint8_t> text) {
    std::string decoded = decode_debug_ascii(text);
    while (!decoded.empty() && (decoded.back() == '\n' || decoded.back() == '\r'))
        decoded.pop_back();

    std::string line = (kind == kProjDebugHeader) ? "[debug] " : "[text] ";
    line += decoded;
    {
        std::lock_guard<std::mutex> lock(mu_);
        text_lines_.push_back(std::move(line));
        if (text_lines_.size() > 64)
            text_lines_.erase(text_lines_.begin());
    }
    cv_.notify_all();
}

void ProjectorController::on_notify(uint8_t cmd) {
    const char* name = "notify";
    if (cmd == kProjNotifyPauseStream)
        name = "PAUSE_STREAM";
    else if (cmd == kProjNotifyResumeStream)
        name = "RESUME_STREAM";
    else if (cmd == kProjNotifyResetStream)
        name = "RESET_STREAM";

    std::lock_guard<std::mutex> lock(mu_);
    text_lines_.push_back(std::string("[flow] ") + name);
    if (text_lines_.size() > 64)
        text_lines_.erase(text_lines_.begin());
}

bool ProjectorController::has_status() const {
    std::lock_guard<std::mutex> lock(mu_);
    return have_status_;
}

ProjectorStatus ProjectorController::status() const {
    std::lock_guard<std::mutex> lock(mu_);
    return status_;
}

std::vector<std::string> ProjectorController::take_text() {
    std::lock_guard<std::mutex> lock(mu_);
    std::vector<std::string> out;
    out.swap(text_lines_);
    return out;
}

bool ProjectorController::wait_for_status(StatusPred pred,
                                          std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::unique_lock<std::mutex> lock(mu_);
    while (std::chrono::steady_clock::now() < deadline) {
        if (have_status_ && pred(status_))
            return true;
        lock.unlock();
        request_status();
        lock.lock();
        const auto remain = deadline - std::chrono::steady_clock::now();
        if (remain <= std::chrono::steady_clock::duration::zero())
            break;
        auto slice = std::chrono::duration_cast<std::chrono::milliseconds>(remain);
        if (slice > std::chrono::milliseconds(80))
            slice = std::chrono::milliseconds(80);
        cv_.wait_for(lock, slice);
    }
    return have_status_ && pred(status_);
}

uint16_t ProjectorController::x() const {
    std::lock_guard<std::mutex> lock(mu_);
    return x_;
}

uint16_t ProjectorController::y() const {
    std::lock_guard<std::mutex> lock(mu_);
    return y_;
}

bool ProjectorController::laser() const {
    std::lock_guard<std::mutex> lock(mu_);
    return laser_;
}

bool ProjectorController::pointing() const {
    std::lock_guard<std::mutex> lock(mu_);
    return pointing_;
}

bool ProjectorController::request_status() {
    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (last_status_tx_.time_since_epoch().count() != 0 &&
            now - last_status_tx_ < std::chrono::milliseconds(100))
            return true;
        last_status_tx_ = now;
    }
    return send(kProjCmdSendStatus);
}

bool ProjectorController::toggle_flag(uint8_t current, bool want, uint8_t cmd,
                                      bool have) {
    if (!have)
        return false;
    if ((current != 0) == want)
        return true;
    return send(cmd);
}

bool ProjectorController::set_enabled(bool enabled) {
    uint8_t current = 0;
    bool have = false;
    {
        std::lock_guard<std::mutex> lock(mu_);
        have = have_status_;
        current = status_.enabled;
    }
    if (!toggle_flag(current, enabled, kProjCmdEnableDisable, have))
        return false;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (have_status_)
            status_.enabled = enabled ? 1 : 0;
    }
    return true;
}

bool ProjectorController::enable_pointing(uint16_t duty_percent) {
    if (!has_status())
        return false;
    if (!set_enabled(true))
        return false;

    uint8_t mode = kProjModePath;
    {
        std::lock_guard<std::mutex> lock(mu_);
        mode = status_.project_mode;
        laser_ = true;
        pointing_ = true;
    }
    if (mode != kProjModePoint) {
        if (!send(kProjCmdToggleProjectMode))
            return false;
        std::lock_guard<std::mutex> lock(mu_);
        status_.project_mode = kProjModePoint;
        status_.mode = kProjModePoint;
    }
    if (!set_laser_duty_cycle(duty_percent))
        return false;
    if (!play())
        return false;
    return send_current_point();
}

bool ProjectorController::disable_pointing() {
    {
        std::lock_guard<std::mutex> lock(mu_);
        laser_ = false;
        pointing_ = false;
    }
    send_current_point();
    stop();
    return set_enabled(false);
}

bool ProjectorController::nudge(int dx, int dy) {
    {
        std::lock_guard<std::mutex> lock(mu_);
        x_ = saturate_dac(static_cast<int>(x_) + dx);
        y_ = saturate_dac(static_cast<int>(y_) + dy);
        laser_ = true;
    }
    return send_current_point();
}

bool ProjectorController::send_current_point() {
    uint16_t x = 0;
    uint16_t y = 0;
    bool laser = false;
    {
        std::lock_guard<std::mutex> lock(mu_);
        x = x_;
        y = y_;
        laser = laser_;
    }
    uint8_t payload[5];
    pack_set_point(payload, x, y, laser);
    return send_payload(kProjCmdSetPoint, payload);
}

bool ProjectorController::play() { return send(kProjCmdPlay); }
bool ProjectorController::stop() { return send(kProjCmdStop); }
bool ProjectorController::get_fault_msg() { return send(kProjCmdGetFaultMsg); }

bool ProjectorController::set_laser_duty_cycle(uint16_t percent) {
    if (percent > 100)
        percent = 100;
    const uint8_t payload[2] = {
        static_cast<uint8_t>(percent & 0xFF),
        static_cast<uint8_t>((percent >> 8) & 0xFF),
    };
    return send_payload(kProjCmdSetLaserDuty, payload);
}

bool ProjectorController::send(uint8_t cmd) {
    return send_payload(cmd, {});
}

bool ProjectorController::send_payload(uint8_t cmd,
                                       std::span<const uint8_t> payload) {
    auto frame = CommandEncoder::pack(cmd, payload);
    if (!frame)
        return false;
    return device_.write(*frame);
}
