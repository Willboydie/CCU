#include "ProjectorController.hpp"

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
    if (cmd == kProjNotifyPauseStream) {
        send_gate_ = false;
        if (bytes_in_ring_ < kProjRingPauseBytes)
            bytes_in_ring_ = kProjRingPauseBytes;
    } else if (cmd == kProjNotifyResumeStream) {
        send_gate_ = true;
        if (bytes_in_ring_ > kProjRingResumeBytes)
            bytes_in_ring_ = kProjRingResumeBytes;
    } else if (cmd == kProjNotifyResetStream) {
        bytes_in_ring_ = 0;
        packet_index_ = 1;
        cursor_ = 0;
        send_gate_ = true;
        if (streaming_ && awaiting_reset_ack_) {
            awaiting_reset_ack_ = false;
            pending_play_ = true;
        }
    }

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

bool ProjectorController::streaming() const {
    std::lock_guard<std::mutex> lock(mu_);
    return streaming_;
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

bool ProjectorController::set_project_mode(uint8_t mode) {
    if (mode != kProjModePath && mode != kProjModePoint)
        return false;
    if (!has_status())
        return false;

    bool was_streaming = false;
    uint8_t current = kProjModePath;
    {
        std::lock_guard<std::mutex> lock(mu_);
        was_streaming = streaming_;
        current = have_status_ ? status_.project_mode : kProjModePath;
        if (current == mode)
            return true;
    }
    if (was_streaming && !stop_path_stream())
        return false;

    if (current == kProjModePoint && mode == kProjModePath) {
        {
            std::lock_guard<std::mutex> lock(mu_);
            laser_ = false;
            pointing_ = false;
        }
        send_current_point();
        stop();
    }

    if (!send(kProjCmdToggleProjectMode))
        return false;
    {
        std::lock_guard<std::mutex> lock(mu_);
        status_.project_mode = mode;
        status_.mode = mode;
    }
    return true;
}

bool ProjectorController::enable_pointing(uint16_t duty_percent) {
    if (!has_status())
        return false;
    if (streaming() && !stop_path_stream())
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
bool ProjectorController::reset_stream() { return send(kProjCmdResetStream); }
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

bool ProjectorController::set_playback_speed(uint16_t khz) {
    if (khz < 1)
        khz = 1;
    if (khz > 200)
        khz = 200;
    {
        std::lock_guard<std::mutex> lock(mu_);
        playback_khz_ = khz;
    }
    const uint8_t payload[2] = {
        static_cast<uint8_t>(khz & 0xFF),
        static_cast<uint8_t>((khz >> 8) & 0xFF),
    };
    return send_payload(kProjCmdSetPlaybackSpeed, payload);
}

bool ProjectorController::start_path_stream(std::vector<ProjPathPoint> points,
                                            uint16_t khz,
                                            uint16_t duty_percent) {
    if (points.empty())
        return false;
    if (!has_status())
        return false;

    if (streaming())
        stop_path_stream();

    if (pointing()) {
        {
            std::lock_guard<std::mutex> lock(mu_);
            laser_ = false;
            pointing_ = false;
        }
        send_current_point();
        stop();
    }

    if (!set_enabled(true))
        return false;
    if (!set_project_mode(kProjModePath))
        return false;

    uint8_t debug = 0;
    {
        std::lock_guard<std::mutex> lock(mu_);
        debug = have_status_ ? status_.debug_mode : 0;
    }
    if (debug && !send(kProjCmdDebugMode))
        return false;

    if (!set_playback_speed(khz))
        return false;
    if (!set_laser_duty_cycle(duty_percent))
        return false;
    if (!reset_stream())
        return false;

    const auto now = std::chrono::steady_clock::now();
    {
        std::lock_guard<std::mutex> lock(mu_);
        path_points_ = std::move(points);
        cursor_ = 0;
        packet_index_ = 1;
        bytes_in_ring_ = 0;
        send_gate_ = true;
        streaming_ = true;
        awaiting_reset_ack_ = true;
        pending_play_ = false;
        reset_sent_at_ = now;
        last_stream_tx_ = now;
        if (have_status_)
            status_.debug_mode = 0;
    }
    return true;
}

bool ProjectorController::update_path_stream(std::vector<ProjPathPoint> points) {
    if (points.empty())
        return false;
    std::lock_guard<std::mutex> lock(mu_);
    if (!streaming_)
        return false;
    const std::size_t old_n = path_points_.size();
    const double phase = (old_n > 0)
        ? static_cast<double>(cursor_) / static_cast<double>(old_n)
        : 0.0;
    path_points_ = std::move(points);
    cursor_ = static_cast<std::size_t>(phase * path_points_.size());
    if (cursor_ >= path_points_.size())
        cursor_ = 0;
    return true;
}

bool ProjectorController::stop_path_stream() {
    {
        std::lock_guard<std::mutex> lock(mu_);
        streaming_ = false;
        awaiting_reset_ack_ = false;
        pending_play_ = false;
        send_gate_ = true;
        cursor_ = 0;
        packet_index_ = 1;
        bytes_in_ring_ = 0;
        path_points_.clear();
    }
    return reset_stream();
}

void ProjectorController::drain_ring_estimate_locked() {
    const auto now = std::chrono::steady_clock::now();
    if (last_stream_tx_.time_since_epoch().count() == 0) {
        last_stream_tx_ = now;
        return;
    }
    const bool draining = have_status_ && status_.projecting &&
                          status_.stream_state != kProjStreamReset;
    if (draining) {
        const double dt = std::chrono::duration<double>(now - last_stream_tx_).count();
        const auto drain = static_cast<std::size_t>(
            playback_khz_ * 1000.0 * kProjStreamRecordSize * dt);
        if (drain >= bytes_in_ring_)
            bytes_in_ring_ = 0;
        else
            bytes_in_ring_ -= drain;
    }
    last_stream_tx_ = now;
}

bool ProjectorController::send_stream_frame(std::size_t n_records) {
    if (n_records == 0 || path_points_.empty())
        return false;
    if (n_records > kProjStreamMaxRecords)
        n_records = kProjStreamMaxRecords;

    const std::size_t payload_len = 1 + n_records * kProjStreamRecordSize;
    std::vector<uint8_t> payload(payload_len);
    payload[0] = packet_index_;

    for (std::size_t i = 0; i < n_records; ++i) {
        const ProjPathPoint& p = path_points_[cursor_];
        pack_stream_record(payload.data() + 1 + i * kProjStreamRecordSize,
                           p.x, p.y, p.laser);
        ++cursor_;
        if (cursor_ >= path_points_.size())
            cursor_ = 0;
    }

    auto frame = pack_projector_frame(kProjCmdPointStream, payload);
    if (!frame)
        return false;
    if (!device_.write(*frame))
        return false;

    packet_index_ = static_cast<uint8_t>(packet_index_ + 1);
    bytes_in_ring_ += n_records * kProjStreamRecordSize;
    return true;
}

void ProjectorController::pump_stream() {
    bool should_play = false;
    {
        std::lock_guard<std::mutex> lock(mu_);
        if (!streaming_)
            return;

        const auto now = std::chrono::steady_clock::now();
        if (awaiting_reset_ack_) {
            if (now - reset_sent_at_ < std::chrono::milliseconds(400))
                return;
            awaiting_reset_ack_ = false;
            pending_play_ = true;
        }
        if (pending_play_) {
            pending_play_ = false;
            should_play = true;
        }
    }

    if (should_play)
        play();

    std::lock_guard<std::mutex> lock(mu_);
    if (!streaming_ || !send_gate_ || path_points_.empty())
        return;

    drain_ring_estimate_locked();

    constexpr std::size_t kMaxFramesPerPump = 8;
    const std::size_t cap = (kProjRingBytes * 7) / 10;
    std::size_t frames = 0;
    while (frames < kMaxFramesPerPump && bytes_in_ring_ < cap) {
        if (!send_stream_frame(kProjStreamMaxRecords))
            break;
        ++frames;
    }
}

bool ProjectorController::send(uint8_t cmd) {
    return send_payload(cmd, {});
}

bool ProjectorController::send_payload(uint8_t cmd,
                                       std::span<const uint8_t> payload) {
    auto frame = pack_projector_frame(cmd, payload);
    if (!frame)
        return false;
    return device_.write(*frame);
}
