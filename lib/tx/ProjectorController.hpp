#pragma once

#include "../utils/projector_protocol.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <span>
#include <string>
#include <vector>

class SerialDevice;

class ProjectorController {
public:
    using StatusPred = std::function<bool(const ProjectorStatus&)>;

    explicit ProjectorController(SerialDevice& device);

    void on_status(const ProjectorStatus& status);
    void on_text(uint8_t kind, std::span<const uint8_t> text);
    void on_notify(uint8_t cmd);

    bool has_status() const;
    ProjectorStatus status() const;
    std::vector<std::string> take_text();

    bool wait_for_status(StatusPred pred, std::chrono::milliseconds timeout);

    uint16_t x() const;
    uint16_t y() const;
    bool laser() const;
    bool pointing() const;
    bool streaming() const;

    bool request_status();
    bool set_enabled(bool enabled);
    bool set_project_mode(uint8_t mode);
    bool enable_pointing(uint16_t duty_percent);
    bool disable_pointing();
    bool nudge(int dx, int dy);
    bool send_current_point();

    bool play();
    bool stop();
    bool set_laser_duty_cycle(uint16_t percent);
    bool set_playback_speed(uint16_t khz);
    bool reset_stream();
    bool get_fault_msg();

    bool start_path_stream(std::vector<ProjPathPoint> points, uint16_t khz,
                           uint16_t duty_percent);
    bool update_path_stream(std::vector<ProjPathPoint> points);
    bool stop_path_stream();
    void pump_stream();

    bool send(uint8_t cmd);

private:
    bool send_payload(uint8_t cmd, std::span<const uint8_t> payload);
    bool toggle_flag(uint8_t current, bool want, uint8_t cmd, bool have);
    bool send_stream_frame(std::size_t n_records);
    void drain_ring_estimate_locked();

    SerialDevice& device_;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    bool have_status_ = false;
    ProjectorStatus status_{};
    std::vector<std::string> text_lines_;
    std::chrono::steady_clock::time_point last_status_tx_{};

    uint16_t x_ = kProjDacCentre;
    uint16_t y_ = kProjDacCentre;
    bool laser_ = false;
    bool pointing_ = false;

    bool streaming_ = false;
    bool send_gate_ = true;
    bool awaiting_reset_ack_ = false;
    bool pending_play_ = false;
    uint8_t packet_index_ = 1;
    std::size_t cursor_ = 0;
    std::size_t bytes_in_ring_ = 0;
    uint16_t playback_khz_ = 20;
    std::vector<ProjPathPoint> path_points_;
    std::chrono::steady_clock::time_point last_stream_tx_{};
    std::chrono::steady_clock::time_point reset_sent_at_{};
};
