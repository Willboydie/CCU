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

    bool request_status();
    bool set_enabled(bool enabled);
    bool enable_pointing(uint16_t duty_percent);
    bool disable_pointing();
    bool nudge(int dx, int dy);
    bool send_current_point();

    bool play();
    bool stop();
    bool set_laser_duty_cycle(uint16_t percent);
    bool get_fault_msg();

    bool send(uint8_t cmd);

private:
    bool send_payload(uint8_t cmd, std::span<const uint8_t> payload);
    bool toggle_flag(uint8_t current, bool want, uint8_t cmd, bool have);

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
};
