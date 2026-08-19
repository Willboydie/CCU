#pragma once

#include "../utils/protocol.hpp"

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <mutex>
#include <span>
#include <string>
#include <vector>

class SerialDevice;

class LidarController {
public:
    using StatusPred = std::function<bool(const StatusPayload&)>;

    explicit LidarController(SerialDevice& device);

    // Consumer-thread observers. Do not call wait_* from this thread.
    void on_status(const StatusPayload& status);
    void on_control(const ControlPayload& control);
    void on_text(uint8_t kind, std::span<const uint8_t> text);

    bool has_status() const;
    StatusPayload status() const;
    bool has_control() const;
    ControlPayload control() const;
    std::vector<std::string> take_text();

    bool wait_for_status(StatusPred pred, std::chrono::milliseconds timeout);
    bool wait_until_idle(std::chrono::milliseconds timeout);
    bool wait_enabled(bool enabled, std::chrono::milliseconds timeout);
    bool wait_scan_mode(uint8_t mode, std::chrono::milliseconds timeout);

    bool set_enabled(bool enabled);
    bool set_debug_mode(bool on);
    bool toggle_control_mode();
    bool sleep();
    bool wake();
    bool restart();
    bool enter_bootloader();
    bool get_error_msg();
    bool reconfigure_ldm(float preset);

    bool set_target_x(float x);
    bool set_target_y(float y);

    bool scan_engine_on();
    bool scan_engine_off();
    bool seek_neutral();
    bool set_point_to_point_target(float x, float y);
    bool begin_coarse_scanning();
    bool stop_coarse_scanning();
    bool update_coarse_scan_centre(float x, float y);

    bool set_x_amplitude(float v);
    bool set_y_amplitude(float v);
    bool set_x_frequency(float v);
    bool set_y_frequency(float v);
    bool set_x_phase(float v);
    bool set_y_phase(float v);

    bool send(uint8_t cmd);
    bool send_f32(uint8_t cmd, std::initializer_list<float> args);

private:
    bool send_payload(uint8_t cmd, std::span<const uint8_t> payload);
    bool toggle_flag(uint8_t current, bool want, uint8_t cmd, bool have);

    SerialDevice& device_;
    mutable std::mutex mu_;
    std::condition_variable cv_;
    bool have_status_ = false;
    bool have_control_ = false;
    StatusPayload status_{};
    ControlPayload control_{};
    std::vector<std::string> text_lines_;
};
