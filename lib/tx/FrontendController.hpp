#pragma once

#include "../utils/frontend_protocol.hpp"

#include <atomic>
#include <cstdint>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

class SerialDevice;

class FrontendController {
public:
    explicit FrontendController(SerialDevice& device);
    ~FrontendController();

    FrontendController(const FrontendController&) = delete;
    FrontendController& operator=(const FrontendController&) = delete;

    void set_telemetry(const FrontendTelemetry& t);
    FrontendTelemetry telemetry() const;

    bool begin_session(const std::string& firmware_version);
    bool send_firmware_version(const std::string& firmware_version);
    bool send_telemetry_once();
    bool send_survey_point(const FrontendSurveyPoint& point);
    bool send_projection_start(const std::vector<FrontendNode>& nodes);

    void set_streaming(bool on);
    bool streaming() const;

    void set_apply_rx(bool on);
    bool apply_rx() const;

    void on_rx(uint8_t cmd, std::span<const uint8_t> payload);
    std::vector<std::string> take_log();

    uint64_t telemetry_count() const;
    uint64_t rx_count() const;

private:
    bool write_frame(const std::vector<uint8_t>& frame);
    void log_line(std::string line);
    void apply_rx_locked(uint8_t cmd, std::span<const uint8_t> payload);
    void ticker_main();

    SerialDevice& device_;
    mutable std::mutex mu_;
    FrontendTelemetry telemetry_{};
    std::vector<std::string> log_;
    std::thread ticker_;
    std::atomic<bool> streaming_{false};
    std::atomic<bool> apply_rx_{false};
    std::atomic<uint64_t> telemetry_n_{0};
    std::atomic<uint64_t> rx_n_{0};
};
