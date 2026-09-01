#pragma once

#include "../msg_queue/MessageQueue.hpp"
#include "../parser/DeviceParser.hpp"
#include "../parser/IFrameParser.hpp"
#include "../../utils/params.hpp"

#include <boost/asio.hpp>
#include <array>
#include <atomic>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <thread>
#include <vector>

class SerialDevice {
public:
    SerialDevice(std::string port, int baudrate, MessageQueue& queue,
                 IFrameParser* parser = nullptr);
    ~SerialDevice();

    SerialDevice(const SerialDevice&) = delete;
    SerialDevice& operator=(const SerialDevice&) = delete;

    bool open();
    void close();
    void start();
    void stop();
    bool write(std::span<const uint8_t> data);

private:
    struct Session : std::enable_shared_from_this<Session> {
        explicit Session(boost::asio::io_context& ioc)
            : port(ioc) {}

        void arm_read();
        void enqueue_write(std::vector<uint8_t> data);
        void pump_write();

        boost::asio::serial_port port;
        std::atomic<bool> running{false};
        std::function<void(std::span<const uint8_t>)> on_bytes;
        std::vector<uint8_t> read_buf;
        std::deque<std::vector<uint8_t>> write_q;
        bool writing = false;
    };

    struct Slot {
        std::array<uint8_t, RX_SERIAL_DEVICE_READ_BUFFER_LENGTH> data{};
        std::atomic<bool> in_use{false};
    };

    void start_io_thread();
    void stop_io_thread();
    void publish_frame(uint8_t cmd, uint16_t len);
    std::size_t find_free_slot() const;
    void wait_for_in_flight();

    std::string port_name_;
    int baudrate_;
    MessageQueue& queue_;
    boost::asio::io_context ioc_;
    std::optional<boost::asio::executor_work_guard<
        boost::asio::io_context::executor_type>> work_;
    std::thread thread_;
    std::shared_ptr<Session> session_;

    static constexpr std::size_t kRingSlots = RX_SERIAL_DEVICE_PAYLOAD_RING_SLOTS;
    std::array<Slot, kRingSlots> ring_{};
    std::size_t current_slot_ = 0;
    DeviceParser owned_parser_;
    IFrameParser* parser_ = nullptr;
};
