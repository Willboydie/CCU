#pragma once

#include <cstdint>
#include <functional>

class DeviceParser {
public:
    using FrameHandler = std::function<void(uint8_t cmd, uint16_t len)>;

    DeviceParser();

    void set_buffer(uint8_t* buffer, uint32_t cap);
    void set_on_frame(FrameHandler on_frame);
    void pump(uint8_t byte);
    void reset();

private:
    enum class State : uint8_t {
        AwaitCmd,
        AwaitPayloadLength,
        AwaitPayload,
        SkipPayload,
    };

    void complete_frame();

    State state_ = State::AwaitCmd;
    uint8_t cmd_ = 0;
    uint8_t* buffer_ = nullptr;
    uint32_t cap_ = 0;
    uint32_t payload_bytes_read_ = 0;
    uint8_t payload_length_bytes_[2] = {0, 0};
    uint8_t payload_length_bytes_read_ = 0;
    uint16_t payload_length_ = 0;
    FrameHandler on_frame_;
};
