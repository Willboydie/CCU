#pragma once

#include "IFrameParser.hpp"

#include <cstdint>

// PROJECTOR_HOST_PROTOCOL.md RX parser: interleaved debug lines
// (0xFF 0x04 0x06 ... \n), raw flow-control bytes (0xF7/0xF8/0xF9),
// and framed status (0xFA + uint16 LE length + payload).
class ProjectorParser : public IFrameParser {
public:
    ProjectorParser();

    void set_buffer(uint8_t* buffer, uint32_t cap) override;
    void set_on_frame(FrameHandler on_frame) override;
    void pump(uint8_t byte) override;
    void reset() override;

private:
    enum class State : uint8_t {
        Await,
        DebugP1,
        DebugP2,
        DebugBody,
        FrameLen,
        FramePayload,
        SkipPayload,
    };

    void complete_frame();

    State state_ = State::Await;
    uint8_t cmd_ = 0;
    uint8_t* buffer_ = nullptr;
    uint32_t cap_ = 0;
    uint32_t payload_bytes_read_ = 0;
    uint8_t payload_length_bytes_[2] = {0, 0};
    uint8_t payload_length_bytes_read_ = 0;
    uint16_t payload_length_ = 0;
    FrameHandler on_frame_;
};
