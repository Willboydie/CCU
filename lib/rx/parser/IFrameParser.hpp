#pragma once

#include <cstdint>
#include <functional>

class IFrameParser {
public:
    using FrameHandler = std::function<void(uint8_t cmd, uint16_t len)>;

    virtual ~IFrameParser() = default;

    virtual void set_buffer(uint8_t* buffer, uint32_t cap) = 0;
    virtual void set_on_frame(FrameHandler on_frame) = 0;
    virtual void pump(uint8_t byte) = 0;
    virtual void reset() = 0;
};
