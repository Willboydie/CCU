#include "DeviceParser.hpp"

DeviceParser::DeviceParser() = default;

void DeviceParser::set_buffer(uint8_t* buffer, uint32_t cap) {
    buffer_ = buffer;
    cap_ = cap;
}

void DeviceParser::set_on_frame(FrameHandler on_frame) {
    on_frame_ = std::move(on_frame);
}

void DeviceParser::reset() {
    state_ = State::AwaitCmd;
    cmd_ = 0;
    payload_bytes_read_ = 0;
    payload_length_bytes_[0] = 0;
    payload_length_bytes_[1] = 0;
    payload_length_bytes_read_ = 0;
    payload_length_ = 0;
}

void DeviceParser::complete_frame() {
    state_ = State::AwaitCmd;
    payload_bytes_read_ = 0;
    payload_length_bytes_read_ = 0;
    if (on_frame_)
        on_frame_(cmd_, payload_length_);
}

void DeviceParser::pump(uint8_t byte) {
    switch (state_) {
        case State::AwaitCmd:
            cmd_ = byte;
            state_ = State::AwaitPayloadLength;
            payload_length_bytes_read_ = 0;
            break;

        case State::AwaitPayloadLength:
            payload_length_bytes_[payload_length_bytes_read_++] = byte;
            if (payload_length_bytes_read_ != 2)
                break;

            payload_length_ = static_cast<uint16_t>(
                payload_length_bytes_[0] | (payload_length_bytes_[1] << 8));
            payload_length_bytes_read_ = 0;
            payload_bytes_read_ = 0;

            if (payload_length_ == 0) {
                complete_frame();
                break;
            }
            if (buffer_ == nullptr || payload_length_ > cap_) {
                state_ = State::SkipPayload;
                break;
            }
            state_ = State::AwaitPayload;
            break;

        case State::AwaitPayload:
            buffer_[payload_bytes_read_++] = byte;
            if (payload_bytes_read_ != payload_length_)
                break;
            complete_frame();
            break;

        case State::SkipPayload:
            ++payload_bytes_read_;
            if (payload_bytes_read_ != payload_length_)
                break;
            reset();
            break;
    }
}
