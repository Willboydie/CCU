#include "ProjectorParser.hpp"

#include "../../utils/projector_protocol.hpp"

ProjectorParser::ProjectorParser() = default;

void ProjectorParser::set_buffer(uint8_t* buffer, uint32_t cap) {
    buffer_ = buffer;
    cap_ = cap;
}

void ProjectorParser::set_on_frame(FrameHandler on_frame) {
    on_frame_ = std::move(on_frame);
}

void ProjectorParser::reset() {
    state_ = State::Await;
    cmd_ = 0;
    payload_bytes_read_ = 0;
    payload_length_bytes_[0] = 0;
    payload_length_bytes_[1] = 0;
    payload_length_bytes_read_ = 0;
    payload_length_ = 0;
}

void ProjectorParser::complete_frame() {
    state_ = State::Await;
    payload_bytes_read_ = 0;
    payload_length_bytes_read_ = 0;
    if (on_frame_)
        on_frame_(cmd_, payload_length_);
}

void ProjectorParser::pump(uint8_t byte) {
    switch (state_) {
        case State::Await:
            if (byte == kProjStatusHeader) {
                cmd_ = byte;
                state_ = State::FrameLen;
                payload_length_bytes_read_ = 0;
            } else if (byte == kProjNotifyPauseStream ||
                       byte == kProjNotifyResumeStream ||
                       byte == kProjNotifyResetStream) {
                cmd_ = byte;
                payload_length_ = 0;
                complete_frame();
            } else if (byte == kProjDebugPrefix0) {
                state_ = State::DebugP1;
            }
            break;

        case State::DebugP1:
            if (byte == kProjDebugPrefix1) {
                state_ = State::DebugP2;
            } else {
                state_ = State::Await;
                pump(byte);
            }
            break;

        case State::DebugP2:
            if (byte == kProjDebugPrefix2) {
                cmd_ = kProjDebugHeader;
                payload_bytes_read_ = 0;
                payload_length_ = 0;
                state_ = State::DebugBody;
            } else {
                state_ = State::Await;
                pump(byte);
            }
            break;

        case State::DebugBody:
            if (byte == '\n') {
                payload_length_ = static_cast<uint16_t>(payload_bytes_read_);
                complete_frame();
            } else if (buffer_ != nullptr && payload_bytes_read_ < cap_) {
                buffer_[payload_bytes_read_++] = byte;
            }
            break;

        case State::FrameLen:
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
            state_ = State::FramePayload;
            break;

        case State::FramePayload:
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
