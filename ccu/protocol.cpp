#include "protocol.hpp"

#include <cstring>

namespace {

class Writer {
public:
    void u8(uint8_t v) {
        if (!ok_) return;
        buf_.push_back(v);
    }

    void u16(uint16_t v) {
        u8(static_cast<uint8_t>(v & 0xFF));
        u8(static_cast<uint8_t>((v >> 8) & 0xFF));
    }

    void i16(int16_t v) { u16(static_cast<uint16_t>(v)); }

    void f32(float v) {
        if (!ok_) return;
        uint32_t bits = 0;
        std::memcpy(&bits, &v, 4);
        u8(static_cast<uint8_t>(bits & 0xFF));
        u8(static_cast<uint8_t>((bits >> 8) & 0xFF));
        u8(static_cast<uint8_t>((bits >> 16) & 0xFF));
        u8(static_cast<uint8_t>((bits >> 24) & 0xFF));
    }

    void boolean(bool v) { u8(v ? 1 : 0); }

    void str(std::string_view s) {
        if (!ok_) return;
        if (s.size() > ccu::kStringMax) {
            ok_ = false;
            return;
        }
        u8(static_cast<uint8_t>(s.size()));
        if (!s.empty()) {
            buf_.insert(buf_.end(),
                        reinterpret_cast<const uint8_t*>(s.data()),
                        reinterpret_cast<const uint8_t*>(s.data()) + s.size());
        }
    }

    void fail() { ok_ = false; }
    bool ok() const { return ok_; }
    const std::vector<uint8_t>& bytes() const { return buf_; }

private:
    std::vector<uint8_t> buf_;
    bool ok_ = true;
};

class Reader {
public:
    explicit Reader(std::span<const uint8_t> in) : in_(in) {}

    uint8_t u8() {
        if (!ok_) return 0;
        if (i_ >= in_.size()) {
            ok_ = false;
            return 0;
        }
        return in_[i_++];
    }

    uint16_t u16() {
        const uint8_t lo = u8();
        const uint8_t hi = u8();
        return static_cast<uint16_t>(lo | (static_cast<uint16_t>(hi) << 8));
    }

    int16_t i16() { return static_cast<int16_t>(u16()); }

    float f32() {
        if (!ok_) return 0.f;
        if (i_ + 4 > in_.size()) {
            ok_ = false;
            return 0.f;
        }
        const uint32_t bits = static_cast<uint32_t>(in_[i_]) |
                              (static_cast<uint32_t>(in_[i_ + 1]) << 8) |
                              (static_cast<uint32_t>(in_[i_ + 2]) << 16) |
                              (static_cast<uint32_t>(in_[i_ + 3]) << 24);
        i_ += 4;
        float v = 0.f;
        std::memcpy(&v, &bits, 4);
        return v;
    }

    bool boolean() { return u8() != 0; }

    std::string str() {
        if (!ok_) return {};
        if (i_ >= in_.size()) {
            ok_ = false;
            return {};
        }
        const uint8_t n = in_[i_++];
        if (n > ccu::kStringMax) {
            ok_ = false;
            return {};
        }
        if (i_ + n > in_.size()) {
            ok_ = false;
            return {};
        }
        std::string s(reinterpret_cast<const char*>(in_.data() + i_), n);
        i_ += n;
        return s;
    }

    std::size_t remaining() const {
        if (!ok_ || i_ > in_.size()) return 0;
        return in_.size() - i_;
    }

    void fail() { ok_ = false; }
    bool ok() const { return ok_; }
    bool finished() const { return ok_ && i_ == in_.size(); }

private:
    std::span<const uint8_t> in_;
    std::size_t i_ = 0;
    bool ok_ = true;
};

struct View {
    uint8_t cmd = 0;
    std::span<const uint8_t> payload;
};

std::optional<View> view_frame(std::span<const uint8_t> bytes) {
    if (bytes.size() < 3) return std::nullopt;
    const uint8_t cmd = bytes[0];
    if (!ccu::cmd_is_known(cmd)) return std::nullopt;
    const uint16_t len = static_cast<uint16_t>(
        bytes[1] | (static_cast<uint16_t>(bytes[2]) << 8));
    if (bytes.size() != static_cast<std::size_t>(len) + 3) return std::nullopt;
    View view;
    view.cmd = cmd;
    view.payload = bytes.subspan(3);
    return view;
}

void write_pose(Writer& w, const ccu::Pose& pose) {
    w.f32(pose.east_mm);
    w.f32(pose.north_mm);
    w.f32(pose.up_mm);
    w.f32(pose.roll_rad);
    w.f32(pose.pitch_rad);
    w.f32(pose.yaw_rad);
}

ccu::Pose read_pose(Reader& r) {
    ccu::Pose pose;
    pose.east_mm = r.f32();
    pose.north_mm = r.f32();
    pose.up_mm = r.f32();
    pose.roll_rad = r.f32();
    pose.pitch_rad = r.f32();
    pose.yaw_rad = r.f32();
    return pose;
}

void write_node(Writer& w, const ccu::PlanNode& node) {
    w.u16(node.id);
    w.f32(node.east_mm);
    w.f32(node.north_mm);
    w.f32(node.corner_inner_angle_rad);
    w.f32(node.corner_orientation_rad);
}

ccu::PlanNode read_node(Reader& r) {
    ccu::PlanNode node;
    node.id = r.u16();
    node.east_mm = r.f32();
    node.north_mm = r.f32();
    node.corner_inner_angle_rad = r.f32();
    node.corner_orientation_rad = r.f32();
    return node;
}

bool plan_fits(std::size_t count) {
    if (count > 0xFFFF) return false;
    const std::size_t bytes = 2 + count * ccu::kPlanNodeBytes;
    return bytes <= 0xFFFF;
}

void write_nodes(Writer& w, const std::vector<ccu::PlanNode>& nodes) {
    if (!plan_fits(nodes.size())) {
        w.fail();
        return;
    }
    w.u16(static_cast<uint16_t>(nodes.size()));
    for (const ccu::PlanNode& node : nodes) write_node(w, node);
}

std::vector<ccu::PlanNode> read_nodes(Reader& r) {
    if (r.remaining() < 2) {
        r.fail();
        return {};
    }
    const uint16_t count = r.u16();
    const std::size_t need = static_cast<std::size_t>(count) * ccu::kPlanNodeBytes;
    if (r.remaining() != need) {
        r.fail();
        return {};
    }
    std::vector<ccu::PlanNode> nodes;
    nodes.reserve(count);
    for (uint16_t i = 0; i < count; ++i) nodes.push_back(read_node(r));
    return nodes;
}

std::optional<std::vector<uint8_t>> seal(uint8_t cmd, const Writer& w) {
    if (!w.ok()) return std::nullopt;
    return ccu::pack_frame(cmd, w.bytes());
}

template <typename T, typename Fn>
std::optional<T> decode(uint8_t cmd, std::span<const uint8_t> frame, Fn fn) {
    const std::optional<View> view = view_frame(frame);
    if (!view || view->cmd != cmd) return std::nullopt;
    Reader reader(view->payload);
    T value = fn(reader);
    if (!reader.finished()) return std::nullopt;
    return value;
}

}  // namespace

namespace ccu {

bool cmd_is_known(uint8_t cmd) {
    switch (cmd) {
    case kCmdDeviceId:
    case kCmdPairingAccepted:
    case kCmdRegularTelemetry:
    case kCmdSurveyResult:
    case kCmdProjectionReady:
    case kCmdConnected:
    case kCmdBeginNewSession:
    case kCmdJoinSession:
    case kCmdEndSession:
    case kCmdBeginOriginSet:
    case kCmdBeginTagFinding:
    case kCmdPerformSurvey:
    case kCmdAbortSurvey:
    case kCmdPerformProjection:
    case kCmdProjectorPause:
    case kCmdProjectorPlay:
    case kCmdProjectionEnd:
    case kCmdSetLaserPower:
    case kCmdNudgeX:
    case kCmdNudgeY:
    case kCmdTagAligned:
        return true;
    default:
        return false;
    }
}

std::optional<std::vector<uint8_t>> pack_frame(uint8_t cmd, std::span<const uint8_t> payload) {
    if (!cmd_is_known(cmd)) return std::nullopt;
    if (payload.size() > 0xFFFF) return std::nullopt;
    std::vector<uint8_t> out(3 + payload.size());
    out[0] = cmd;
    const auto len = static_cast<uint16_t>(payload.size());
    out[1] = static_cast<uint8_t>(len & 0xFF);
    out[2] = static_cast<uint8_t>((len >> 8) & 0xFF);
    if (!payload.empty())
        std::memcpy(out.data() + 3, payload.data(), payload.size());
    return out;
}

std::optional<RawFrame> unpack_frame(std::span<const uint8_t> bytes) {
    const std::optional<View> view = view_frame(bytes);
    if (!view) return std::nullopt;
    RawFrame raw;
    raw.cmd = view->cmd;
    raw.payload.assign(view->payload.begin(), view->payload.end());
    return raw;
}

std::optional<std::vector<uint8_t>> pack_device_id(const DeviceId& msg) {
    Writer w;
    w.str(msg.device_id);
    w.str(msg.display_name);
    w.boolean(msg.session_active);
    w.str(msg.session_id);
    return seal(kCmdDeviceId, w);
}

std::optional<DeviceId> unpack_device_id(std::span<const uint8_t> frame) {
    return decode<DeviceId>(kCmdDeviceId, frame, [](Reader& r) {
        DeviceId msg;
        msg.device_id = r.str();
        msg.display_name = r.str();
        msg.session_active = r.boolean();
        msg.session_id = r.str();
        return msg;
    });
}

std::optional<std::vector<uint8_t>> pack_pairing_accepted(std::string_view device_id) {
    Writer w;
    w.str(device_id);
    return seal(kCmdPairingAccepted, w);
}

std::optional<std::string> unpack_pairing_accepted(std::span<const uint8_t> frame) {
    return decode<std::string>(kCmdPairingAccepted, frame, [](Reader& r) { return r.str(); });
}

std::optional<std::vector<uint8_t>> pack_regular_telemetry(const RegularTelemetry& msg) {
    Writer w;
    w.str(msg.device_id);
    w.str(msg.session_id);
    w.boolean(msg.origin_is_set);
    write_pose(w, msg.pose);
    w.f32(msg.effective_yaw);
    w.boolean(msg.moved);
    w.u8(static_cast<uint8_t>(msg.origin_set_status));
    w.u8(static_cast<uint8_t>(msg.tag_finding_status));
    w.u8(static_cast<uint8_t>(msg.survey_status));
    w.u8(static_cast<uint8_t>(msg.projector_status));
    w.boolean(msg.projector_running);
    w.boolean(msg.laser_on);
    w.u8(msg.laser_power);
    return seal(kCmdRegularTelemetry, w);
}

std::optional<RegularTelemetry> unpack_regular_telemetry(std::span<const uint8_t> frame) {
    return decode<RegularTelemetry>(kCmdRegularTelemetry, frame, [](Reader& r) {
        RegularTelemetry msg;
        msg.device_id = r.str();
        msg.session_id = r.str();
        msg.origin_is_set = r.boolean();
        msg.pose = read_pose(r);
        msg.effective_yaw = r.f32();
        msg.moved = r.boolean();
        msg.origin_set_status = static_cast<OriginSetStatus>(r.u8());
        msg.tag_finding_status = static_cast<TagFindingStatus>(r.u8());
        msg.survey_status = static_cast<SurveyStatus>(r.u8());
        msg.projector_status = static_cast<ProjectorStatus>(r.u8());
        msg.projector_running = r.boolean();
        msg.laser_on = r.boolean();
        msg.laser_power = r.u8();
        return msg;
    });
}

std::optional<std::vector<uint8_t>> pack_survey_result(const SurveyResult& msg) {
    Writer w;
    w.u16(msg.id);
    w.f32(msg.east_mm);
    w.f32(msg.north_mm);
    w.f32(msg.up_mm);
    w.f32(msg.corner_inner_angle_rad);
    w.f32(msg.corner_orientation_rad);
    w.boolean(msg.surveyed);
    w.boolean(msg.success);
    w.u8(static_cast<uint8_t>(msg.error_code));
    w.boolean(msg.survey_finished);
    auto frame = seal(kCmdSurveyResult, w);
    if (!frame || frame->size() != 3 + kSurveyResultBytes) return std::nullopt;
    return frame;
}

std::optional<SurveyResult> unpack_survey_result(std::span<const uint8_t> frame) {
    const std::optional<View> view = view_frame(frame);
    if (!view || view->cmd != kCmdSurveyResult) return std::nullopt;
    if (view->payload.size() != kSurveyResultBytes) return std::nullopt;
    Reader r(view->payload);
    SurveyResult msg;
    msg.id = r.u16();
    msg.east_mm = r.f32();
    msg.north_mm = r.f32();
    msg.up_mm = r.f32();
    msg.corner_inner_angle_rad = r.f32();
    msg.corner_orientation_rad = r.f32();
    msg.surveyed = r.boolean();
    msg.success = r.boolean();
    msg.error_code = static_cast<SurveyError>(r.u8());
    msg.survey_finished = r.boolean();
    if (!r.finished()) return std::nullopt;
    return msg;
}

std::optional<std::vector<uint8_t>> pack_projection_ready(const ProjectionReady& msg) {
    if (msg.nodes.size() > 0xFFFF || msg.errors.size() > 0xFF) return std::nullopt;
    const std::size_t bytes = 2 + msg.nodes.size() * kPlanNodeBytes + 1 + msg.errors.size() * 3;
    if (bytes > 0xFFFF) return std::nullopt;
    Writer w;
    w.u16(static_cast<uint16_t>(msg.nodes.size()));
    for (const PlanNode& node : msg.nodes) write_node(w, node);
    w.u8(static_cast<uint8_t>(msg.errors.size()));
    for (const NodeError& error : msg.errors) {
        w.u16(error.id);
        w.u8(error.error_code);
    }
    return seal(kCmdProjectionReady, w);
}

std::optional<ProjectionReady> unpack_projection_ready(std::span<const uint8_t> frame) {
    return decode<ProjectionReady>(kCmdProjectionReady, frame, [](Reader& r) {
        ProjectionReady msg;
        if (r.remaining() < 2) {
            r.fail();
            return msg;
        }
        const uint16_t count = r.u16();
        const std::size_t node_bytes = static_cast<std::size_t>(count) * kPlanNodeBytes;
        if (r.remaining() < node_bytes + 1) {
            r.fail();
            return msg;
        }
        msg.nodes.reserve(count);
        for (uint16_t i = 0; i < count; ++i) msg.nodes.push_back(read_node(r));
        const uint8_t error_count = r.u8();
        if (r.remaining() != static_cast<std::size_t>(error_count) * 3) {
            r.fail();
            return msg;
        }
        msg.errors.reserve(error_count);
        for (uint8_t i = 0; i < error_count; ++i) {
            NodeError error;
            error.id = r.u16();
            error.error_code = r.u8();
            msg.errors.push_back(error);
        }
        return msg;
    });
}

std::optional<std::vector<uint8_t>> pack_connected(std::string_view tablet_id) {
    Writer w;
    w.str(tablet_id);
    return seal(kCmdConnected, w);
}

std::optional<std::string> unpack_connected(std::span<const uint8_t> frame) {
    return decode<std::string>(kCmdConnected, frame, [](Reader& r) { return r.str(); });
}

std::optional<std::vector<uint8_t>> pack_empty(uint8_t cmd) {
    return pack_frame(cmd, {});
}

std::optional<EmptyPayload> unpack_empty(uint8_t cmd, std::span<const uint8_t> frame) {
    return decode<EmptyPayload>(cmd, frame, [](Reader&) { return EmptyPayload{}; });
}

std::optional<std::vector<uint8_t>> pack_begin_new_session() { return pack_empty(kCmdBeginNewSession); }
std::optional<EmptyPayload> unpack_begin_new_session(std::span<const uint8_t> frame) {
    return unpack_empty(kCmdBeginNewSession, frame);
}

std::optional<std::vector<uint8_t>> pack_session_id(uint8_t cmd, std::string_view session_id) {
    Writer w;
    w.str(session_id);
    return seal(cmd, w);
}

std::optional<std::string> unpack_session_id(uint8_t cmd, std::span<const uint8_t> frame) {
    return decode<std::string>(cmd, frame, [](Reader& r) { return r.str(); });
}

std::optional<std::vector<uint8_t>> pack_join_session(std::string_view session_id) {
    return pack_session_id(kCmdJoinSession, session_id);
}
std::optional<std::string> unpack_join_session(std::span<const uint8_t> frame) {
    return unpack_session_id(kCmdJoinSession, frame);
}

std::optional<std::vector<uint8_t>> pack_end_session(std::string_view session_id) {
    return pack_session_id(kCmdEndSession, session_id);
}
std::optional<std::string> unpack_end_session(std::span<const uint8_t> frame) {
    return unpack_session_id(kCmdEndSession, frame);
}

std::optional<std::vector<uint8_t>> pack_begin_origin_set() { return pack_empty(kCmdBeginOriginSet); }
std::optional<EmptyPayload> unpack_begin_origin_set(std::span<const uint8_t> frame) {
    return unpack_empty(kCmdBeginOriginSet, frame);
}

std::optional<std::vector<uint8_t>> pack_begin_tag_finding() { return pack_empty(kCmdBeginTagFinding); }
std::optional<EmptyPayload> unpack_begin_tag_finding(std::span<const uint8_t> frame) {
    return unpack_empty(kCmdBeginTagFinding, frame);
}

std::optional<std::vector<uint8_t>> pack_perform_survey(const std::vector<PlanNode>& nodes) {
    Writer w;
    write_nodes(w, nodes);
    return seal(kCmdPerformSurvey, w);
}

std::optional<std::vector<PlanNode>> unpack_perform_survey(std::span<const uint8_t> frame) {
    return decode<std::vector<PlanNode>>(kCmdPerformSurvey, frame, [](Reader& r) {
        return read_nodes(r);
    });
}

std::optional<std::vector<uint8_t>> pack_abort_survey() { return pack_empty(kCmdAbortSurvey); }
std::optional<EmptyPayload> unpack_abort_survey(std::span<const uint8_t> frame) {
    return unpack_empty(kCmdAbortSurvey, frame);
}

std::optional<std::vector<uint8_t>> pack_perform_projection(const std::vector<PlanNode>& nodes) {
    Writer w;
    write_nodes(w, nodes);
    return seal(kCmdPerformProjection, w);
}

std::optional<std::vector<PlanNode>> unpack_perform_projection(std::span<const uint8_t> frame) {
    return decode<std::vector<PlanNode>>(kCmdPerformProjection, frame, [](Reader& r) {
        return read_nodes(r);
    });
}

std::optional<std::vector<uint8_t>> pack_projector_pause() { return pack_empty(kCmdProjectorPause); }
std::optional<EmptyPayload> unpack_projector_pause(std::span<const uint8_t> frame) {
    return unpack_empty(kCmdProjectorPause, frame);
}

std::optional<std::vector<uint8_t>> pack_projector_play() { return pack_empty(kCmdProjectorPlay); }
std::optional<EmptyPayload> unpack_projector_play(std::span<const uint8_t> frame) {
    return unpack_empty(kCmdProjectorPlay, frame);
}

std::optional<std::vector<uint8_t>> pack_projection_end() { return pack_empty(kCmdProjectionEnd); }
std::optional<EmptyPayload> unpack_projection_end(std::span<const uint8_t> frame) {
    return unpack_empty(kCmdProjectionEnd, frame);
}

std::optional<std::vector<uint8_t>> pack_set_laser_power(uint8_t duty) {
    if (duty < kLaserDutyMin || duty > kLaserDutyMax) return std::nullopt;
    const uint8_t payload[] = {duty};
    return pack_frame(kCmdSetLaserPower, payload);
}

std::optional<uint8_t> unpack_set_laser_power(std::span<const uint8_t> frame) {
    return decode<uint8_t>(kCmdSetLaserPower, frame, [](Reader& r) {
        const uint8_t duty = r.u8();
        if (duty < kLaserDutyMin || duty > kLaserDutyMax) r.fail();
        return duty;
    });
}

std::optional<std::vector<uint8_t>> pack_nudge(uint8_t cmd, int16_t delta) {
    Writer w;
    w.i16(delta);
    return seal(cmd, w);
}

std::optional<int16_t> unpack_nudge(uint8_t cmd, std::span<const uint8_t> frame) {
    return decode<int16_t>(cmd, frame, [](Reader& r) { return r.i16(); });
}

std::optional<std::vector<uint8_t>> pack_nudge_x(int16_t dx) { return pack_nudge(kCmdNudgeX, dx); }
std::optional<int16_t> unpack_nudge_x(std::span<const uint8_t> frame) {
    return unpack_nudge(kCmdNudgeX, frame);
}

std::optional<std::vector<uint8_t>> pack_nudge_y(int16_t dy) { return pack_nudge(kCmdNudgeY, dy); }
std::optional<int16_t> unpack_nudge_y(std::span<const uint8_t> frame) {
    return unpack_nudge(kCmdNudgeY, frame);
}

std::optional<std::vector<uint8_t>> pack_tag_aligned(std::string_view tag_id) {
    Writer w;
    w.str(tag_id);
    return seal(kCmdTagAligned, w);
}

std::optional<std::string> unpack_tag_aligned(std::span<const uint8_t> frame) {
    return decode<std::string>(kCmdTagAligned, frame, [](Reader& r) { return r.str(); });
}

}  // namespace ccu
