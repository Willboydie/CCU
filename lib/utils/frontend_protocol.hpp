#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// FRONTEND_PROTOCOL.md — tablet ↔ marker, forwarded by the USB-C ESP-NOW dongle.
// WIFI_DONGLE_HOSTING.md — USB CDC 921600 8N1, raw frames, never cmd 0xE5.

inline constexpr int kFrontendBaud = 921600;
inline constexpr uint8_t kFrontendProtocolVersion = 1;
inline constexpr uint8_t kFrontendReservedPairingCmd = 0xE5;
inline constexpr uint16_t kFrontendMaxFrame = 1024;
inline constexpr uint16_t kFrontendMaxPayload = kFrontendMaxFrame - 3;
inline constexpr int kFrontendTelemetryHz = 10;

inline constexpr uint8_t kFeCmdFirmwareVersion = 0x01;
inline constexpr uint8_t kFeCmdTelemetry = 0x02;
inline constexpr uint8_t kFeCmdSurveyPoint = 0x03;
inline constexpr uint8_t kFeCmdProjectionStart = 0x04;

inline constexpr uint8_t kFeCmdSetLaserPower = 0x10;
inline constexpr uint8_t kFeCmdSetProjectionMode = 0x11;
inline constexpr uint8_t kFeCmdBeginTagSearch = 0x12;
inline constexpr uint8_t kFeCmdBeginSurvey = 0x13;
inline constexpr uint8_t kFeCmdBeginProjection = 0x14;
inline constexpr uint8_t kFeCmdTagFound = 0x15;
inline constexpr uint8_t kFeCmdNudgeBeamX = 0x16;
inline constexpr uint8_t kFeCmdNudgeBeamY = 0x17;
inline constexpr uint8_t kFeCmdSetOrigin = 0x18;
inline constexpr uint8_t kFeCmdRegisterNodes = 0x19;

inline constexpr uint8_t kFeConnLinked = 0;
inline constexpr uint8_t kFeConnSearching = 1;
inline constexpr uint8_t kFeConnLost = 2;

inline constexpr uint8_t kFeModeOff = 0;
inline constexpr uint8_t kFeModeSurvey = 1;
inline constexpr uint8_t kFeModeProject = 2;

inline constexpr uint16_t kFeTelemetryPayloadSize = 40;
inline constexpr uint16_t kFeSurveyPointPayloadSize = 27;
inline constexpr uint16_t kFePoseSize = 24;
inline constexpr uint16_t kFeNodeSize = 18;

inline constexpr float kFeTwoPi = 6.28318530717958647692f;

inline float wrap_2pi(float r) {
    r = std::fmod(r, kFeTwoPi);
    if (r < 0.0f)
        r += kFeTwoPi;
    return r;
}

#pragma pack(push, 1)
struct FrontendPose {
    float east_mm;
    float north_mm;
    float up_mm;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
};

struct FrontendNode {
    uint16_t id;
    float east_mm;
    float north_mm;
    float corner_inner_angle_rad;
    float corner_orientation_rad;
};

struct FrontendTelemetry {
    uint8_t healthOK;
    uint8_t moved;
    uint8_t originIsSet;
    uint8_t isSurveying;
    uint8_t surveyReady;
    uint8_t isProjecting;
    uint8_t laserOn;
    uint8_t connectionStatus;
    uint8_t projectionMode;
    uint8_t laserPower;
    uint8_t poseValid;
    uint8_t reserved;
    float effectiveYaw;
    FrontendPose lastKnownPose;
};

struct FrontendSurveyPoint {
    uint16_t nodeId;
    uint8_t success;
    FrontendPose pose;
};
#pragma pack(pop)

static_assert(sizeof(FrontendPose) == kFePoseSize);
static_assert(sizeof(FrontendNode) == kFeNodeSize);
static_assert(sizeof(FrontendTelemetry) == kFeTelemetryPayloadSize);
static_assert(sizeof(FrontendSurveyPoint) == kFeSurveyPointPayloadSize);

inline FrontendTelemetry make_default_telemetry() {
    FrontendTelemetry t{};
    t.healthOK = 1;
    t.connectionStatus = kFeConnLinked;
    t.laserPower = 50;
    return t;
}

inline std::optional<std::vector<uint8_t>> pack_frontend_frame(
    uint8_t cmd, std::span<const uint8_t> payload) {
    if (cmd == kFrontendReservedPairingCmd)
        return std::nullopt;
    if (payload.size() > kFrontendMaxPayload)
        return std::nullopt;

    std::vector<uint8_t> out(3 + payload.size());
    out[0] = cmd;
    const auto len = static_cast<uint16_t>(payload.size());
    out[1] = static_cast<uint8_t>(len & 0xFF);
    out[2] = static_cast<uint8_t>((len >> 8) & 0xFF);
    if (!payload.empty())
        std::memcpy(out.data() + 3, payload.data(), payload.size());
    return out;
}

inline void wrap_telemetry_angles(FrontendTelemetry& t) {
    t.effectiveYaw = wrap_2pi(t.effectiveYaw);
    t.lastKnownPose.roll_rad = wrap_2pi(t.lastKnownPose.roll_rad);
    t.lastKnownPose.pitch_rad = wrap_2pi(t.lastKnownPose.pitch_rad);
    t.lastKnownPose.yaw_rad = wrap_2pi(t.lastKnownPose.yaw_rad);
}

inline void clamp_telemetry(FrontendTelemetry& t) {
    auto as_bool = [](uint8_t v) -> uint8_t { return v ? 1 : 0; };
    t.healthOK = as_bool(t.healthOK);
    t.moved = as_bool(t.moved);
    t.originIsSet = as_bool(t.originIsSet);
    t.isSurveying = as_bool(t.isSurveying);
    t.surveyReady = as_bool(t.surveyReady);
    t.isProjecting = as_bool(t.isProjecting);
    t.laserOn = as_bool(t.laserOn);
    t.poseValid = as_bool(t.poseValid);
    if (t.connectionStatus > kFeConnLost)
        t.connectionStatus = kFeConnLost;
    if (t.projectionMode > kFeModeProject)
        t.projectionMode = kFeModeProject;
    if (t.laserPower > 100)
        t.laserPower = 100;
    wrap_telemetry_angles(t);
}

inline std::optional<std::vector<uint8_t>> pack_firmware_version(
    std::string_view firmware) {
    std::vector<uint8_t> payload(1 + firmware.size());
    payload[0] = kFrontendProtocolVersion;
    if (!firmware.empty())
        std::memcpy(payload.data() + 1, firmware.data(), firmware.size());
    return pack_frontend_frame(kFeCmdFirmwareVersion, payload);
}

inline std::optional<std::vector<uint8_t>> pack_telemetry(
    FrontendTelemetry t) {
    clamp_telemetry(t);
    return pack_frontend_frame(
        kFeCmdTelemetry,
        {reinterpret_cast<const uint8_t*>(&t), sizeof(t)});
}

inline std::optional<std::vector<uint8_t>> pack_survey_point(
    FrontendSurveyPoint p) {
    p.success = p.success ? 1 : 0;
    p.pose.roll_rad = wrap_2pi(p.pose.roll_rad);
    p.pose.pitch_rad = wrap_2pi(p.pose.pitch_rad);
    p.pose.yaw_rad = wrap_2pi(p.pose.yaw_rad);
    return pack_frontend_frame(
        kFeCmdSurveyPoint,
        {reinterpret_cast<const uint8_t*>(&p), sizeof(p)});
}

inline std::optional<std::vector<uint8_t>> pack_projection_start(
    std::span<const FrontendNode> nodes) {
    if (nodes.size() > (kFrontendMaxPayload - 2) / kFeNodeSize)
        return std::nullopt;
    const auto count = static_cast<uint16_t>(nodes.size());
    std::vector<uint8_t> payload(2 + static_cast<std::size_t>(count) * kFeNodeSize);
    payload[0] = static_cast<uint8_t>(count & 0xFF);
    payload[1] = static_cast<uint8_t>((count >> 8) & 0xFF);
    if (count > 0)
        std::memcpy(payload.data() + 2, nodes.data(),
                    static_cast<std::size_t>(count) * kFeNodeSize);
    return pack_frontend_frame(kFeCmdProjectionStart, payload);
}

inline bool unpack_telemetry(std::span<const uint8_t> payload,
                             FrontendTelemetry& out) {
    if (payload.size() < sizeof(FrontendTelemetry))
        return false;
    std::memcpy(&out, payload.data(), sizeof(out));
    return true;
}

inline bool unpack_pose(std::span<const uint8_t> payload, FrontendPose& out) {
    if (payload.size() < sizeof(FrontendPose))
        return false;
    std::memcpy(&out, payload.data(), sizeof(out));
    return true;
}

inline bool unpack_survey_point(std::span<const uint8_t> payload,
                               FrontendSurveyPoint& out) {
    if (payload.size() < sizeof(FrontendSurveyPoint))
        return false;
    std::memcpy(&out, payload.data(), sizeof(out));
    return true;
}

inline const char* frontend_cmd_name(uint8_t cmd) {
    switch (cmd) {
        case kFeCmdFirmwareVersion: return "FIRMWARE_VERSION";
        case kFeCmdTelemetry: return "TELEMETRY";
        case kFeCmdSurveyPoint: return "SURVEY_POINT";
        case kFeCmdProjectionStart: return "PROJECTION_START";
        case kFeCmdSetLaserPower: return "SET_LASER_POWER";
        case kFeCmdSetProjectionMode: return "SET_PROJECTION_MODE";
        case kFeCmdBeginTagSearch: return "BEGIN_TAG_SEARCH";
        case kFeCmdBeginSurvey: return "BEGIN_SURVEY";
        case kFeCmdBeginProjection: return "BEGIN_PROJECTION";
        case kFeCmdTagFound: return "TAG_FOUND";
        case kFeCmdNudgeBeamX: return "NUDGE_BEAM_X";
        case kFeCmdNudgeBeamY: return "NUDGE_BEAM_Y";
        case kFeCmdSetOrigin: return "SET_ORIGIN";
        case kFeCmdRegisterNodes: return "REGISTER_NODES";
        default: return nullptr;
    }
}

inline const char* connection_status_name(uint8_t v) {
    switch (v) {
        case kFeConnLinked: return "Linked";
        case kFeConnSearching: return "Searching";
        case kFeConnLost: return "Lost";
        default: return "?";
    }
}

inline const char* projection_mode_name(uint8_t v) {
    switch (v) {
        case kFeModeOff: return "Off";
        case kFeModeSurvey: return "Survey";
        case kFeModeProject: return "Project";
        default: return "?";
    }
}

inline std::string format_pose(const FrontendPose& p) {
    char buf[192];
    std::snprintf(buf, sizeof(buf),
                  "e=%.1f n=%.1f u=%.1f roll=%.3f pitch=%.3f yaw=%.3f",
                  static_cast<double>(p.east_mm),
                  static_cast<double>(p.north_mm),
                  static_cast<double>(p.up_mm),
                  static_cast<double>(p.roll_rad),
                  static_cast<double>(p.pitch_rad),
                  static_cast<double>(p.yaw_rad));
    return buf;
}

inline std::string hex_preview(std::span<const uint8_t> bytes, std::size_t max_n = 24) {
    std::string s;
    const std::size_t n = bytes.size() < max_n ? bytes.size() : max_n;
    s.reserve(n * 3 + 4);
    for (std::size_t i = 0; i < n; ++i) {
        char b[4];
        std::snprintf(b, sizeof(b), "%02X ", bytes[i]);
        s += b;
    }
    if (bytes.size() > max_n)
        s += "...";
    return s;
}

inline bool parse_u16_list(std::span<const uint8_t> payload,
                           std::vector<uint16_t>& ids) {
    if (payload.size() < 2)
        return false;
    uint16_t count = 0;
    std::memcpy(&count, payload.data(), 2);
    const std::size_t want = 2u + static_cast<std::size_t>(count) * 2u;
    if (payload.size() != want)
        return false;
    ids.resize(count);
    if (count > 0)
        std::memcpy(ids.data(), payload.data() + 2, static_cast<std::size_t>(count) * 2u);
    return true;
}

inline bool parse_register_nodes(std::span<const uint8_t> payload,
                                std::vector<FrontendNode>& nodes,
                                uint8_t* flags_out = nullptr) {
    auto try_at = [&](std::size_t off) {
        if (payload.size() < off + 2)
            return false;
        uint16_t count = 0;
        std::memcpy(&count, payload.data() + off, 2);
        const std::size_t want =
            off + 2u + static_cast<std::size_t>(count) * kFeNodeSize;
        if (payload.size() != want)
            return false;
        nodes.resize(count);
        if (count > 0)
            std::memcpy(nodes.data(), payload.data() + off + 2,
                        static_cast<std::size_t>(count) * kFeNodeSize);
        return true;
    };

    if (try_at(0)) {
        if (flags_out)
            *flags_out = 0;
        return true;
    }
    if (payload.size() >= 3 && try_at(1)) {
        if (flags_out)
            *flags_out = payload[0];
        return true;
    }
    return false;
}

inline std::string format_id_list(const std::vector<uint16_t>& ids) {
    std::string s = "[";
    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (i)
            s += ", ";
        s += std::to_string(ids[i]);
        if (i >= 31 && ids.size() > 32) {
            s += ", ...";
            break;
        }
    }
    s += "]";
    return s;
}

inline std::string format_frontend_rx(uint8_t cmd, std::span<const uint8_t> payload) {
    const char* name = frontend_cmd_name(cmd);
    char head[64];
    if (name)
        std::snprintf(head, sizeof(head), "%s", name);
    else
        std::snprintf(head, sizeof(head), "cmd 0x%02X", cmd);

    auto bad = [&](const char* why) {
        std::string s = std::string(head) + "  " + why;
        s += "  len=" + std::to_string(payload.size());
        if (!payload.empty())
            s += "  " + hex_preview(payload);
        return s;
    };

    switch (cmd) {
        case kFeCmdSetLaserPower: {
            if (payload.size() != 1)
                return bad("bad payload");
            return std::string(head) + "  duty=" + std::to_string(payload[0]);
        }
        case kFeCmdSetProjectionMode: {
            if (payload.size() != 1)
                return bad("bad payload");
            return std::string(head) + "  " + std::string(projection_mode_name(payload[0])) +
                   " (" + std::to_string(payload[0]) + ")";
        }
        case kFeCmdBeginTagSearch: {
            if (!payload.empty())
                return bad("expected empty payload");
            return head;
        }
        case kFeCmdBeginSurvey:
        case kFeCmdBeginProjection: {
            std::vector<uint16_t> ids;
            if (!parse_u16_list(payload, ids))
                return bad("bad id list");
            return std::string(head) + "  count=" + std::to_string(ids.size()) +
                   "  ids=" + format_id_list(ids);
        }
        case kFeCmdTagFound: {
            if (payload.size() != 2)
                return bad("bad payload");
            uint16_t tag = 0;
            std::memcpy(&tag, payload.data(), 2);
            return std::string(head) + "  tagID=" + std::to_string(tag);
        }
        case kFeCmdNudgeBeamX:
        case kFeCmdNudgeBeamY: {
            if (payload.size() != 4)
                return bad("bad payload");
            float d = 0;
            std::memcpy(&d, payload.data(), 4);
            char buf[96];
            std::snprintf(buf, sizeof(buf), "%s  %s=%.4f deg",
                          head, cmd == kFeCmdNudgeBeamX ? "dx" : "dy",
                          static_cast<double>(d));
            return buf;
        }
        case kFeCmdSetOrigin: {
            FrontendPose pose{};
            if (!unpack_pose(payload, pose) || payload.size() != sizeof(FrontendPose))
                return bad("bad pose");
            return std::string(head) + "  " + format_pose(pose);
        }
        case kFeCmdRegisterNodes: {
            std::vector<FrontendNode> nodes;
            uint8_t flags = 0;
            if (!parse_register_nodes(payload, nodes, &flags))
                return bad("bad node table");
            std::string s = std::string(head) + "  count=" + std::to_string(nodes.size());
            if (payload.size() >= 3 && (payload.size() - 3) % kFeNodeSize == 0 &&
                payload.size() != 2u + nodes.size() * kFeNodeSize)
                s += "  flags=" + std::to_string(flags);
            const std::size_t show = nodes.size() < 8 ? nodes.size() : 8;
            for (std::size_t i = 0; i < show; ++i) {
                const FrontendNode& n = nodes[i];
                char line[160];
                std::snprintf(line, sizeof(line),
                              " | id=%u e=%.1f n=%.1f inner=%.3f orient=%.3f",
                              static_cast<unsigned>(n.id),
                              static_cast<double>(n.east_mm),
                              static_cast<double>(n.north_mm),
                              static_cast<double>(n.corner_inner_angle_rad),
                              static_cast<double>(n.corner_orientation_rad));
                s += line;
            }
            if (nodes.size() > show)
                s += " | ...";
            return s;
        }
        default:
            return std::string(head) + "  len=" + std::to_string(payload.size()) +
                   (payload.empty() ? "" : "  " + hex_preview(payload));
    }
}

inline std::string format_firmware_tx(std::string_view firmware) {
    std::string s = "TX FIRMWARE_VERSION  protocol=1  \"";
    s.append(firmware);
    s += "\"";
    return s;
}

inline std::string format_survey_tx(const FrontendSurveyPoint& p) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "TX SURVEY_POINT  id=%u  success=%u  %s",
                  static_cast<unsigned>(p.nodeId),
                  static_cast<unsigned>(p.success ? 1 : 0),
                  format_pose(p.pose).c_str());
    return buf;
}

inline std::string format_projection_tx(std::span<const FrontendNode> nodes) {
    return "TX PROJECTION_START  count=" + std::to_string(nodes.size());
}
