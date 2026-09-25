#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

// Promo tablet link. Frame layout is docs/CMD_PROTOCOL.md.
// [cmd u8][length u16 LE][payload]. Strings are a u8 count (max 32) plus bytes.

namespace ccu {

inline constexpr uint8_t kCmdDeviceId = 0x01;
inline constexpr uint8_t kCmdPairingAccepted = 0x02;
inline constexpr uint8_t kCmdRegularTelemetry = 0x03;
inline constexpr uint8_t kCmdSurveyResult = 0x04;
inline constexpr uint8_t kCmdProjectionReady = 0x05;

inline constexpr uint8_t kCmdConnected = 0x10;

inline constexpr uint8_t kCmdBeginNewSession = 0x20;
inline constexpr uint8_t kCmdJoinSession = 0x21;
inline constexpr uint8_t kCmdEndSession = 0x22;
inline constexpr uint8_t kCmdBeginOriginSet = 0x23;
inline constexpr uint8_t kCmdBeginTagFinding = 0x24;
inline constexpr uint8_t kCmdPerformSurvey = 0x25;
inline constexpr uint8_t kCmdAbortSurvey = 0x26;
inline constexpr uint8_t kCmdPerformProjection = 0x27;
inline constexpr uint8_t kCmdProjectorPause = 0x28;
inline constexpr uint8_t kCmdProjectorPlay = 0x29;
inline constexpr uint8_t kCmdProjectionEnd = 0x2A;
inline constexpr uint8_t kCmdSetLaserPower = 0x2B;
inline constexpr uint8_t kCmdNudgeX = 0x2C;
inline constexpr uint8_t kCmdNudgeY = 0x2D;
inline constexpr uint8_t kCmdTagAligned = 0x2E;

inline constexpr uint8_t kStringMax = 32;
inline constexpr uint8_t kLaserDutyMin = 15;
inline constexpr uint8_t kLaserDutyMax = 99;
inline constexpr uint8_t kLaserDutyDefault = 30;

inline constexpr std::size_t kPoseBytes = 24;
inline constexpr std::size_t kPlanNodeBytes = 18;
inline constexpr std::size_t kSurveyResultBytes = 26;

enum class OriginSetStatus : uint8_t {
    Idle = 0,
    WaitingForTag = 1,
};

enum class TagFindingStatus : uint8_t {
    PoseKnown = 0,
    Computing = 1,
    WaitingForTag = 2,
};

enum class SurveyStatus : uint8_t {
    Idle = 0,
    Scanning = 1,
};

enum class ProjectorStatus : uint8_t {
    Idle = 0,
    ComputingPath = 1,
    Projecting = 2,
};

enum class SurveyError : uint8_t {
    None = 0,
    NotVisible = 1,
    OutOfFov = 2,
    Aborted = 3,
};

// Cartesian components are metres.
struct Pose {
    float east_m = 0.f;
    float north_m = 0.f;
    float up_m = 0.f;
    float roll_rad = 0.f;
    float pitch_rad = 0.f;
    float yaw_rad = 0.f;
};

struct PlanNode {
    uint16_t id = 0;
    float east_m = 0.f;
    float north_m = 0.f;
    float corner_inner_angle_rad = 0.f;
    float corner_orientation_rad = 0.f;
};

struct DeviceId {
    std::string device_id;
    std::string display_name;
    bool session_active = false;
    std::string session_id;
};

struct RegularTelemetry {
    std::string device_id;
    std::string session_id;
    bool origin_is_set = false;
    Pose pose{};
    float effective_yaw = 0.f;
    bool moved = false;
    OriginSetStatus origin_set_status = OriginSetStatus::Idle;
    TagFindingStatus tag_finding_status = TagFindingStatus::PoseKnown;
    SurveyStatus survey_status = SurveyStatus::Idle;
    ProjectorStatus projector_status = ProjectorStatus::Idle;
    bool projector_running = false;
    bool laser_on = false;
    uint8_t laser_power = kLaserDutyDefault;
};

struct SurveyResult {
    uint16_t id = 0;
    float east_m = 0.f;
    float north_m = 0.f;
    float up_m = 0.f;
    float corner_inner_angle_rad = 0.f;
    float corner_orientation_rad = 0.f;
    bool surveyed = false;
    bool success = false;
    SurveyError error_code = SurveyError::None;
    bool survey_finished = false;
};

struct NodeError {
    uint16_t id = 0;
    uint8_t error_code = 0;
};

struct ProjectionReady {
    std::vector<PlanNode> nodes;
    std::vector<NodeError> errors;
};

struct EmptyPayload {};

struct RawFrame {
    uint8_t cmd = 0;
    std::vector<uint8_t> payload;
};

bool cmd_is_known(uint8_t cmd);

std::optional<std::vector<uint8_t>> pack_frame(uint8_t cmd, std::span<const uint8_t> payload);
std::optional<RawFrame> unpack_frame(std::span<const uint8_t> bytes);

std::optional<std::vector<uint8_t>> pack_device_id(const DeviceId& msg);
std::optional<DeviceId> unpack_device_id(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_pairing_accepted(std::string_view device_id);
std::optional<std::string> unpack_pairing_accepted(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_regular_telemetry(const RegularTelemetry& msg);
std::optional<RegularTelemetry> unpack_regular_telemetry(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_survey_result(const SurveyResult& msg);
std::optional<SurveyResult> unpack_survey_result(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_projection_ready(const ProjectionReady& msg);
std::optional<ProjectionReady> unpack_projection_ready(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_connected(std::string_view tablet_id);
std::optional<std::string> unpack_connected(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_begin_new_session();
std::optional<EmptyPayload> unpack_begin_new_session(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_join_session(std::string_view session_id);
std::optional<std::string> unpack_join_session(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_end_session(std::string_view session_id);
std::optional<std::string> unpack_end_session(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_begin_origin_set();
std::optional<EmptyPayload> unpack_begin_origin_set(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_begin_tag_finding();
std::optional<EmptyPayload> unpack_begin_tag_finding(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_perform_survey(const std::vector<PlanNode>& nodes);
std::optional<std::vector<PlanNode>> unpack_perform_survey(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_abort_survey();
std::optional<EmptyPayload> unpack_abort_survey(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_perform_projection(const std::vector<PlanNode>& nodes);
std::optional<std::vector<PlanNode>> unpack_perform_projection(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_projector_pause();
std::optional<EmptyPayload> unpack_projector_pause(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_projector_play();
std::optional<EmptyPayload> unpack_projector_play(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_projection_end();
std::optional<EmptyPayload> unpack_projection_end(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_set_laser_power(uint8_t duty);
std::optional<uint8_t> unpack_set_laser_power(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_nudge_x(int16_t dx);
std::optional<int16_t> unpack_nudge_x(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_nudge_y(int16_t dy);
std::optional<int16_t> unpack_nudge_y(std::span<const uint8_t> frame);

std::optional<std::vector<uint8_t>> pack_tag_aligned(std::string_view tag_id);
std::optional<std::string> unpack_tag_aligned(std::span<const uint8_t> frame);

}  // namespace ccu
