#include "protocol.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>
#include <vector>

namespace {

int g_fails = 0;

void check(bool cond, const char* what) {
    if (cond) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_fails;
}

void check_eq_u8(uint8_t got, uint8_t want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %u want %u)\n", what, got, want);
    ++g_fails;
}

void check_eq_u16(uint16_t got, uint16_t want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %u want %u)\n", what, got, want);
    ++g_fails;
}

void check_eq_i16(int16_t got, int16_t want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %d want %d)\n", what, got, want);
    ++g_fails;
}

void check_eq_f(float got, float want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

void check_eq_str(const std::string& got, const char* want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got \"%s\" want \"%s\")\n", what, got.c_str(), want);
    ++g_fails;
}

void check_bytes(const std::vector<uint8_t>& got, const uint8_t* want, std::size_t n, const char* what) {
    if (got.size() == n && std::memcmp(got.data(), want, n) == 0) return;
    std::fprintf(stderr, "FAIL: %s\n  got (%zu):", what, got.size());
    for (uint8_t b : got) std::fprintf(stderr, " %02X", b);
    std::fprintf(stderr, "\n  want (%zu):", n);
    for (std::size_t i = 0; i < n; ++i) std::fprintf(stderr, " %02X", want[i]);
    std::fprintf(stderr, "\n");
    ++g_fails;
}

void check_node(const ccu::PlanNode& node, uint16_t id, float east, float north,
                float inner, float orient, const char* what) {
    check_eq_u16(node.id, id, what);
    check_eq_f(node.east_m, east, what);
    check_eq_f(node.north_m, north, what);
    check_eq_f(node.corner_inner_angle_rad, inner, what);
    check_eq_f(node.corner_orientation_rad, orient, what);
}

// 1.0, 2.0, 0.5, 0 as little-endian binary32. Shared by survey and plan fixtures.
constexpr uint8_t kNode1[] = {
    0x01, 0x00,
    0x00, 0x00, 0x80, 0x3F,
    0x00, 0x00, 0x00, 0x40,
    0x00, 0x00, 0x00, 0x3F,
    0x00, 0x00, 0x00, 0x00,
};

void test_catalog_and_reserved() {
    check(ccu::kPoseBytes == 24, "pose is 24 bytes");
    check(ccu::kPlanNodeBytes == 18, "plan node is 18 bytes");
    check(ccu::kSurveyResultBytes == 26, "survey result is 26 bytes");
    check(sizeof(kNode1) == ccu::kPlanNodeBytes, "fixture node is 18 bytes");

    const uint8_t reserved[] = {0xE4, 0xE5, 0xE6, 0x00, 0x06, 0x11, 0x2F};
    for (uint8_t cmd : reserved) {
        check(!ccu::cmd_is_known(cmd), "reserved or unknown cmd is not catalogued");
        check(!ccu::pack_frame(cmd, {}).has_value(), "reserved or unknown cmd pack refused");
        const uint8_t frame[] = {cmd, 0x00, 0x00};
        check(!ccu::unpack_frame(frame).has_value(), "reserved or unknown cmd unpack refused");
    }

    const uint8_t e5_payload[] = {0xE5, 0x01, 0x00, 0x00};
    check(!ccu::unpack_frame(e5_payload).has_value(), "0xE5 with a payload is refused");
}

void test_device_id() {
    // deviceId "sm-ccu", displayName "Smart Marker", sessionActive 1, sessionId "abcd1234"
    const uint8_t want[] = {
        0x01, 0x1E, 0x00,
        0x06, 's', 'm', '-', 'c', 'c', 'u',
        0x0C, 'S', 'm', 'a', 'r', 't', ' ', 'M', 'a', 'r', 'k', 'e', 'r',
        0x01,
        0x08, 'a', 'b', 'c', 'd', '1', '2', '3', '4',
    };
    ccu::DeviceId msg;
    msg.device_id = "sm-ccu";
    msg.display_name = "Smart Marker";
    msg.session_active = true;
    msg.session_id = "abcd1234";
    auto packed = ccu::pack_device_id(msg);
    check(packed.has_value(), "DEVICE_ID packs");
    if (packed) check_bytes(*packed, want, sizeof(want), "DEVICE_ID bytes");

    auto unpacked = ccu::unpack_device_id(want);
    check(unpacked.has_value(), "DEVICE_ID unpacks");
    if (!unpacked) return;
    check_eq_str(unpacked->device_id, "sm-ccu", "DEVICE_ID deviceId");
    check_eq_str(unpacked->display_name, "Smart Marker", "DEVICE_ID displayName");
    check(unpacked->session_active, "DEVICE_ID sessionActive");
    check_eq_str(unpacked->session_id, "abcd1234", "DEVICE_ID sessionId");

    // Any non-zero bool byte is true. Repack canonicalises it to 1.
    const uint8_t nonzero[] = {0x01, 0x04, 0x00, 0x00, 0x00, 0x07, 0x00};
    auto loose = ccu::unpack_device_id(nonzero);
    check(loose.has_value() && loose->session_active, "non-zero bool is true");
    check(loose.has_value() && loose->device_id.empty() && loose->session_id.empty(),
          "empty strings around a loose bool");
}

void test_pairing_accepted() {
    const uint8_t want[] = {0x02, 0x07, 0x00, 0x06, 's', 'm', '-', 'c', 'c', 'u'};
    auto packed = ccu::pack_pairing_accepted("sm-ccu");
    check(packed.has_value(), "PAIRING_ACCEPTED packs");
    if (packed) check_bytes(*packed, want, sizeof(want), "PAIRING_ACCEPTED bytes");
    auto unpacked = ccu::unpack_pairing_accepted(want);
    check(unpacked.has_value() && *unpacked == "sm-ccu", "PAIRING_ACCEPTED deviceId");
}

void test_regular_telemetry() {
    // Pose 1,2,3,4,5,6. effectiveYaw 0.5. Status bytes are the enum values.
    const uint8_t want[] = {
        0x03, 0x35, 0x00,
        0x06, 's', 'm', '-', 'c', 'c', 'u',
        0x08, 'a', 'b', 'c', 'd', '1', '2', '3', '4',
        0x01,
        0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x00, 0x40,
        0x00, 0x00, 0x40, 0x40,
        0x00, 0x00, 0x80, 0x40,
        0x00, 0x00, 0xA0, 0x40,
        0x00, 0x00, 0xC0, 0x40,
        0x00, 0x00, 0x00, 0x3F,
        0x01,
        0x00,
        0x00,
        0x01,
        0x02,
        0x01,
        0x00,
        0x1E,
    };
    check(sizeof(want) == 56, "telemetry fixture is 3 + 53 bytes");

    ccu::RegularTelemetry msg;
    msg.device_id = "sm-ccu";
    msg.session_id = "abcd1234";
    msg.origin_is_set = true;
    msg.pose = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
    msg.effective_yaw = 0.5f;
    msg.moved = true;
    msg.origin_set_status = ccu::OriginSetStatus::Idle;
    msg.tag_finding_status = ccu::TagFindingStatus::PoseKnown;
    msg.survey_status = ccu::SurveyStatus::Scanning;
    msg.projector_status = ccu::ProjectorStatus::Projecting;
    msg.projector_running = true;
    msg.laser_on = false;
    msg.laser_power = 30;

    auto packed = ccu::pack_regular_telemetry(msg);
    check(packed.has_value(), "REGULAR_TELEMETRY packs");
    if (packed) check_bytes(*packed, want, sizeof(want), "REGULAR_TELEMETRY bytes");

    auto unpacked = ccu::unpack_regular_telemetry(want);
    check(unpacked.has_value(), "REGULAR_TELEMETRY unpacks");
    if (!unpacked) return;
    check_eq_str(unpacked->device_id, "sm-ccu", "telemetry deviceId");
    check_eq_str(unpacked->session_id, "abcd1234", "telemetry sessionId");
    check(unpacked->origin_is_set, "telemetry originIsSet");
    check_eq_f(unpacked->pose.east_m, 1.f, "pose east");
    check_eq_f(unpacked->pose.north_m, 2.f, "pose north");
    check_eq_f(unpacked->pose.up_m, 3.f, "pose up");
    check_eq_f(unpacked->pose.roll_rad, 4.f, "pose roll");
    check_eq_f(unpacked->pose.pitch_rad, 5.f, "pose pitch");
    check_eq_f(unpacked->pose.yaw_rad, 6.f, "pose yaw");
    check_eq_f(unpacked->effective_yaw, 0.5f, "effectiveYaw");
    check(unpacked->moved, "telemetry moved");
    check(unpacked->origin_set_status == ccu::OriginSetStatus::Idle, "originSetStatus");
    check(unpacked->tag_finding_status == ccu::TagFindingStatus::PoseKnown, "tagFindingStatus");
    check(unpacked->survey_status == ccu::SurveyStatus::Scanning, "surveyStatus");
    check(unpacked->projector_status == ccu::ProjectorStatus::Projecting, "projectorStatus");
    check(unpacked->projector_running, "projectorRunning");
    check(!unpacked->laser_on, "laserOn");
    check_eq_u8(unpacked->laser_power, 30, "telemetry laserPower");

    // Duty range is enforced on SET_LASER_POWER, not on the telemetry field.
    uint8_t reported[sizeof(want)];
    std::memcpy(reported, want, sizeof(want));
    reported[sizeof(want) - 1] = 100;
    auto wide = ccu::unpack_regular_telemetry(reported);
    check(wide.has_value() && wide->laser_power == 100, "telemetry laserPower is not range-checked");
}

void test_survey_result() {
    const uint8_t ok[] = {
        0x04, 0x1A, 0x00,
        0x01, 0x00,
        0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x00, 0x40,
        0x00, 0x00, 0x40, 0x40,
        0x00, 0x00, 0x00, 0x3F,
        0x00, 0x00, 0x00, 0x00,
        0x01, 0x01, 0x00, 0x01,
    };
    check(sizeof(ok) == 3 + ccu::kSurveyResultBytes, "success survey fixture size");

    ccu::SurveyResult msg;
    msg.id = 1;
    msg.east_m = 1.f;
    msg.north_m = 2.f;
    msg.up_m = 3.f;
    msg.corner_inner_angle_rad = 0.5f;
    msg.corner_orientation_rad = 0.f;
    msg.surveyed = true;
    msg.success = true;
    msg.error_code = ccu::SurveyError::None;
    msg.survey_finished = true;
    auto packed = ccu::pack_survey_result(msg);
    check(packed.has_value(), "SURVEY_RESULT packs");
    if (packed) check_bytes(*packed, ok, sizeof(ok), "SURVEY_RESULT bytes");

    auto unpacked = ccu::unpack_survey_result(ok);
    check(unpacked.has_value(), "SURVEY_RESULT unpacks");
    if (unpacked) {
        check_eq_u16(unpacked->id, 1, "survey id");
        check_eq_f(unpacked->up_m, 3.f, "survey up");
        check(unpacked->surveyed && unpacked->success && unpacked->survey_finished, "survey flags");
        check(unpacked->error_code == ccu::SurveyError::None, "survey error none");
    }

    const uint8_t failed[] = {
        0x04, 0x1A, 0x00,
        0x07, 0x00,
        0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x00, 0x40,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x02, 0x00,
    };
    auto bad = ccu::unpack_survey_result(failed);
    check(bad.has_value(), "failed SURVEY_RESULT unpacks");
    if (bad) {
        check_eq_u16(bad->id, 7, "failed survey id");
        check_eq_f(bad->up_m, 0.f, "failed survey up is 0");
        check(!bad->surveyed && !bad->success && !bad->survey_finished, "failed survey flags");
        check(bad->error_code == ccu::SurveyError::OutOfFov, "survey OUT_OF_FOV");
    }

    uint8_t short_frame[sizeof(ok) - 1];
    std::memcpy(short_frame, ok, sizeof(short_frame));
    short_frame[1] = static_cast<uint8_t>(sizeof(short_frame) - 3);
    check(!ccu::unpack_survey_result(short_frame).has_value(), "short SURVEY_RESULT dropped");

    uint8_t long_frame[sizeof(ok) + 1];
    std::memcpy(long_frame, ok, sizeof(ok));
    long_frame[1] = static_cast<uint8_t>(ccu::kSurveyResultBytes + 1);
    long_frame[sizeof(ok)] = 0x00;
    check(!ccu::unpack_survey_result(long_frame).has_value(), "long SURVEY_RESULT dropped");
}

void test_projection_ready() {
    const uint8_t want[] = {
        0x05, 0x18, 0x00,
        0x01, 0x00,
        0x01, 0x00,
        0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x00, 0x40,
        0x00, 0x00, 0x00, 0x3F,
        0x00, 0x00, 0x00, 0x00,
        0x01,
        0x02, 0x00,
        0x04,
    };
    check(sizeof(want) == 3 + 24, "PROJECTION_READY fixture size");

    ccu::ProjectionReady msg;
    msg.nodes.push_back({1, 1.f, 2.f, 0.5f, 0.f});
    msg.errors.push_back({2, 4});
    auto packed = ccu::pack_projection_ready(msg);
    check(packed.has_value(), "PROJECTION_READY packs");
    if (packed) check_bytes(*packed, want, sizeof(want), "PROJECTION_READY bytes");

    auto unpacked = ccu::unpack_projection_ready(want);
    check(unpacked.has_value(), "PROJECTION_READY unpacks");
    if (unpacked) {
        check(unpacked->nodes.size() == 1, "one drawn node");
        if (!unpacked->nodes.empty())
            check_node(unpacked->nodes[0], 1, 1.f, 2.f, 0.5f, 0.f, "drawn node");
        check(unpacked->errors.size() == 1, "one omitted node");
        if (!unpacked->errors.empty()) {
            check_eq_u16(unpacked->errors[0].id, 2, "omitted id");
            check_eq_u8(unpacked->errors[0].error_code, 4, "NOT_SURVEYED");
        }
    }

    const uint8_t empty[] = {0x05, 0x03, 0x00, 0x00, 0x00, 0x00};
    auto packed_empty = ccu::pack_projection_ready({});
    check(packed_empty.has_value(), "empty PROJECTION_READY packs");
    if (packed_empty) check_bytes(*packed_empty, empty, sizeof(empty), "empty PROJECTION_READY bytes");
    auto unpacked_empty = ccu::unpack_projection_ready(empty);
    check(unpacked_empty.has_value() && unpacked_empty->nodes.empty() && unpacked_empty->errors.empty(),
          "empty PROJECTION_READY unpacks");

    std::vector<ccu::NodeError> too_many(256);
    check(!ccu::pack_projection_ready({{}, too_many}).has_value(), "errorCount above 255 dropped");
}

void test_connected_and_strings() {
    const uint8_t want[] = {0x10, 0x04, 0x00, 0x03, 'p', 'a', 'd'};
    auto packed = ccu::pack_connected("pad");
    check(packed.has_value(), "CONNECTED packs");
    if (packed) check_bytes(*packed, want, sizeof(want), "CONNECTED bytes");
    auto unpacked = ccu::unpack_connected(want);
    check(unpacked.has_value() && *unpacked == "pad", "CONNECTED tabletId");

    const std::string limit(32, 'a');
    const std::string over(33, 'a');
    auto packed_limit = ccu::pack_connected(limit);
    check(packed_limit.has_value(), "32-byte string packs");
    if (packed_limit) {
        check_eq_u8((*packed_limit)[3], 32, "length byte is 32");
        auto round = ccu::unpack_connected(*packed_limit);
        check(round.has_value() && *round == limit, "32-byte string unpacks");
    }
    check(!ccu::pack_connected(over).has_value(), "33-byte string pack dropped");

    uint8_t over_frame[3 + 1 + 33];
    over_frame[0] = 0x10;
    over_frame[1] = 34;
    over_frame[2] = 0x00;
    over_frame[3] = 33;
    std::memset(over_frame + 4, 'x', 33);
    check(!ccu::unpack_connected(over_frame).has_value(), "33-byte string unpack dropped");

    const uint8_t overrun[] = {0x10, 0x04, 0x00, 0x0A, 'a', 'b', 'c'};
    check(!ccu::unpack_connected(overrun).has_value(), "string longer than payload dropped");

    const uint8_t trailing[] = {0x10, 0x05, 0x00, 0x03, 'p', 'a', 'd', 0x00};
    check(!ccu::unpack_connected(trailing).has_value(), "trailing payload byte dropped");

    const uint8_t truncated[] = {0x10, 0x04, 0x00, 0x03, 'p'};
    check(!ccu::unpack_frame(truncated).has_value(), "truncated frame dropped");
}

void test_empty_commands() {
    struct Case {
        uint8_t cmd;
        const char* name;
        std::optional<std::vector<uint8_t>> (*pack)();
        std::optional<ccu::EmptyPayload> (*unpack)(std::span<const uint8_t>);
    };
    const Case cases[] = {
        {ccu::kCmdBeginNewSession, "BEGIN_NEW_SESSION", ccu::pack_begin_new_session, ccu::unpack_begin_new_session},
        {ccu::kCmdBeginOriginSet, "BEGIN_ORIGIN_SET", ccu::pack_begin_origin_set, ccu::unpack_begin_origin_set},
        {ccu::kCmdBeginTagFinding, "BEGIN_TAG_FINDING", ccu::pack_begin_tag_finding, ccu::unpack_begin_tag_finding},
        {ccu::kCmdAbortSurvey, "ABORT_SURVEY", ccu::pack_abort_survey, ccu::unpack_abort_survey},
        {ccu::kCmdProjectorPause, "PROJECTOR_PAUSE", ccu::pack_projector_pause, ccu::unpack_projector_pause},
        {ccu::kCmdProjectorPlay, "PROJECTOR_PLAY", ccu::pack_projector_play, ccu::unpack_projector_play},
        {ccu::kCmdProjectionEnd, "PROJECTION_END", ccu::pack_projection_end, ccu::unpack_projection_end},
    };
    for (const Case& c : cases) {
        const uint8_t want[] = {c.cmd, 0x00, 0x00};
        auto packed = c.pack();
        check(packed.has_value(), c.name);
        if (packed) check_bytes(*packed, want, sizeof(want), c.name);
        check(c.unpack(want).has_value(), c.name);
        const uint8_t extra[] = {c.cmd, 0x01, 0x00, 0x00};
        check(!c.unpack(extra).has_value(), c.name);
    }
}

void test_session_ids() {
    const uint8_t join[] = {0x21, 0x09, 0x00, 0x08, 'a', 'b', 'c', 'd', '1', '2', '3', '4'};
    const uint8_t end[] = {0x22, 0x09, 0x00, 0x08, 'a', 'b', 'c', 'd', '1', '2', '3', '4'};
    auto packed_join = ccu::pack_join_session("abcd1234");
    auto packed_end = ccu::pack_end_session("abcd1234");
    check(packed_join.has_value(), "JOIN_SESSION packs");
    check(packed_end.has_value(), "END_SESSION packs");
    if (packed_join) check_bytes(*packed_join, join, sizeof(join), "JOIN_SESSION bytes");
    if (packed_end) check_bytes(*packed_end, end, sizeof(end), "END_SESSION bytes");
    auto unpacked_join = ccu::unpack_join_session(join);
    auto unpacked_end = ccu::unpack_end_session(end);
    check(unpacked_join.has_value() && *unpacked_join == "abcd1234", "JOIN_SESSION id");
    check(unpacked_end.has_value() && *unpacked_end == "abcd1234", "END_SESSION id");
    check(!ccu::unpack_join_session(end).has_value(), "JOIN_SESSION rejects END_SESSION bytes");
}

void test_plan_commands() {
    const uint8_t survey[] = {
        0x25, 0x14, 0x00,
        0x01, 0x00,
        0x01, 0x00,
        0x00, 0x00, 0x80, 0x3F,
        0x00, 0x00, 0x00, 0x40,
        0x00, 0x00, 0x00, 0x3F,
        0x00, 0x00, 0x00, 0x00,
    };
    uint8_t projection[sizeof(survey)];
    std::memcpy(projection, survey, sizeof(survey));
    projection[0] = 0x27;

    const std::vector<ccu::PlanNode> nodes{{1, 1.f, 2.f, 0.5f, 0.f}};
    auto packed_survey = ccu::pack_perform_survey(nodes);
    auto packed_projection = ccu::pack_perform_projection(nodes);
    check(packed_survey.has_value(), "PERFORM_SURVEY packs");
    check(packed_projection.has_value(), "PERFORM_PROJECTION packs");
    if (packed_survey) check_bytes(*packed_survey, survey, sizeof(survey), "PERFORM_SURVEY bytes");
    if (packed_projection)
        check_bytes(*packed_projection, projection, sizeof(projection), "PERFORM_PROJECTION bytes");

    auto unpacked_survey = ccu::unpack_perform_survey(survey);
    check(unpacked_survey.has_value() && unpacked_survey->size() == 1, "PERFORM_SURVEY count");
    if (unpacked_survey && !unpacked_survey->empty())
        check_node((*unpacked_survey)[0], 1, 1.f, 2.f, 0.5f, 0.f, "survey node");

    const uint8_t empty_survey[] = {0x25, 0x02, 0x00, 0x00, 0x00};
    auto packed_empty = ccu::pack_perform_survey({});
    check(packed_empty.has_value(), "empty PERFORM_SURVEY packs");
    if (packed_empty) check_bytes(*packed_empty, empty_survey, sizeof(empty_survey), "empty PERFORM_SURVEY");

    const uint8_t short_count[] = {
        0x25, 0x04, 0x00,
        0x02, 0x00,
        0x01, 0x00,
    };
    check(!ccu::unpack_perform_survey(short_count).has_value(), "survey count past payload dropped");
}

void test_laser_power() {
    const uint8_t duty30[] = {0x2B, 0x01, 0x00, 30};
    const uint8_t duty15[] = {0x2B, 0x01, 0x00, 15};
    const uint8_t duty99[] = {0x2B, 0x01, 0x00, 99};
    auto packed = ccu::pack_set_laser_power(30);
    check(packed.has_value(), "SET_LASER_POWER 30 packs");
    if (packed) check_bytes(*packed, duty30, sizeof(duty30), "SET_LASER_POWER bytes");
    auto unpacked = ccu::unpack_set_laser_power(duty30);
    check(unpacked.has_value() && *unpacked == 30, "SET_LASER_POWER 30");

    check(ccu::pack_set_laser_power(15).has_value(), "duty 15 packs");
    check(ccu::pack_set_laser_power(99).has_value(), "duty 99 packs");
    check(ccu::unpack_set_laser_power(duty15).has_value(), "duty 15 unpacks");
    check(ccu::unpack_set_laser_power(duty99).has_value(), "duty 99 unpacks");

    check(!ccu::pack_set_laser_power(14).has_value(), "duty 14 pack dropped");
    check(!ccu::pack_set_laser_power(100).has_value(), "duty 100 pack dropped");
    check(!ccu::pack_set_laser_power(0).has_value(), "duty 0 pack dropped");

    const uint8_t low[] = {0x2B, 0x01, 0x00, 14};
    const uint8_t high[] = {0x2B, 0x01, 0x00, 100};
    const uint8_t empty[] = {0x2B, 0x00, 0x00};
    check(!ccu::unpack_set_laser_power(low).has_value(), "duty 14 unpack dropped");
    check(!ccu::unpack_set_laser_power(high).has_value(), "duty 100 unpack dropped");
    check(!ccu::unpack_set_laser_power(empty).has_value(), "empty SET_LASER_POWER dropped");
}

void test_nudges_and_tag() {
    const uint8_t nudge_x[] = {0x2C, 0x02, 0x00, 0xFE, 0xFF};
    const uint8_t nudge_y[] = {0x2D, 0x02, 0x00, 0x05, 0x00};
    const uint8_t min_x[] = {0x2C, 0x02, 0x00, 0x00, 0x80};
    const uint8_t max_x[] = {0x2C, 0x02, 0x00, 0xFF, 0x7F};
    auto packed_x = ccu::pack_nudge_x(-2);
    auto packed_y = ccu::pack_nudge_y(5);
    check(packed_x.has_value(), "NUDGE_X packs");
    check(packed_y.has_value(), "NUDGE_Y packs");
    if (packed_x) check_bytes(*packed_x, nudge_x, sizeof(nudge_x), "NUDGE_X bytes");
    if (packed_y) check_bytes(*packed_y, nudge_y, sizeof(nudge_y), "NUDGE_Y bytes");
    auto unpacked_x = ccu::unpack_nudge_x(nudge_x);
    auto unpacked_y = ccu::unpack_nudge_y(nudge_y);
    check(unpacked_x.has_value(), "NUDGE_X unpacks");
    check(unpacked_y.has_value(), "NUDGE_Y unpacks");
    if (unpacked_x) check_eq_i16(*unpacked_x, -2, "NUDGE_X dx");
    if (unpacked_y) check_eq_i16(*unpacked_y, 5, "NUDGE_Y dy");

    auto packed_min = ccu::pack_nudge_x(-32768);
    auto packed_max = ccu::pack_nudge_x(32767);
    if (packed_min) check_bytes(*packed_min, min_x, sizeof(min_x), "NUDGE_X min");
    if (packed_max) check_bytes(*packed_max, max_x, sizeof(max_x), "NUDGE_X max");
    auto unpacked_min = ccu::unpack_nudge_x(min_x);
    auto unpacked_max = ccu::unpack_nudge_x(max_x);
    check(unpacked_min.has_value(), "NUDGE_X min unpacks");
    check(unpacked_max.has_value(), "NUDGE_X max unpacks");
    if (unpacked_min) check_eq_i16(*unpacked_min, -32768, "dx min");
    if (unpacked_max) check_eq_i16(*unpacked_max, 32767, "dx max");

    const uint8_t tag[] = {0x2E, 0x03, 0x00, 0x02, 't', '1'};
    auto packed_tag = ccu::pack_tag_aligned("t1");
    check(packed_tag.has_value(), "TAG_ALIGNED packs");
    if (packed_tag) check_bytes(*packed_tag, tag, sizeof(tag), "TAG_ALIGNED bytes");
    auto unpacked_tag = ccu::unpack_tag_aligned(tag);
    check(unpacked_tag.has_value() && *unpacked_tag == "t1", "TAG_ALIGNED tagId");
    check(!ccu::unpack_nudge_x(nudge_y).has_value(), "NUDGE_X rejects NUDGE_Y");
}

}  // namespace

int main() {
    test_catalog_and_reserved();
    test_device_id();
    test_pairing_accepted();
    test_regular_telemetry();
    test_survey_result();
    test_projection_ready();
    test_connected_and_strings();
    test_empty_commands();
    test_session_ids();
    test_plan_commands();
    test_laser_power();
    test_nudges_and_tag();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_protocol_test passed\n");
    return 0;
}
