#include "rx/parser/DeviceParser.hpp"
#include "tx/CommandEncoder.hpp"
#include "utils/cmd_refs.hpp"
#include "utils/protocol.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
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

void check_eq_f(float got, float want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

void test_packed_sizes() {
    check(sizeof(ControlPacket) == 29, "ControlPacket is 29 bytes (HOST_PROTOCOL §4.1)");
    check(sizeof(StatusPacket) == 24, "StatusPacket is 24 bytes (HOST_PROTOCOL §4.2)");
    check(sizeof(ControlPayload) == 26, "control payload after prefix is 26 bytes");
    check(sizeof(StatusPayload) == 21, "status payload after prefix is 21 bytes");
    check(kControlLenField == 26, "control len field is 26");
    check(kStatusLenField == 21, "status len field is 21");
}

void test_command_example() {
    // HOST_PROTOCOL.md §2: TARGET_X_SET (0x19) payload 3.54 rad
    // 19 04 00 5C 8F 62 40
    auto frame = CommandEncoder::pack_f32(kCmdTargetXSet, {3.54f});
    check(frame.has_value(), "TARGET_X_SET encodes");
    if (!frame) return;

    const std::vector<uint8_t> want{0x19, 0x04, 0x00, 0x5C, 0x8F, 0x62, 0x40};
    check(frame->size() == want.size(), "TARGET_X_SET frame length");
    if (frame->size() == want.size())
        check(std::memcmp(frame->data(), want.data(), want.size()) == 0,
              "TARGET_X_SET bytes match HOST_PROTOCOL example");
}

void test_zero_len_command() {
    // HOST_PROTOCOL.md §2: 01 00 00  DISABLE_ENABLE
    auto frame = CommandEncoder::pack(kCmdDisableEnable, {});
    check(frame.has_value(), "DISABLE_ENABLE encodes");
    if (!frame) return;
    const std::vector<uint8_t> want{0x01, 0x00, 0x00};
    check(frame->size() == want.size(), "zero-payload command is 3 bytes");
    if (frame->size() == want.size())
        check(std::memcmp(frame->data(), want.data(), want.size()) == 0,
              "DISABLE_ENABLE bytes match HOST_PROTOCOL example");
}

void test_two_float_command() {
    auto frame = CommandEncoder::pack_f32(kCmdSetPoint2PointTarget, {3.54f, 3.39f});
    check(frame.has_value(), "SET_POINT2POINT_TARGET encodes");
    if (!frame) return;
    check(frame->size() == 11, "two-float command is 11 bytes");
    check_eq_u8((*frame)[0], kCmdSetPoint2PointTarget, "P2P cmd");
    check_eq_u8((*frame)[1], 8, "P2P len lo");
    check_eq_u8((*frame)[2], 0, "P2P len hi");
}

void test_set_velocity_max_command() {
    auto frame = CommandEncoder::pack_f32(kCmdSetVelocityMax, {1.5f});
    check(frame.has_value(), "SET_VELOCITY_MAX encodes");
    if (!frame) return;
    check(frame->size() == 7, "one-float command is 7 bytes");
    check_eq_u8((*frame)[0], kCmdSetVelocityMax, "SET_VELOCITY_MAX cmd");
    check_eq_u8((*frame)[1], 4, "len lo");
    check_eq_u8((*frame)[2], 0, "len hi");
}

void test_command_payload_max() {
    uint8_t too_big[kCmdPayloadMax + 1]{};
    check(!CommandEncoder::pack(kCmdDisableEnable, {too_big, sizeof(too_big)}).has_value(),
          "command payload >16 is rejected (HOST_PROTOCOL §2)");
    uint8_t ok[kCmdPayloadMax]{};
    check(CommandEncoder::pack(kCmdDisableEnable, {ok, sizeof(ok)}).has_value(),
          "command payload of 16 encodes");
}

void test_scan_engine_zero_len_commands() {
    struct Case {
        uint8_t cmd;
        const char* name;
    };
    const Case cases[] = {
        {kCmdScanEngineOn, "SCANENGINE_ON"},
        {kCmdScanEngineOff, "SCANENGINE_OFF"},
        {kCmdSeekNeutral, "SEEK_NEUTRAL"},
        {kCmdGetErrorMsg, "GET_ERROR_MSG"},
    };
    for (const Case& c : cases) {
        auto frame = CommandEncoder::pack(c.cmd, {});
        check(frame.has_value(), c.name);
        if (!frame) continue;
        check(frame->size() == 3, "scan-engine command is 3 bytes");
        check_eq_u8((*frame)[0], c.cmd, c.name);
        check_eq_u8((*frame)[1], 0, "len lo");
        check_eq_u8((*frame)[2], 0, "len hi");
    }
}

void test_dynamic_routine_names() {
    check(std::strcmp(dynamic_routine_name(kRoutineNone), "NONE") == 0, "routine 0");
    check(std::strcmp(dynamic_routine_name(kRoutineWindingUpToCoarse),
                      "WINDING_UP_TO_COARSE") == 0,
          "routine 1");
    check(std::strcmp(dynamic_routine_name(kRoutineTaperingDownFromCoarse),
                      "TAPERING_DOWN_FROM_COARSE") == 0,
          "routine 2");
    check(std::strcmp(dynamic_routine_name(3), "?") == 0, "unknown routine");
}

void test_parse_control() {
    ControlPacket pkt{};
    pkt.header = kControlHeader;
    pkt.len = kControlLenField;
    pkt.body.target_angle_x = 3.54f;
    pkt.body.target_angle_y = 3.39f;
    pkt.body.distance = 123;
    pkt.body.mechanical_angle_x = 3.50f;
    pkt.body.mechanical_angle_y = 3.40f;
    pkt.body.associated_angle_x = 3.51f;
    pkt.body.associated_angle_y = 3.41f;

    uint8_t wire[sizeof(ControlPacket)];
    std::memcpy(wire, &pkt, sizeof(wire));

    uint8_t buf[64]{};
    uint8_t got_cmd = 0;
    uint16_t got_len = 0;
    bool got = false;
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        got = true;
        got_cmd = cmd;
        got_len = len;
    });
    for (uint8_t b : wire)
        parser.pump(b);

    check(got, "parser emits control frame");
    check_eq_u8(got_cmd, kControlHeader, "control header");
    check_eq_u16(got_len, kControlPayloadSize, "control payload length");

    ControlPayload body{};
    check(unpack_control({buf, got_len}, body), "unpack control");
    check_eq_f(body.target_angle_x, 3.54f, "target_angle_x");
    check_eq_f(body.target_angle_y, 3.39f, "target_angle_y");
    check_eq_u16(body.distance, 123, "distance");
    check_eq_f(body.mechanical_angle_x, 3.50f, "mechanical_angle_x");
    check_eq_f(body.mechanical_angle_y, 3.40f, "mechanical_angle_y");
    check_eq_f(body.associated_angle_x, 3.51f, "associated_angle_x");
    check_eq_f(body.associated_angle_y, 3.41f, "associated_angle_y");
}

void test_parse_status() {
    StatusPacket pkt{};
    pkt.header = kStatusHeader;
    pkt.len = kStatusLenField;
    pkt.body.enabled = 1;
    pkt.body.scan_mode = kScanModePoint2Point;
    pkt.body.dynamic_routine = kRoutineNone;
    pkt.body.debug_mode = 0;
    pkt.body.has_fault = 0;
    pkt.body.coarse_centre_x = 3.54f;
    pkt.body.coarse_centre_y = 3.39f;
    pkt.body.p2p_target_x = 3.7f;
    pkt.body.p2p_target_y = 3.5f;

    uint8_t wire[sizeof(StatusPacket)];
    std::memcpy(wire, &pkt, sizeof(wire));

    uint8_t buf[64]{};
    uint8_t got_cmd = 0;
    uint16_t got_len = 0;
    bool got = false;
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        got = true;
        got_cmd = cmd;
        got_len = len;
    });
    for (uint8_t b : wire)
        parser.pump(b);

    check(got, "parser emits status frame");
    check_eq_u8(got_cmd, kStatusHeader, "status header");
    check_eq_u16(got_len, kStatusPayloadSize, "status payload length");

    StatusPayload body{};
    check(unpack_status({buf, got_len}, body), "unpack status");
    check_eq_u8(body.enabled, 1, "enabled");
    check_eq_u8(body.scan_mode, kScanModePoint2Point, "scan_mode");
    check_eq_u8(body.dynamic_routine, kRoutineNone, "dynamic_routine");
    check_eq_f(body.p2p_target_x, 3.7f, "p2p_target_x");
    check_eq_f(body.p2p_target_y, 3.5f, "p2p_target_y");
}

void test_debug_frame() {
    const uint8_t debug[] = {0xFF, 0x02, 0x00, 'h', 'i'};
    uint8_t buf[64]{};
    uint8_t got_cmd = 0;
    uint16_t got_len = 0;
    int n = 0;
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        ++n;
        got_cmd = cmd;
        got_len = len;
    });
    for (uint8_t b : debug)
        parser.pump(b);
    check(n == 1, "parser emits one debug frame");
    check_eq_u8(got_cmd, kDebugHeader, "debug header");
    check_eq_u16(got_len, 2, "debug payload length");
    check(std::memcmp(buf, "hi", 2) == 0, "debug payload");
}

void test_debug_ascii_example() {
    // HOST_PROTOCOL.md §7: payload 41 74 20 6E 65 75 74 72 61 6C 2E → "At neutral."
    const uint8_t payload[] = {
        0x41, 0x74, 0x20, 0x6E, 0x65, 0x75, 0x74, 0x72, 0x61, 0x6C, 0x2E};
    std::vector<uint8_t> wire;
    wire.push_back(kDebugHeader);
    wire.push_back(static_cast<uint8_t>(sizeof(payload)));
    wire.push_back(0x00);
    wire.insert(wire.end(), payload, payload + sizeof(payload));

    uint8_t buf[kDebugMaxLen]{};
    uint8_t got_cmd = 0;
    uint16_t got_len = 0;
    int n = 0;
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        ++n;
        got_cmd = cmd;
        got_len = len;
    });
    for (uint8_t b : wire)
        parser.pump(b);

    check(n == 1, "parser emits documented debug example");
    check_eq_u8(got_cmd, kDebugHeader, "debug example header");
    check_eq_u16(got_len, static_cast<uint16_t>(sizeof(payload)), "debug example length");
    check(decode_debug_ascii({buf, got_len}) == "At neutral.",
          "debug example decodes as ASCII");
}

void test_debug_payload_not_parsed_as_command() {
    // HOST_PROTOCOL.md §7: do not parse the debug payload as a command.
    uint8_t buf[64]{};
    int n = 0;
    uint8_t got_cmd = 0;
    uint16_t got_len = 0;
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        ++n;
        got_cmd = cmd;
        got_len = len;
    });
    const uint8_t frame[] = {0xFF, 0x03, 0x00, 0x55, 0x1B, 0x00};
    for (uint8_t b : frame)
        parser.pump(b);
    check(n == 1, "debug payload bytes are not extra frames");
    check_eq_u8(got_cmd, kDebugHeader, "debug cmd");
    check_eq_u16(got_len, 3, "debug length");
    check(buf[0] == 0x55 && buf[1] == 0x1B && buf[2] == 0x00, "opaque debug bytes");
}

void test_any_cmd_is_framed() {
    const uint8_t frame[] = {0x42, 0x04, 0x00, 1, 2, 3, 4};
    uint8_t buf[64]{};
    uint8_t got_cmd = 0;
    uint16_t got_len = 0;
    int n = 0;
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        ++n;
        got_cmd = cmd;
        got_len = len;
    });
    for (uint8_t b : frame)
        parser.pump(b);
    check(n == 1, "parser emits unknown cmd");
    check_eq_u8(got_cmd, 0x42, "unknown cmd byte");
    check_eq_u16(got_len, 4, "declared payload length");
    check(std::memcmp(buf, "\x01\x02\x03\x04", 4) == 0, "opaque payload");
}

void test_zero_length_payload() {
    const uint8_t frame[] = {0xFF, 0x00, 0x00};
    uint8_t buf[64]{};
    uint8_t got_cmd = 0;
    uint16_t got_len = 0xFFFF;
    int n = 0;
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        ++n;
        got_cmd = cmd;
        got_len = len;
    });
    for (uint8_t b : frame)
        parser.pump(b);
    check(n == 1, "zero-length frame is emitted");
    check_eq_u8(got_cmd, kDebugHeader, "zero-length cmd");
    check_eq_u16(got_len, 0, "zero-length payload");
}

void test_trusts_declared_length() {
    // Parser does not validate cmd-specific sizes: len=4 on a 0x55 frame is kept.
    const uint8_t wire[] = {kControlHeader, 0x04, 0x00, 10, 20, 30, 40};
    uint8_t buf[64]{};
    uint8_t got_cmd = 0;
    uint16_t got_len = 0;
    int n = 0;
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        ++n;
        got_cmd = cmd;
        got_len = len;
    });
    for (uint8_t b : wire)
        parser.pump(b);
    check(n == 1, "frame with non-canonical control len is emitted");
    check_eq_u8(got_cmd, kControlHeader, "trusted-len cmd");
    check_eq_u16(got_len, 4, "trusted declared length");
    check(buf[0] == 10 && buf[3] == 40, "trusted-len payload");
}

void test_skips_payload_larger_than_buffer() {
    uint8_t buf[8]{};
    int n = 0;
    uint8_t got_cmd = 0;
    uint16_t got_len = 0;
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        ++n;
        got_cmd = cmd;
        got_len = len;
    });

    const uint8_t oversize_hdr[] = {0x01, 0x0A, 0x00};  // len = 10 > cap 8
    for (uint8_t b : oversize_hdr)
        parser.pump(b);
    for (uint8_t i = 0; i < 10; ++i)
        parser.pump(i);
    check(n == 0, "oversize payload is not emitted");

    const uint8_t ok[] = {0xFF, 0x02, 0x00, 'h', 'i'};
    for (uint8_t b : ok)
        parser.pump(b);
    check(n == 1, "parser stays in sync after oversize skip");
    check_eq_u8(got_cmd, kDebugHeader, "frame after skip");
    check_eq_u16(got_len, 2, "length after skip");
    check(std::memcmp(buf, "hi", 2) == 0, "payload after skip");
}

void test_interleaved_control_status() {
    ControlPacket c{};
    c.header = kControlHeader;
    c.len = kControlLenField;
    c.body.distance = 1;
    StatusPacket s{};
    s.header = kStatusHeader;
    s.len = kStatusLenField;
    s.body.enabled = 1;

    uint8_t stream[sizeof(c) + sizeof(s)];
    std::memcpy(stream, &c, sizeof(c));
    std::memcpy(stream + sizeof(c), &s, sizeof(s));

    int n_ctrl = 0;
    int n_stat = 0;
    uint8_t buf[64]{};
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t) {
        if (cmd == kControlHeader) ++n_ctrl;
        if (cmd == kStatusHeader) ++n_stat;
        parser.set_buffer(buf, sizeof(buf));
    });
    for (uint8_t b : stream)
        parser.pump(b);
    check(n_ctrl == 1 && n_stat == 1, "interleaved 0x55 then 0xAA");
}

void test_interleaved_debug() {
    ControlPacket c{};
    c.header = kControlHeader;
    c.len = kControlLenField;
    c.body.distance = 7;

    const uint8_t dbg[] = {0xFF, 0x03, 0x00, 'a', 'b', 'c'};
    uint8_t stream[sizeof(c) + sizeof(dbg)];
    std::memcpy(stream, &c, sizeof(c));
    std::memcpy(stream + sizeof(c), dbg, sizeof(dbg));

    int n_ctrl = 0;
    int n_dbg = 0;
    uint8_t buf[64]{};
    DeviceParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        if (cmd == kControlHeader) ++n_ctrl;
        if (cmd == kDebugHeader) {
            ++n_dbg;
            check_eq_u16(len, 3, "interleaved debug length");
            check(std::memcmp(buf, "abc", 3) == 0, "interleaved debug text");
        }
        parser.set_buffer(buf, sizeof(buf));
    });
    for (uint8_t b : stream)
        parser.pump(b);
    check(n_ctrl == 1 && n_dbg == 1, "interleaved 0x55 then 0xFF");
}

}  // namespace

int main() {
    test_packed_sizes();
    test_command_example();
    test_zero_len_command();
    test_two_float_command();
    test_set_velocity_max_command();
    test_command_payload_max();
    test_scan_engine_zero_len_commands();
    test_dynamic_routine_names();
    test_parse_control();
    test_parse_status();
    test_debug_frame();
    test_debug_ascii_example();
    test_debug_payload_not_parsed_as_command();
    test_any_cmd_is_framed();
    test_zero_length_payload();
    test_trusts_declared_length();
    test_skips_payload_larger_than_buffer();
    test_interleaved_control_status();
    test_interleaved_debug();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("protocol tests passed\n");
    return 0;
}
