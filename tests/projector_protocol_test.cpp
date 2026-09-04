#include "rx/parser/ProjectorParser.hpp"
#include "tx/CommandEncoder.hpp"
#include "utils/projector_calibration.hpp"
#include "utils/projector_protocol.hpp"

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

void test_set_point_payload() {
    uint8_t p[5]{};
    pack_set_point(p, 0x1234, 0xABCD, true);
    check_eq_u8(p[0], 0x34, "SET_POINT X lo");
    check_eq_u8(p[1], 0x12, "SET_POINT X hi");
    check_eq_u8(p[2], 0xCD, "SET_POINT Y lo");
    check_eq_u8(p[3], 0xAB, "SET_POINT Y hi");
    check_eq_u8(p[4], 1, "SET_POINT laser on");

    auto frame = CommandEncoder::pack(kProjCmdSetPoint, {p, sizeof(p)});
    check(frame.has_value(), "SET_POINT encodes");
    if (!frame) return;
    check(frame->size() == 8, "SET_POINT frame is 8 bytes");
    check_eq_u8((*frame)[0], kProjCmdSetPoint, "SET_POINT cmd");
    check_eq_u8((*frame)[1], 5, "SET_POINT len lo");
    check_eq_u8((*frame)[2], 0, "SET_POINT len hi");
}

void test_parse_status() {
    ProjectorStatus body{};
    body.power_state = kProjPowerAwake;
    body.mode = kProjModePoint;
    body.enabled = 1;
    body.projecting = 1;
    body.laser_duty_cycle = 50;
    body.project_mode = kProjModePoint;
    body.stream_state = kProjStreamReset;

    uint8_t wire[3 + sizeof(ProjectorStatus)]{};
    wire[0] = kProjStatusHeader;
    wire[1] = kProjStatusPayloadSize;
    wire[2] = 0;
    std::memcpy(wire + 3, &body, sizeof(body));

    uint8_t buf[64]{};
    uint8_t got_cmd = 0;
    uint16_t got_len = 0;
    bool got = false;
    ProjectorParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        got = true;
        got_cmd = cmd;
        got_len = len;
    });
    for (uint8_t b : wire)
        parser.pump(b);

    check(got, "parser emits status");
    check_eq_u8(got_cmd, kProjStatusHeader, "status header");
    check_eq_u16(got_len, kProjStatusPayloadSize, "status payload length");

    ProjectorStatus unpacked{};
    check(unpack_projector_status({buf, got_len}, unpacked), "unpack status");
    check_eq_u8(unpacked.enabled, 1, "enabled");
    check_eq_u8(unpacked.project_mode, kProjModePoint, "project_mode");
    check_eq_u8(unpacked.laser_duty_cycle, 50, "duty");
}

void test_parse_debug_and_raw() {
    const uint8_t stream[] = {
        kProjNotifyPauseStream,
        kProjDebugPrefix0, kProjDebugPrefix1, kProjDebugPrefix2,
        'f', 'a', 'u', 'l', 't', '\n',
        kProjNotifyResumeStream,
    };

    uint8_t buf[64]{};
    std::vector<uint8_t> cmds;
    std::vector<uint16_t> lens;
    std::string debug;
    ProjectorParser parser;
    parser.set_buffer(buf, sizeof(buf));
    parser.set_on_frame([&](uint8_t cmd, uint16_t len) {
        cmds.push_back(cmd);
        lens.push_back(len);
        if (cmd == kProjDebugHeader)
            debug.assign(reinterpret_cast<const char*>(buf), len);
    });
    for (uint8_t b : stream)
        parser.pump(b);

    check(cmds.size() == 3, "three frames");
    if (cmds.size() == 3) {
        check_eq_u8(cmds[0], kProjNotifyPauseStream, "pause");
        check_eq_u16(lens[0], 0, "pause len");
        check_eq_u8(cmds[1], kProjDebugHeader, "debug");
        check(debug == "fault", "debug text");
        check_eq_u8(cmds[2], kProjNotifyResumeStream, "resume");
    }
}

void test_saturate() {
    check_eq_u16(saturate_dac(-1), 0, "clamp low");
    check_eq_u16(saturate_dac(70000), kProjDacMax, "clamp high");
    check_eq_u16(saturate_dac(32768), kProjDacCentre, "centre");
}

void test_calib_derivative() {
    check(sizeof(ProjectorCalibDatum) == 7, "calib datum is 7 packed bytes");
    float d = 0;
    check(calib_derivative(1.0f, 100, d), "deriv ok");
    check(d > 9.9e-5f && d < 1.01e-4f, "f' = 0.01 / (1.0 * 100)");
    check(!calib_derivative(0.0f, 100, d), "zero range rejected");
    check(!calib_derivative(1.0f, 0, d), "zero ΔDAC rejected");
}

}  // namespace

int main() {
    test_set_point_payload();
    test_parse_status();
    test_parse_debug_and_raw();
    test_saturate();
    test_calib_derivative();
    if (g_fails) {
        std::fprintf(stderr, "%d projector protocol tests failed\n", g_fails);
        return 1;
    }
    std::printf("projector protocol tests passed\n");
    return 0;
}
