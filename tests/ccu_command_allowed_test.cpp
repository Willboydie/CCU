#include "command_allowed.hpp"
#include "protocol.hpp"

#include <cstdio>

namespace {

int g_fails = 0;

void expect(ccu::AcceptMode mode, uint8_t cmd, bool want, const char* what) {
    const bool got = ccu::command_allowed(mode, cmd);
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %s want %s)\n", what, got ? "allow" : "drop",
                 want ? "allow" : "drop");
    ++g_fails;
}

constexpr ccu::AcceptMode kIdle{false, ccu::WorkMode::Idle, false};
constexpr ccu::AcceptMode kRunClear{true, ccu::WorkMode::Idle, false};
constexpr ccu::AcceptMode kRunSet{true, ccu::WorkMode::Idle, true};
constexpr ccu::AcceptMode kSurvey{true, ccu::WorkMode::Surveying, true};
constexpr ccu::AcceptMode kProject{true, ccu::WorkMode::Projecting, true};

void row(uint8_t cmd, const char* name, bool idle, bool clear, bool set, bool survey, bool project) {
    struct Cell {
        ccu::AcceptMode mode;
        bool allow;
        const char* where;
    };
    const Cell cells[] = {
        {kIdle, idle, "session idle"},
        {kRunClear, clear, "running, origin clear"},
        {kRunSet, set, "running, origin set"},
        {kSurvey, survey, "surveying"},
        {kProject, project, "projecting"},
    };
    for (const Cell& cell : cells) {
        char what[160];
        std::snprintf(what, sizeof(what), "%s / %s", name, cell.where);
        expect(cell.mode, cmd, cell.allow, what);
    }
}

void test_accept_table() {
    row(ccu::kCmdBeginNewSession, "BEGIN_NEW_SESSION", true, false, false, false, false);
    row(ccu::kCmdJoinSession, "JOIN_SESSION", true, false, false, false, false);
    row(ccu::kCmdConnected, "CONNECTED", false, true, true, true, true);
    row(ccu::kCmdEndSession, "END_SESSION", false, true, true, true, true);
    row(ccu::kCmdBeginOriginSet, "BEGIN_ORIGIN_SET", false, true, false, false, false);
    row(ccu::kCmdBeginTagFinding, "BEGIN_TAG_FINDING", false, false, false, false, false);
    row(ccu::kCmdTagAligned, "TAG_ALIGNED", false, false, false, false, false);
    row(ccu::kCmdPerformSurvey, "PERFORM_SURVEY", false, false, true, false, false);
    row(ccu::kCmdAbortSurvey, "ABORT_SURVEY", false, false, false, true, false);
    row(ccu::kCmdPerformProjection, "PERFORM_PROJECTION", false, false, true, false, false);
    row(ccu::kCmdProjectorPause, "PROJECTOR_PAUSE", false, false, false, false, true);
    row(ccu::kCmdProjectorPlay, "PROJECTOR_PLAY", false, false, false, false, true);
    row(ccu::kCmdProjectionEnd, "PROJECTION_END", false, false, false, false, true);
    row(ccu::kCmdSetLaserPower, "SET_LASER_POWER", false, true, true, true, true);
    row(ccu::kCmdNudgeX, "NUDGE_X", false, false, false, false, true);
    row(ccu::kCmdNudgeY, "NUDGE_Y", false, false, false, false, true);
}

void test_origin_and_idle_do_not_leak() {
    const ccu::AcceptMode idle_set{false, ccu::WorkMode::Idle, true};
    expect(idle_set, ccu::kCmdBeginNewSession, true, "BEGIN_NEW_SESSION while idle with origin set");
    expect(idle_set, ccu::kCmdPerformSurvey, false, "survey refused while session idle");
    expect(kRunClear, ccu::kCmdPerformSurvey, false, "survey refused while origin is clear");
    expect(kRunClear, ccu::kCmdPerformProjection, false, "projection refused while origin is clear");
    expect(kRunSet, ccu::kCmdBeginOriginSet, false, "second origin command refused");

    const ccu::AcceptMode survey_clear{true, ccu::WorkMode::Surveying, false};
    expect(survey_clear, ccu::kCmdAbortSurvey, true, "abort survey with origin clear");
    expect(survey_clear, ccu::kCmdPerformSurvey, false, "survey start refused while already surveying");

    const ccu::AcceptMode project_clear{true, ccu::WorkMode::Projecting, false};
    expect(project_clear, ccu::kCmdNudgeX, true, "nudge while projecting, origin clear");
    expect(project_clear, ccu::kCmdProjectorPause, true, "pause while projecting, origin clear");

    const ccu::AcceptMode not_running{false, ccu::WorkMode::Projecting, true};
    expect(not_running, ccu::kCmdJoinSession, true, "JOIN while session is not running");
    expect(not_running, ccu::kCmdNudgeY, false, "nudge refused when session is not running");
    expect(not_running, ccu::kCmdSetLaserPower, false, "laser refused when session is not running");
}

void test_tags_and_unknown_dropped() {
    const ccu::AcceptMode modes[] = {kIdle, kRunClear, kRunSet, kSurvey, kProject};
    const uint8_t tags[] = {ccu::kCmdBeginTagFinding, ccu::kCmdTagAligned};
    for (ccu::AcceptMode mode : modes) {
        for (uint8_t cmd : tags) {
            expect(mode, cmd, false, "tag command dropped");
        }
    }
    expect(kRunSet, ccu::kCmdDeviceId, false, "device packet is not a command");
    expect(kProject, 0xE5, false, "0xE5 refused while projecting");
    expect(kIdle, 0x00, false, "unknown cmd refused while idle");
}

}  // namespace

int main() {
    test_accept_table();
    test_origin_and_idle_do_not_leak();
    test_tags_and_unknown_dropped();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_command_allowed_test passed\n");
    return 0;
}
