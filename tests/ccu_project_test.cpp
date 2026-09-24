#include "path_engine.hpp"
#include "project.hpp"
#include "protocol.hpp"
#include "session.hpp"
#include "survey.hpp"

#include <cstdio>
#include <optional>
#include <vector>

namespace {

int g_fails = 0;

void check(bool cond, const char* what) {
    if (cond) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_fails;
}

void check_eq_u16(uint16_t got, uint16_t want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %u want %u)\n", what, got, want);
    ++g_fails;
}

void check_eq_u8(uint8_t got, uint8_t want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %u want %u)\n", what, got, want);
    ++g_fails;
}

void check_eq_f(float got, float want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

std::optional<float> fake_range() { return 2500.f; }

void send(const std::optional<std::vector<uint8_t>>& frame) {
    check(frame.has_value(), "frame packed");
    if (frame) ccu::session_on_frame(*frame);
}

bool same_point(const ccu::PathPoint& got, const ccu::PathPoint& want) {
    return got.east_mm == want.east_mm && got.north_mm == want.north_mm && got.up_mm == want.up_mm &&
           got.laser_on == want.laser_on && got.duty == want.duty;
}

void test_projection_ready() {
    ccu::session_reset();
    send(ccu::pack_begin_new_session());
    ccu::session_set_stage(0.f);
    send(ccu::pack_begin_origin_set());

    ccu::PlanNode surveyed;
    surveyed.id = 1;
    surveyed.east_mm = 0.f;
    surveyed.north_mm = 2000.f;
    surveyed.corner_inner_angle_rad = 0.5f;
    surveyed.corner_orientation_rad = 0.25f;

    ccu::PlanNode missing;
    missing.id = 7;
    missing.east_mm = 100.f;
    missing.north_mm = 2000.f;

    ccu::survey_begin({surveyed}, fake_range);
    std::optional<ccu::SurveyResult> done;
    for (int i = 0; i < 30 && !done; ++i) {
        const std::optional<ccu::SurveyResult> result = ccu::survey_poll();
        if (result && result->survey_finished) done = result;
    }
    check(done.has_value() && done->success, "survey stores a success");
    if (!done) return;

    const ccu::ProjectionReady ready = ccu::project_begin({surveyed, missing});
    check(ready.nodes.size() == 1, "surveyed node is drawn");
    if (!ready.nodes.empty()) {
        check_eq_u16(ready.nodes[0].id, 1, "drawn id");
        check_eq_f(ready.nodes[0].east_mm, 0.f, "drawn east");
        check_eq_f(ready.nodes[0].north_mm, 2000.f, "drawn north");
    }
    check(ready.errors.size() == 1, "one omitted node");
    if (!ready.errors.empty()) {
        check_eq_u16(ready.errors[0].id, 7, "unsurveyed id");
        check_eq_u8(ready.errors[0].error_code, 4, "NOT_SURVEYED");
    }

    const std::vector<ccu::PathPoint> want = ccu::path_engine(
        {{done->east_mm, done->north_mm, done->up_mm}}, ccu::kLaserDutyDefault);
    const std::vector<ccu::PathPoint>& got = ccu::project_path();
    check(got.size() == want.size(), "stored path length matches G8");
    bool path_matches = got.size() == want.size();
    for (std::size_t i = 0; path_matches && i < got.size(); ++i)
        path_matches = same_point(got[i], want[i]);
    check(path_matches, "stored path is the G8 corner path");
}

}  // namespace

int main() {
    test_projection_ready();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_project_test passed\n");
    return 0;
}
