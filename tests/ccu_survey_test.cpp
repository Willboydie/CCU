#include "lidar_model.hpp"
#include "protocol.hpp"
#include "session.hpp"
#include "survey.hpp"

#include <cmath>
#include <cstdio>
#include <optional>
#include <vector>

namespace {

int g_fails = 0;
int g_reads = 0;

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

void check_near(float got, float want, const char* what) {
    constexpr float kTol = 1e-3f;
    if (got >= want - kTol && got <= want + kTol) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

std::optional<float> fake_range() {
    ++g_reads;
    return 2500.f;
}

void send(const std::optional<std::vector<uint8_t>>& frame) {
    check(frame.has_value(), "frame packed");
    if (frame) ccu::session_on_frame(*frame);
}

void test_out_of_fov_then_success() {
    ccu::session_reset();
    send(ccu::pack_begin_new_session());
    ccu::session_set_stage(0.f);
    send(ccu::pack_begin_origin_set());
    check(ccu::session_effective_yaw() == 0.f, "yaw is north");

    ccu::PlanNode outside;
    outside.id = 2;
    outside.east_mm = 1000.f;
    outside.north_mm = 0.f;
    outside.corner_inner_angle_rad = 0.3f;
    outside.corner_orientation_rad = 0.4f;

    ccu::PlanNode inside;
    inside.id = 1;
    inside.east_mm = 0.f;
    inside.north_mm = 2000.f;
    inside.corner_inner_angle_rad = 1.1f;
    inside.corner_orientation_rad = 0.2f;

    g_reads = 0;
    ccu::survey_begin({outside, inside}, fake_range);

    std::optional<ccu::SurveyResult> first;
    for (int i = 0; i < 5 && !first; ++i) first = ccu::survey_poll();

    check(first.has_value(), "first result");
    if (first) {
        check_eq_u16(first->id, 2, "out-of-wedge id");
        check(!first->surveyed, "out-of-wedge surveyed is 0");
        check(!first->success, "out-of-wedge fails");
        check(first->up_mm == 0.f, "out-of-wedge up is 0");
        check(first->error_code == ccu::SurveyError::OutOfFov, "OUT_OF_FOV");
        check(!first->survey_finished, "first result is not finished");
        check(first->east_mm == 1000.f && first->north_mm == 0.f, "out-of-wedge plan position");
    }
    check(g_reads == 0, "lidar is not read for the out-of-wedge node");

    std::optional<ccu::SurveyResult> second;
    for (int i = 0; i < 30 && !second; ++i) second = ccu::survey_poll();

    const float elevation = std::atan2(ccu::kFloorUpMm, 2000.f);
    const float expected_up = 2500.f * std::sin(elevation);
    check(second.has_value(), "second result");
    if (second) {
        check_eq_u16(second->id, 1, "in-wedge id");
        check(second->surveyed, "in-wedge surveyed");
        check(second->success, "in-wedge success");
        check(second->error_code == ccu::SurveyError::None, "success error is none");
        check_near(second->up_mm, expected_up, "naive up");
        check(second->survey_finished, "surveyFinished on the second");
        check(second->corner_inner_angle_rad == 1.1f, "inner angle kept");
        check(second->corner_orientation_rad == 0.2f, "orientation kept");
    }
    check(g_reads == 20, "dwell keeps the latest of 20 ranges");
}

}  // namespace

int main() {
    test_out_of_fov_then_success();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_survey_test passed\n");
    return 0;
}
