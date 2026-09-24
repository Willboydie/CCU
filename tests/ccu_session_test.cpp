#include "protocol.hpp"
#include "session.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>

namespace {

int g_fails = 0;

void check(bool cond, const char* what) {
    if (cond) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_fails;
}

void check_eq_f(float got, float want, const char* what) {
    if (got == want) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

void check_near(float got, float want, const char* what) {
    constexpr float kTol = 1e-5f;
    if (got >= want - kTol && got <= want + kTol) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

bool is_hex8(const std::string& id) {
    if (id.size() != 8) return false;
    for (char c : id) {
        const bool digit = c >= '0' && c <= '9';
        const bool hex = c >= 'a' && c <= 'f';
        if (!digit && !hex) return false;
    }
    return true;
}

void send(const std::optional<std::vector<uint8_t>>& frame) {
    check(frame.has_value(), "frame packed");
    if (frame) ccu::session_on_frame(*frame);
}

void test_origin_latch() {
    ccu::session_reset();
    check(!ccu::session_running(), "reset is not running");
    check(!ccu::session_origin_is_set(), "reset has no origin");

    send(ccu::pack_begin_new_session());
    check(ccu::session_running(), "BEGIN_NEW_SESSION latches running");
    check(is_hex8(ccu::session_id()), "session id is 8 hex characters");
    check(!ccu::session_origin_is_set(), "new session has no origin");

    constexpr float kStage = 1.2f;
    ccu::session_set_stage(kStage);
    send(ccu::pack_begin_origin_set());
    check(ccu::session_origin_is_set(), "BEGIN_ORIGIN_SET sets origin");
    check_eq_f(ccu::session_north_offset(), kStage, "north offset stores stage 1.2");
    check_eq_f(ccu::session_effective_yaw(), 0.f, "yaw is 0 at the origin set");

    const float turned = kStage - std::numbers::pi_v<float> / 2.f;
    ccu::session_set_stage(turned);
    check_near(ccu::session_effective_yaw(), std::numbers::pi_v<float> / 2.f,
               "stage decrease of pi/2 yields yaw pi/2");

    send(ccu::pack_begin_origin_set());
    check(ccu::session_origin_is_set(), "origin stays set");
    check_eq_f(ccu::session_north_offset(), kStage, "second origin command is dropped");
    check_near(ccu::session_effective_yaw(), std::numbers::pi_v<float> / 2.f,
               "yaw still follows the turned stage");

    send(ccu::pack_end_session("zzzzzzzz"));
    check(ccu::session_running(), "mismatched END_SESSION is dropped");
    check(ccu::session_origin_is_set(), "mismatched END_SESSION keeps origin");

    send(ccu::pack_end_session(ccu::session_id()));
    check(!ccu::session_running(), "END_SESSION clears running");
    check(!ccu::session_origin_is_set(), "END_SESSION clears origin");
    check(ccu::session_id().empty(), "END_SESSION clears the session id");
    check_eq_f(ccu::session_north_offset(), 0.f, "END_SESSION clears north offset");
}

void test_join_keeps_origin() {
    ccu::session_reset();
    send(ccu::pack_begin_new_session());
    check(ccu::session_link_up(), "BEGIN_NEW_SESSION latches the link");
    ccu::session_mark_pairing_sent();

    constexpr float kStage = 1.2f;
    ccu::session_set_stage(kStage);
    send(ccu::pack_begin_origin_set());
    const std::string id = ccu::session_id();

    send(ccu::pack_join_session(id));
    check(!ccu::session_pairing_unsent(), "JOIN while linked is dropped");
    check_eq_f(ccu::session_north_offset(), kStage, "dropped JOIN keeps north offset");

    send(ccu::pack_connected("pad"));
    check(ccu::session_link_up(), "CONNECTED keeps the link");

    ccu::session_expire_link();
    check(!ccu::session_link_up(), "silence drops the link");
    check(ccu::session_running(), "dropped link keeps the session");
    check(ccu::session_origin_is_set(), "dropped link keeps origin");
    check_eq_f(ccu::session_north_offset(), kStage, "dropped link keeps north offset");

    send(ccu::pack_connected("pad"));
    check(!ccu::session_link_up(), "CONNECTED does not revive a dropped link");

    send(ccu::pack_join_session("zzzzzzzz"));
    check(!ccu::session_link_up(), "mismatched JOIN is dropped");
    check(!ccu::session_pairing_unsent(), "mismatched JOIN does not pair");

    send(ccu::pack_join_session(id));
    check(ccu::session_link_up(), "matching JOIN latches the link");
    check(ccu::session_pairing_unsent(), "matching JOIN sends pairing");
    check(ccu::session_origin_is_set(), "JOIN keeps origin");
    check_eq_f(ccu::session_north_offset(), kStage, "JOIN keeps north offset");
    check_eq_f(ccu::session_effective_yaw(), 0.f, "JOIN does not reset yaw");

    ccu::session_set_stage(kStage - std::numbers::pi_v<float> / 2.f);
    check_near(ccu::session_effective_yaw(), std::numbers::pi_v<float> / 2.f,
               "yaw after JOIN follows the current stage");
}

}  // namespace

void test_end_mid_survey() {
    ccu::session_reset();
    send(ccu::pack_begin_new_session());
    ccu::session_set_stage(0.f);
    send(ccu::pack_begin_origin_set());

    const ccu::PlanNode first{1, 0.f, 1000.f, 0.f, 0.f};
    const ccu::PlanNode second{2, 0.f, 2000.f, 0.f, 0.f};
    send(ccu::pack_perform_survey({first, second}));
    const std::string id = ccu::session_id();
    send(ccu::pack_end_session(id));

    const std::vector<ccu::SurveyResult> aborted = ccu::session_take_survey_results();
    check(aborted.size() == 2, "both remaining nodes are aborted");
    if (aborted.size() == 2) {
        check(aborted[0].id == 1 && aborted[1].id == 2, "aborted ids");
        check(aborted[0].error_code == ccu::SurveyError::Aborted, "first ABORTED");
        check(aborted[1].error_code == ccu::SurveyError::Aborted, "second ABORTED");
        check(!aborted[0].survey_finished, "surveyFinished is on the last abort");
        check(aborted[1].survey_finished, "last abort is surveyFinished");
        check(!aborted[0].success && !aborted[1].success, "aborted nodes fail");
    }
    check(!ccu::session_running(), "session ended");

    send(ccu::pack_begin_new_session());
    ccu::session_set_stage(0.f);
    send(ccu::pack_begin_origin_set());
    send(ccu::pack_perform_projection({first, second}));
    const std::optional<ccu::ProjectionReady> ready = ccu::session_take_projection_ready();
    check(ready.has_value(), "later projection answers");
    if (!ready) return;
    check(ready->nodes.empty(), "survey_reset left nothing to draw");
    check(ready->errors.size() == 2, "both ids are omitted");
    if (ready->errors.size() == 2) {
        check(ready->errors[0].id == 1 && ready->errors[0].error_code == 4, "first NOT_SURVEYED");
        check(ready->errors[1].id == 2 && ready->errors[1].error_code == 4, "second NOT_SURVEYED");
    }
}

int main() {
    test_origin_latch();
    test_join_keeps_origin();
    test_end_mid_survey();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_session_test passed\n");
    return 0;
}
