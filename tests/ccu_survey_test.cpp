#include "device_config.hpp"
#include "lidar.hpp"
#include "lidar_model.hpp"
#include "protocol.hpp"
#include "session.hpp"
#include "survey.hpp"
#include "utils/protocol.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <optional>
#include <utility>
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

void check_near(float got, float want, const char* what) {
    constexpr float kTol = 1e-3f;
    if (got >= want - kTol && got <= want + kTol) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

void send(const std::optional<std::vector<uint8_t>>& frame) {
    check(frame.has_value(), "frame packed");
    if (frame) ccu::session_on_frame(*frame);
}

void load_cfg(const char* body) {
    const char* path = "ccu_survey_gate.cfg";
    std::ofstream out(path);
    check(static_cast<bool>(out), "survey config opens");
    out << body;
    out.close();
    ccu::device_config_load(path);
}

void boot() {
    ccu::session_reset();
    ccu::lidar_test_reset();
    ccu::survey_reset();
    load_cfg("DWELL_TIME_S=0\n");
    send(ccu::pack_begin_new_session());
    ccu::session_set_stage(0.f);
    send(ccu::pack_begin_origin_set());
}

StatusPayload status_of(uint8_t enabled, uint8_t scan_mode) {
    StatusPayload status{};
    status.enabled = enabled;
    status.scan_mode = scan_mode;
    status.dynamic_routine = kRoutineNone;
    return status;
}

ControlPayload control_at(float mech_x, float mech_y, float assoc_x, float assoc_y, uint16_t cm) {
    ControlPayload control{};
    control.mechanical_angle_x = mech_x;
    control.mechanical_angle_y = mech_y;
    control.associated_angle_x = assoc_x;
    control.associated_angle_y = assoc_y;
    control.distance = cm;
    return control;
}

bool has_cmd(const std::vector<std::vector<uint8_t>>& frames, uint8_t cmd) {
    for (const std::vector<uint8_t>& frame : frames) {
        if (!frame.empty() && frame[0] == cmd) return true;
    }
    return false;
}

int count_cmd(const std::vector<std::vector<uint8_t>>& frames, uint8_t cmd) {
    int n = 0;
    for (const std::vector<uint8_t>& frame : frames) {
        if (!frame.empty() && frame[0] == cmd) ++n;
    }
    return n;
}

std::optional<std::pair<float, float>> p2p_number(const std::vector<std::vector<uint8_t>>& frames, int index) {
    int seen = 0;
    for (const std::vector<uint8_t>& frame : frames) {
        if (frame.size() < 11 || frame[0] != 0x2D) continue;
        if (seen++ != index) continue;
        float x = 0.f;
        float y = 0.f;
        std::memcpy(&x, frame.data() + 3, 4);
        std::memcpy(&y, frame.data() + 7, 4);
        return std::pair<float, float>{x, y};
    }
    return std::nullopt;
}

bool quiet_poll(const char* what) {
    const std::optional<ccu::SurveyResult> result = ccu::survey_poll();
    check(!result.has_value(), what);
    return !result.has_value();
}

bool arm_gimbal(float low_y) {
    ccu::lidar_push_status(status_of(0, kScanModeOff));
    if (!quiet_poll("enable waits")) return false;
    ccu::lidar_push_status(status_of(1, kScanModeOff));
    if (!quiet_poll("scan engine waits")) return false;
    ccu::lidar_push_status(status_of(1, kScanModePoint2Point));
    if (!quiet_poll("low target waits")) return false;
    const float ox = ccu::lidar_angle_offset_x();
    ccu::lidar_push_control(control_at(ox, low_y, ox, low_y, 0));
    return quiet_poll("high target waits");
}

ccu::PlanNode node_at(uint16_t id, float east, float north) {
    ccu::PlanNode node;
    node.id = id;
    node.east_m = east;
    node.north_m = north;
    node.corner_inner_angle_rad = 1.2f;
    node.corner_orientation_rad = 0.3f;
    return node;
}

void test_out_of_fov() {
    boot();
    ccu::survey_begin({node_at(2, 1.f, 0.f)});
    const std::optional<ccu::SurveyResult> result = ccu::survey_poll();
    check(result.has_value(), "out-of-wedge result");
    if (!result) return;
    check_eq_u16(result->id, 2, "out-of-wedge id");
    check(!result->surveyed && !result->success, "out-of-wedge fails");
    check(result->up_m == 0.f, "out-of-wedge up is 0");
    check(result->error_code == ccu::SurveyError::OutOfFov, "OUT_OF_FOV");
    check(result->survey_finished, "single out-of-wedge node finishes");
    const std::vector<std::vector<uint8_t>> commands = ccu::lidar_take_commands();
    check(!has_cmd(commands, 0x2D), "out-of-wedge sends no point-to-point command");
    check(!ccu::survey_node(2).has_value(), "out-of-wedge stores no Node");
}

void test_motor_window() {
    boot();
    load_cfg("ANGLE_MAX=0.90\nDWELL_TIME_S=0\n");
    ccu::survey_begin({node_at(3, 0.f, 2.f)});
    const std::optional<ccu::SurveyResult> result = ccu::survey_poll();
    check(result.has_value(), "window result");
    if (!result) return;
    check(result->error_code == ccu::SurveyError::NotVisible, "motor window is NOT_VISIBLE");
    check(!result->success && !result->surveyed, "motor window stores nothing");
    check(result->survey_finished, "motor window finishes");
    const std::vector<std::vector<uint8_t>> commands = ccu::lidar_take_commands();
    check(!has_cmd(commands, 0x2D), "motor window sends no point-to-point command");
    check(!has_cmd(commands, 0x01), "motor window leaves the motors disabled");
    check(!ccu::survey_node(3).has_value(), "motor window stores no Node");
}

std::optional<ccu::SurveyResult> sweep_node(const ccu::PlanNode& node, float hit_y, uint16_t hit_cm,
                                            const std::vector<ControlPayload>& dwell, bool expect_hit) {
    const float ox = ccu::lidar_angle_offset_x();
    const float oy = ccu::lidar_angle_offset_y();
    const float amax = ccu::angle_max_rad();
    const float low_y = oy - amax;
    const float high_y = oy + amax;
    ccu::lidar_test_reset();
    ccu::survey_begin({node});
    if (!arm_gimbal(low_y)) return std::nullopt;
    ccu::lidar_push_control(control_at(ox, hit_y, ox, hit_y, hit_cm));
    ccu::lidar_push_control(control_at(ox, high_y, ox, high_y, 0));
    if (expect_hit) {
        if (!quiet_poll("return waits")) return std::nullopt;
        ccu::lidar_push_control(control_at(ox, hit_y, ox, hit_y, 0));
        if (!quiet_poll("coarse waits")) return std::nullopt;
        ccu::lidar_push_status(status_of(1, kScanModeCoarse));
        for (const ControlPayload& sample : dwell) ccu::lidar_push_control(sample);
        return ccu::survey_poll();
    }
    return ccu::survey_poll();
}

void test_tolerance_miss_stores_nothing() {
    boot();
    const std::optional<ccu::SurveyResult> result = sweep_node(node_at(4, 0.f, 2.f), ccu::lidar_angle_offset_y(), 500, {}, false);
    check(result.has_value(), "tolerance result");
    if (!result) return;
    check(result->error_code == ccu::SurveyError::NotVisible, "outside SURVEY_TOL is NOT_VISIBLE");
    check(!result->surveyed && !result->success, "tolerance miss is not a survey");
    check(result->survey_finished, "tolerance miss finishes");
    check(!ccu::survey_node(4).has_value(), "tolerance miss stores no Node");
    const std::vector<std::vector<uint8_t>> commands = ccu::lidar_take_commands();
    check(count_cmd(commands, 0x2D) == 2, "tolerance miss stops after the upward sweep");
}

ControlPayload dwell_at(float azimuth, uint16_t cm, float elevation) {
    const ccu::LidarMotor motor = ccu::lidar_device_to_motor(azimuth, elevation);
    return control_at(motor.x, motor.y, motor.x, motor.y, cm);
}

void test_success_replaces_and_failure_keeps() {
    boot();
    const float ox = ccu::lidar_angle_offset_x();
    const float oy = ccu::lidar_angle_offset_y();
    const ccu::PlanNode node = node_at(1, 0.f, 2.f);
    const std::vector<ControlPayload> flat = {
        dwell_at(0.f, 200, 0.f),
        dwell_at(0.08f, 180, 0.f),
        dwell_at(-0.07f, 220, 0.f),
    };
    const std::optional<ccu::SurveyResult> first = sweep_node(node, oy, 200, flat, true);
    check(first.has_value(), "in-wedge result");
    if (!first) return;
    check(first->success && first->surveyed, "in-wedge sweep succeeds");
    check(first->survey_finished, "in-wedge surveyFinished");
    check_eq_u16(first->id, 1, "stored id");
    check_near(first->up_m, ccu::device_height_m(), "plan up is DEVICE_HEIGHT for a level return");
    check(first->corner_inner_angle_rad == 1.2f, "inner angle kept");
    check(first->corner_orientation_rad == 0.3f, "orientation kept");

    const std::vector<std::vector<uint8_t>> commands = ccu::lidar_take_commands();
    const std::optional<std::pair<float, float>> low = p2p_number(commands, 0);
    const std::optional<std::pair<float, float>> high = p2p_number(commands, 1);
    check(low.has_value() && high.has_value(), "upward sweep sends two point-to-point targets");
    if (low && high) {
        const ccu::LidarMotor low_motor = ccu::lidar_device_to_motor(0.f, -ccu::angle_max_rad());
        const ccu::LidarMotor high_motor = ccu::lidar_device_to_motor(0.f, ccu::angle_max_rad());
        check_near(low->first, low_motor.x, "low target X is device azimuth plus the lidar offset");
        check_near(low->second, low_motor.y, "low target Y is device elevation plus the lidar offset");
        check_near(high->first, high_motor.x, "high target X is device azimuth plus the lidar offset");
        check_near(high->second, high_motor.y, "high target Y is device elevation plus the lidar offset");
        check_near(low->first, ox, "low motor X is 3.540");
        check_near(high->second, oy + ccu::angle_max_rad(), "high motor Y is the offset plus ANGLE_MAX");
    }

    const std::optional<ccu::SurveyNode> stored = ccu::survey_node(1);
    check(stored.has_value(), "sweep stores a Node");
    if (stored) {
        check_eq_u16(stored->id, 1, "Node id");
        check_near(stored->up_m, ccu::device_height_m(), "Node up");
        check(stored->normal_up > 0.f, "localSlopeNormal up component is positive");
        check_near(stored->normal_up, 1.f, "level cloud normal points up");
    }

    const float elevation = 0.1f;
    const uint16_t cm = 201;
    const float raised = ccu::device_height_m() + (static_cast<float>(cm) * 0.01f) * std::sin(elevation);
    const std::vector<ControlPayload> lifted = {
        dwell_at(0.f, cm, elevation),
        dwell_at(0.05f, cm, elevation),
        dwell_at(-0.04f, cm, elevation),
    };
    const std::optional<ccu::SurveyResult> second =
        sweep_node(node, oy + elevation, cm, lifted, true);
    check(second.has_value() && second->success, "second success");
    const std::optional<ccu::SurveyNode> replaced = ccu::survey_node(1);
    check(replaced.has_value(), "second success keeps a Node");
    if (replaced) check_near(replaced->up_m, raised, "second success replaces the Node up");

    const float kept_up = replaced ? replaced->up_m : 0.f;
    const std::optional<ccu::SurveyResult> missed = sweep_node(node, oy, 500, {}, false);
    check(missed.has_value() && missed->error_code == ccu::SurveyError::NotVisible, "later miss is NOT_VISIBLE");
    const std::optional<ccu::SurveyNode> still = ccu::survey_node(1);
    check(still.has_value(), "failure leaves the previous Node");
    if (still) check_near(still->up_m, kept_up, "failure does not replace the Node");
}

}  // namespace

int main() {
    test_out_of_fov();
    test_motor_window();
    test_tolerance_miss_stores_nothing();
    test_success_replaces_and_failure_keeps();
    std::remove("ccu_survey_gate.cfg");
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_survey_test passed\n");
    return 0;
}
