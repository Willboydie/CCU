#include "device_config.hpp"
#include "lidar.hpp"
#include "lidar_model.hpp"
#include "path_engine.hpp"
#include "project.hpp"
#include "projector.hpp"
#include "protocol.hpp"
#include "session.hpp"
#include "survey.hpp"
#include "utils/protocol.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <numbers>
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

void send(const std::optional<std::vector<uint8_t>>& frame) {
    check(frame.has_value(), "frame packed");
    if (frame) ccu::session_on_frame(*frame);
}

void boot() {
    ccu::session_reset();
    ccu::lidar_test_reset();
    ccu::survey_reset();
    ccu::projector_test_reset();
    const char* path = "ccu_project_gate.cfg";
    std::ofstream out(path);
    out << "DWELL_TIME_S=0\n";
    out.close();
    ccu::device_config_load(path);
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

bool plant(const ccu::PlanNode& node) {
    const float ox = ccu::lidar_angle_offset_x();
    const float oy = ccu::lidar_angle_offset_y();
    const float low_y = oy - ccu::angle_max_rad();
    const float high_y = oy + ccu::angle_max_rad();
    ccu::lidar_test_reset();
    ccu::survey_begin({node});
    ccu::lidar_push_status(status_of(0, kScanModeOff));
    if (ccu::survey_poll()) return false;
    ccu::lidar_push_status(status_of(1, kScanModeOff));
    if (ccu::survey_poll()) return false;
    ccu::lidar_push_status(status_of(1, kScanModePoint2Point));
    if (ccu::survey_poll()) return false;
    ccu::lidar_push_control(control_at(ox, low_y, ox, low_y, 0));
    if (ccu::survey_poll()) return false;
    ccu::lidar_push_control(control_at(ox, oy, ox, oy, 200));
    ccu::lidar_push_control(control_at(ox, high_y, ox, high_y, 0));
    if (ccu::survey_poll()) return false;
    ccu::lidar_push_control(control_at(ox, oy, ox, oy, 0));
    if (ccu::survey_poll()) return false;
    ccu::lidar_push_status(status_of(1, kScanModeCoarse));
    const float az[3] = {0.f, 0.08f, -0.07f};
    const uint16_t cm[3] = {200, 180, 220};
    for (int i = 0; i < 3; ++i) {
        const ccu::LidarMotor motor = ccu::lidar_device_to_motor(az[i], 0.f);
        ccu::lidar_push_control(control_at(motor.x, motor.y, motor.x, motor.y, cm[i]));
    }
    const std::optional<ccu::SurveyResult> result = ccu::survey_poll();
    return result.has_value() && result->success;
}

ccu::PlanNode make_node(uint16_t id, float east, float north) {
    ccu::PlanNode node;
    node.id = id;
    node.east_m = east;
    node.north_m = north;
    node.corner_inner_angle_rad = std::numbers::pi_v<float> / 2.f;
    node.corner_orientation_rad = 0.f;
    return node;
}

struct StreamPoint {
    uint16_t x = 0;
    uint16_t y = 0;
    bool laser_on = false;
};

std::vector<StreamPoint> read_stream(const std::vector<std::vector<uint8_t>>& frames) {
    std::vector<StreamPoint> points;
    for (const std::vector<uint8_t>& frame : frames) {
        if (frame.size() < 4 || frame[0] != 0xAA) continue;
        const int len = frame[1] | (frame[2] << 8);
        if (len < 1 || frame.size() < static_cast<std::size_t>(3 + len)) continue;
        for (int i = 1; i + 5 <= len; i += 5) {
            const uint8_t* rec = frame.data() + 3 + i;
            StreamPoint point;
            point.x = static_cast<uint16_t>((rec[0] << 8) | rec[1]);
            point.y = static_cast<uint16_t>((rec[2] << 8) | rec[3]);
            point.laser_on = rec[4] != 0;
            points.push_back(point);
        }
    }
    return points;
}

bool has_cmd(const std::vector<std::vector<uint8_t>>& frames, uint8_t cmd) {
    for (const std::vector<uint8_t>& frame : frames)
        if (!frame.empty() && frame[0] == cmd) return true;
    return false;
}

void test_projection() {
    boot();
    const ccu::PlanNode surveyed = make_node(1, 0.f, 2.f);
    const ccu::PlanNode turned = make_node(2, 2.f, 0.f);
    const ccu::PlanNode missing = make_node(9, 0.2f, 2.f);
    check(plant(surveyed), "G12 transport stores the in-wedge node");

    ccu::session_set_stage(-std::numbers::pi_v<float> / 2.f);
    check(plant(turned), "G12 transport stores the east node at yaw pi/2");
    ccu::session_set_stage(0.f);

    ccu::projector_test_reset();
    const ccu::ProjectionReady ready = ccu::project_begin({surveyed, missing, turned});
    check(ready.nodes.size() == 1, "one node is drawn");
    if (!ready.nodes.empty()) check_eq_u16(ready.nodes[0].id, 1, "drawn id");
    check(ready.errors.size() == 2, "two nodes are omitted");
    bool saw_missing = false;
    bool saw_fov = false;
    for (const ccu::NodeError& error : ready.errors) {
        if (error.id == 9 && error.error_code == 4) saw_missing = true;
        if (error.id == 2 && error.error_code == 2) saw_fov = true;
    }
    check(saw_missing, "unsurveyed id is NOT_SURVEYED");
    check(saw_fov, "node outside the frozen wedge is OUT_OF_FOV");

    const std::optional<ccu::SurveyNode> stored = ccu::survey_node(1);
    check(stored.has_value(), "planted node is still stored");
    if (!stored) return;
    const ccu::PathNode draw{
        stored->east_m,
        stored->north_m,
        stored->up_m,
        stored->corner_inner_angle_rad,
        stored->corner_orientation_rad,
    };
    const std::vector<ccu::DacPoint> want = ccu::path_engine({draw}, 0.f);
    const std::vector<ccu::DacPoint>& got = ccu::project_path();
    check(got.size() == want.size(), "drawn list is the G8 DAC list");
    bool same = got.size() == want.size();
    int laser_on = 0;
    bool closed = false;
    for (std::size_t i = 0; same && i < got.size(); ++i) {
        same = got[i].x == want[i].x && got[i].y == want[i].y && got[i].laser_on == want[i].laser_on;
        if (got[i].laser_on) {
            ++laser_on;
            if (closed) same = false;
        } else {
            closed = true;
        }
    }
    check(same, "DAC pairs and laser flags match G8");
    check(laser_on > 0 && closed && !got.empty() && !got.back().laser_on,
          "laser on for the arms and laser off for the closing join");

    std::vector<std::vector<uint8_t>> frames;
    std::vector<StreamPoint> records;
    for (int i = 0; i < 30 && records.size() < got.size(); ++i) {
        ccu::project_poll();
        const std::vector<std::vector<uint8_t>> more = ccu::projector_take_frames();
        frames.insert(frames.end(), more.begin(), more.end());
        const std::vector<StreamPoint> parsed = read_stream(more);
        records.insert(records.end(), parsed.begin(), parsed.end());
    }
    check(has_cmd(frames, 0xF7), "stream starts with RESET_STREAM");
    check(has_cmd(frames, 0x1B), "stream sets the laser duty");
    check(has_cmd(frames, 0x07), "stream sends PLAY");
    check(records.size() >= got.size(), "POINT_STREAM carries the DAC list");
    bool stream_matches = records.size() >= got.size();
    for (std::size_t i = 0; stream_matches && i < got.size(); ++i) {
        stream_matches = records[i].x == got[i].x && records[i].y == got[i].y &&
                         records[i].laser_on == got[i].laser_on;
    }
    check(stream_matches, "recorded stream frames carry those DAC pairs");
    check_eq_u8(ccu::kLaserDutyDefault, 30, "default duty stays 30");
}

}  // namespace

int main() {
    test_projection();
    std::remove("ccu_project_gate.cfg");
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_project_test passed\n");
    return 0;
}
