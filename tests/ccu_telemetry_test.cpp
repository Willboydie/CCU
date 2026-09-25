#include "device_config.hpp"
#include "protocol.hpp"
#include "session.hpp"
#include "telemetry.hpp"

#include <cstdio>
#include <fstream>
#include <numbers>

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

void send(const std::optional<std::vector<uint8_t>>& frame) {
    check(frame.has_value(), "frame packed");
    if (frame) ccu::session_on_frame(*frame);
}

void check_promo_fields(const ccu::RegularTelemetry& msg, float yaw) {
    check(msg.device_id == "sm-ccu", "device id");
    check(msg.session_id == ccu::session_id(), "session id from the getter");
    check(msg.origin_is_set, "originIsSet");
    check_eq_f(msg.pose.east_m, 0.f, "pose east");
    check_eq_f(msg.pose.north_m, 0.f, "pose north");
    check_eq_f(msg.pose.up_m, ccu::device_height_m(), "pose up is DEVICE_HEIGHT");
    check_eq_f(msg.pose.roll_rad, 0.f, "pose roll");
    check_eq_f(msg.pose.pitch_rad, 0.f, "pose pitch");
    check_eq_f(msg.pose.yaw_rad, 0.f, "pose yaw");
    check_near(msg.effective_yaw, yaw, "effective yaw from G10");
    check(!msg.moved, "moved is 0");
    check(msg.origin_set_status == ccu::OriginSetStatus::Idle, "originSetStatus is 0");
    check(msg.tag_finding_status == ccu::TagFindingStatus::PoseKnown, "tagFindingStatus is 0");
    check(msg.survey_status == ccu::SurveyStatus::Idle, "surveyStatus idle");
    check(msg.projector_status == ccu::ProjectorStatus::Idle, "projectorStatus idle");
    check(!msg.projector_running, "projectorRunning");
    check(!msg.laser_on, "laserOn");
    check(msg.laser_power == ccu::kLaserDutyDefault, "default laser power");
}

void test_telemetry_from_g10_session() {
    ccu::session_reset();
    send(ccu::pack_begin_new_session());
    constexpr float kStage = 1.2f;
    ccu::session_set_stage(kStage);
    send(ccu::pack_begin_origin_set());
    const float turned = kStage - std::numbers::pi_v<float> / 2.f;
    ccu::session_set_stage(turned);

    const float yaw = std::numbers::pi_v<float> / 2.f;
    const ccu::RegularTelemetry msg = ccu::build_regular_telemetry();
    check_promo_fields(msg, yaw);

    const auto frame = ccu::pack_regular_telemetry(msg);
    check(frame.has_value(), "telemetry packs");
    if (!frame) return;
    const auto unpacked = ccu::unpack_regular_telemetry(*frame);
    check(unpacked.has_value(), "telemetry unpacks");
    if (!unpacked) return;
    check_promo_fields(*unpacked, yaw);
}

void write_cfg(const char* path, const char* body) {
    std::ofstream out(path);
    check(static_cast<bool>(out), "config file opens");
    out << body;
}

void test_device_height_config() {
    check_eq_f(ccu::device_height_m(), ccu::kDeviceHeightDefaultM, "default height is 1.365 m");

    const char* path = "ccu_device_height_test.cfg";
    write_cfg(path, "# comment\nDEVICE_HEIGHT=1.5\n");
    const ccu::DeviceConfigStatus loaded = ccu::device_config_load(path);
    check(loaded.opened && loaded.height_set, "DEVICE_HEIGHT loaded");
    check_eq_f(ccu::device_height_m(), 1.5f, "loaded height metres");

    write_cfg(path, "DEVICE_HEIGHT=nope\n");
    const ccu::DeviceConfigStatus invalid = ccu::device_config_load(path);
    check(invalid.opened && !invalid.height_set, "bad DEVICE_HEIGHT is ignored");
    check_eq_f(ccu::device_height_m(), ccu::kDeviceHeightDefaultM, "bad value keeps the default");

    write_cfg(path, "D_MAX_M=13\n");
    const ccu::DeviceConfigStatus missing = ccu::device_config_load(path);
    check(missing.opened && !missing.height_set, "missing key keeps the default");
    check_eq_f(ccu::device_height_m(), ccu::kDeviceHeightDefaultM, "missing key height");

    const ccu::DeviceConfigStatus absent = ccu::device_config_load("ccu_device_height_missing.cfg");
    check(!absent.opened && !absent.height_set, "missing file is reported");
    check_eq_f(ccu::device_height_m(), ccu::kDeviceHeightDefaultM, "missing file keeps the default");
    std::remove(path);
}

}  // namespace

int main() {
    test_telemetry_from_g10_session();
    test_device_height_config();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_telemetry_test passed\n");
    return 0;
}
