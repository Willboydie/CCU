#include "device_config.hpp"
#include "lidar_model.hpp"

#include <cmath>
#include <cstdio>

namespace {

int g_fails = 0;

void check(bool cond, const char* what) {
    if (cond) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_fails;
}

void check_near(float got, float want, const char* what) {
    constexpr float kTol = 1e-4f;
    if (got >= want - kTol && got <= want + kTol) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

void test_neutral_and_inverse() {
    ccu::device_config_reset();
    check_near(ccu::lidar_angle_offset_x(), 3.540f, "motor X offset is 3.540");
    check_near(ccu::lidar_angle_offset_y(), 3.390f, "motor Y offset is 3.390");

    const ccu::LidarAngle neutral = ccu::lidar_motor_to_device(3.540f, 3.390f);
    check_near(neutral.azimuth, 0.f, "motor (3.540, 3.390) is azimuth 0");
    check_near(neutral.elevation, 0.f, "motor (3.540, 3.390) is elevation 0");

    const ccu::LidarMotor motor = ccu::lidar_device_to_motor(0.2f, 0.f);
    check_near(motor.x, 3.740f, "device azimuth +0.2 maps to motor X 3.740");
    check_near(motor.y, 3.390f, "level elevation keeps motor Y at the offset");

    const ccu::LidarAngle back = ccu::lidar_motor_to_device(motor.x, motor.y);
    check_near(back.azimuth, 0.2f, "inverse recovers azimuth +0.2");
    check_near(back.elevation, 0.f, "inverse recovers elevation 0");
}

void test_level_return_has_no_position_offset() {
    ccu::device_config_reset();
    const std::optional<ccu::LidarPoint> device = ccu::lidar_device_point(0.f, 0.f, 2.f);
    check(device.has_value(), "a 2 m return is a point");
    if (!device) return;
    check_near(device->up_m, 0.f, "level 2 m return has device up 0");
    check_near(device->north_m, 2.f, "level 2 m return has device north 2");
    check_near(device->east_m, 0.f, "level boresight has device east 0");

    const std::optional<ccu::LidarPoint> plan = ccu::lidar_plan_point(0.f, 0.f, 2.f, 0.f);
    check(plan.has_value(), "plan point exists");
    if (!plan) return;
    check_near(plan->up_m, ccu::device_height_m(), "yaw 0 plan up is DEVICE_HEIGHT");
    check_near(plan->north_m, 2.f, "yaw 0 plan north stays 2");
    check_near(plan->east_m, 0.f, "yaw 0 plan east stays 0");
    check(!ccu::lidar_device_point(0.f, 0.f, 0.f).has_value(), "distance 0 is ignored");
}

}  // namespace

int main() {
    test_neutral_and_inverse();
    test_level_return_has_no_position_offset();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_lidar_model_test passed\n");
    return 0;
}
