#include "lidar_model.hpp"

#include "scan/Geometry.hpp"

#include <cmath>
#include <cstdio>
#include <numbers>

namespace {

int g_fails = 0;

void check(bool cond, const char* what) {
    if (cond) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_fails;
}

void check_near(float got, float want, const char* what) {
    constexpr float kTol = 1e-3f;
    if (got >= want - kTol && got <= want + kTol) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

void test_neutral_is_boresight() {
    const ccu::LidarAngle neutral = ccu::lidar_motor_to_angle(kMechNeutralX, kMechNeutralY);
    check(neutral.azimuth == 0.f, "neutral motor azimuth is 0");
    check(neutral.elevation == 0.f, "neutral motor elevation is 0");

    const ccu::LidarAngle shifted = ccu::lidar_motor_to_angle(kMechNeutralX + 0.2f, kMechNeutralY - 0.1f);
    check_near(shifted.azimuth, 0.2f, "azimuth is motor X minus neutral");
    check_near(shifted.elevation, -0.1f, "elevation is motor Y minus neutral");
}

void test_up_from_range_and_elevation() {
    check(ccu::lidar_up_mm(800.f, 0.f) == 0.f, "level beam has up 0");

    const float half_pi = std::numbers::pi_v<float> / 2.f;
    check_near(ccu::lidar_up_mm(1500.f, half_pi), 1500.f, "straight up equals the range");
    check_near(ccu::lidar_up_mm(400.f, -half_pi), -400.f, "below horizontal up is negative");

    const float elevation = std::asin(0.5f);
    check_near(ccu::lidar_up_mm(2000.f, elevation), 1000.f, "range 2000 at sin 0.5 is up 1000");
}

void test_origin_is_zero() {
    const ccu::LidarOrigin origin = ccu::lidar_origin_mm();
    check(origin.x_mm == 0.f, "origin east is 0");
    check(origin.y_mm == 0.f, "origin north is 0");
    check(origin.z_mm == 0.f, "origin up is 0");
    check(ccu::kFloorUpMm == -150.f, "missing range uses floor up -150");
}

}  // namespace

int main() {
    test_neutral_is_boresight();
    test_up_from_range_and_elevation();
    test_origin_is_zero();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_lidar_model_test passed\n");
    return 0;
}
