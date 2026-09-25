#include "bearing.hpp"
#include "device_config.hpp"

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
    constexpr float kTol = 1e-5f;
    if (got >= want - kTol && got <= want + kTol) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

void test_bearings() {
    check_near(ccu::bearing_from_north(0.f, 1.f), 0.f, "+north bearing is 0");
    check_near(ccu::bearing_from_north(1.f, 0.f), std::numbers::pi_v<float> / 2.f, "+east bearing is pi/2");
}

void test_fov() {
    ccu::device_config_reset();
    constexpr float kPi = std::numbers::pi_v<float>;
    constexpr float kHalfPi = kPi / 2.f;
    check_near(ccu::angle_max_rad(), 0.40f, "half wedge is 0.40 rad");
    check(ccu::d_max_m() == 20.f, "range limit is 20 m");
    check(ccu::in_fov(0.f, 1.f, 0.f), "+north is in the wedge at yaw 0");
    check(!ccu::in_fov(1.f, 0.f, 0.f), "+east is outside ±0.40 rad at yaw 0");

    const float outside = 0.42f;
    check(!ccu::in_fov(std::sin(outside), std::cos(outside), 0.f), "just outside ±0.40 rad is out of FOV");
    const float inside = 0.38f;
    check(ccu::in_fov(std::sin(inside), std::cos(inside), 0.f), "inside ±0.40 rad is in the wedge");
    check(ccu::in_fov(0.f, 20.f, 0.f), "20 m on +north is in the wedge");
    check(!ccu::in_fov(0.f, 20.05f, 0.f), "just past 20 m is out of FOV");
    check(ccu::in_fov(1.f, 0.f, kHalfPi), "+east is in the wedge at yaw pi/2");
    check(!ccu::in_fov(0.f, 1.f, kHalfPi), "+north is outside ±0.40 rad at yaw pi/2");
}

}  // namespace

int main() {
    test_bearings();
    test_fov();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_bearing_test passed\n");
    return 0;
}
