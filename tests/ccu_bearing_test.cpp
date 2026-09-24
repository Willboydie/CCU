#include "bearing.hpp"

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
    check_near(ccu::bearing_from_north(0.f, 1000.f), 0.f, "+north bearing is 0");
    check_near(ccu::bearing_from_north(1000.f, 0.f), std::numbers::pi_v<float> / 2.f, "+east bearing is pi/2");
}

void test_fov() {
    constexpr float kPi = std::numbers::pi_v<float>;
    constexpr float kHalfPi = kPi / 2.f;
    check_near(ccu::kFovHalfRad, 35.f * kPi / 180.f, "half wedge is 35 degrees");
    check(ccu::in_fov(0.f, 1000.f, 0.f), "+north is in the wedge at yaw 0");
    check(!ccu::in_fov(1000.f, 0.f, 0.f), "+east is outside ±35° at yaw 0");

    const float outside = 40.f * kPi / 180.f;
    check(!ccu::in_fov(std::sin(outside) * 1000.f, std::cos(outside) * 1000.f, 0.f),
          "40 degrees is outside ±35");
    const float inside = 20.f * kPi / 180.f;
    check(ccu::in_fov(std::sin(inside) * 1000.f, std::cos(inside) * 1000.f, 0.f),
          "20 degrees is inside ±35");
    check(ccu::in_fov(1000.f, 0.f, kHalfPi), "+east is in the wedge at yaw pi/2");
    check(!ccu::in_fov(0.f, 1000.f, kHalfPi), "+north is outside ±35° at yaw pi/2");
    check(ccu::kFovRangeMm == 13000.f, "range limit is 13 m");
    check(!ccu::in_fov(0.f, 13050.f, 0.f), "beyond 13 m is out of FOV");
    check(ccu::in_fov(0.f, 13000.f, 0.f), "13 m on +north is in the wedge");
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
