#include "yaw.hpp"

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

void test_equal_angles_are_zero() {
    check(ccu::effective_yaw(0.f, 0.f) == 0.f, "zeros yaw 0");
    check(ccu::effective_yaw(1.2f, 1.2f) == 0.f, "equal angles yaw 0");
    check(ccu::effective_yaw(-0.4f, -0.4f) == 0.f, "equal negative angles yaw 0");
}

void test_clockwise_quarter_turn() {
    constexpr float kHalfPi = std::numbers::pi_v<float> / 2.f;
    const float north = 1.2f;
    const float stage = north - kHalfPi;
    check_near(ccu::effective_yaw(north, stage), kHalfPi, "stage decrease of pi/2 yields yaw pi/2");
    check_near(ccu::effective_yaw(0.f, -kHalfPi), kHalfPi, "stage -pi/2 from north 0");
}

void test_wrap_into_two_pi() {
    constexpr float kPi = std::numbers::pi_v<float>;
    constexpr float kTwoPi = 2.f * kPi;
    constexpr float kHalfPi = kPi / 2.f;

    check(ccu::wrap_2pi(0.f) == 0.f, "wrap 0");
    check(ccu::wrap_2pi(kTwoPi) == 0.f, "wrap 2pi is 0");
    check(ccu::wrap_2pi(-kTwoPi) == 0.f, "wrap -2pi is 0");
    check_near(ccu::wrap_2pi(kTwoPi + 0.3f), 0.3f, "wrap past 2pi");
    check_near(ccu::wrap_2pi(-0.3f), kTwoPi - 0.3f, "wrap negative");
    check_near(ccu::wrap_2pi(-kTwoPi - 0.3f), kTwoPi - 0.3f, "wrap an extra negative turn");
    check_near(ccu::wrap_2pi(7.f), 7.f - kTwoPi, "wrap uses 2pi, not degrees");

    const float anticlockwise = ccu::effective_yaw(0.f, 0.1f);
    check(anticlockwise >= 0.f && anticlockwise < kTwoPi, "small stage increase stays in range");
    check_near(anticlockwise, kTwoPi - 0.1f, "stage increase wraps just under 2pi");

    const float opposite = ccu::effective_yaw(0.f, kHalfPi);
    check(opposite >= 0.f && opposite < kTwoPi, "anticlockwise quarter turn stays in range");
    check_near(opposite, kTwoPi - kHalfPi, "stage increase of pi/2 yields 3pi/2");

    const float extra_turn = ccu::effective_yaw(1.2f + kTwoPi, 1.2f);
    const float to_north = extra_turn < kTwoPi - extra_turn ? extra_turn : kTwoPi - extra_turn;
    check(to_north < 1e-5f, "full extra turn of north offset is due north");

    const float samples[] = {-9.f, -0.2f, 0.f, 0.2f, 3.f, 6.f, 9.f};
    for (float north : samples) {
        for (float stage : samples) {
            const float yaw = ccu::effective_yaw(north, stage);
            check(yaw >= 0.f && yaw < kTwoPi, "effective yaw in [0, 2pi)");
        }
    }
}

}  // namespace

int main() {
    test_equal_angles_are_zero();
    test_clockwise_quarter_turn();
    test_wrap_into_two_pi();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_yaw_test passed\n");
    return 0;
}
