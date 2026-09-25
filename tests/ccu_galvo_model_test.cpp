#include "device_config.hpp"
#include "galvo_model.hpp"

#include <cmath>
#include <cstdio>

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
    constexpr float kTol = 1e-5f;
    if (got >= want - kTol && got <= want + kTol) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

float arccotangent(float n) {
    constexpr float kPi = 3.14159265358979323846f;
    if (n == 0.f) return kPi / 2.f;
    return std::atan2(1.f, n);
}

ccu::GalvoMirrors oracle_mirrors(float north, float east, float up) {
    constexpr float kPi = 3.14159265358979323846f;
    const float h = ccu::kGalvoMirrorSeparationM;
    const float y = arccotangent((h - up) / north) - kPi / 2.f;
    const float along = std::sin(kPi / 2.f + y);
    const float x = (arccotangent((east * along) / (north + h * along)) - kPi / 2.f) / -1.f;
    return {y, x};
}

ccu::GalvoDac oracle_plan(float plan_east, float plan_north, float plan_up, float yaw) {
    float north = plan_east * std::sin(yaw) + plan_north * std::cos(yaw);
    float east = plan_east * std::cos(yaw) - plan_north * std::sin(yaw);
    float up = plan_up;
    north += ccu::lidar_north_offset_m();
    east += ccu::lidar_east_offset_m();
    up += -ccu::device_height_m() + ccu::lidar_up_offset_m();

    const float ox = ccu::galvo_mech_offset_x();
    const float oy = ccu::galvo_mech_offset_y();
    const float east1 = east * std::cos(ox) - north * std::sin(ox);
    const float north1 = east * std::sin(ox) + north * std::cos(ox);
    const float up1 = up;
    const float east2 = east1 * std::cos(oy) + up1 * std::sin(oy);
    const float north2 = north1;
    const float up2 = -east1 * std::sin(oy) + up1 * std::cos(oy);
    const ccu::GalvoMirrors mirrors = oracle_mirrors(north2, east2, up2);
    return {ccu::galvo_dac_from_mirror(mirrors.x_rad), ccu::galvo_dac_from_mirror(mirrors.y_rad)};
}

void test_separation_and_mirror_order() {
    check(ccu::kGalvoMirrorSeparationM == 0.015f, "h is 0.015 m");
    const ccu::GalvoMirrors got = ccu::galvo_mirror_angles(1.f, 0.2f, 0.05f);
    const ccu::GalvoMirrors want = oracle_mirrors(1.f, 0.2f, 0.05f);
    check_near(got.y_rad, want.y_rad, "Y mirror is computed first");
    check_near(got.x_rad, want.x_rad, "X mirror uses the Y angle");
    check(got.x_rad != got.y_rad, "the two mirrors are not the same angle");
}

void test_trunc_and_clamp() {
    const float scale = ccu::galvo_rad_per_dac();
    const float plus = static_cast<float>(1.9 * static_cast<double>(scale));
    const float minus = static_cast<float>(-1.9 * static_cast<double>(scale));
    const float fraction = static_cast<float>(0.5 * static_cast<double>(scale));
    check_eq_u16(ccu::galvo_dac_from_mirror(plus), 32769, "1.9 truncates toward zero to +1");
    check_eq_u16(ccu::galvo_dac_from_mirror(minus), 32767, "-1.9 truncates toward zero to -1");
    check_eq_u16(ccu::galvo_dac_from_mirror(fraction), 32768, "0.5 truncates toward zero, not to 1");
    check_eq_u16(ccu::galvo_dac_from_mirror(100.f), 65535, "large angle clamps at 65535");
    check_eq_u16(ccu::galvo_dac_from_mirror(-100.f), 0, "large negative angle clamps at 0");
}

void test_lidar_shift_moves_boresight() {
    ccu::device_config_reset();
    const float height = ccu::device_height_m();
    const ccu::GalvoDac dac = ccu::galvo_plan_to_dac(0.f, 5.f, height, 0.f);
    check(!(dac.x == 32768 && dac.y == 32768),
          "yaw 0 plan (0, 5, DEVICE_HEIGHT) is not DAC boresight");
    const ccu::GalvoDac want = oracle_plan(0.f, 5.f, height, 0.f);
    check_eq_u16(dac.x, want.x, "shifted DAC X");
    check_eq_u16(dac.y, want.y, "shifted DAC Y");
}

}  // namespace

int main() {
    test_separation_and_mirror_order();
    test_trunc_and_clamp();
    test_lidar_shift_moves_boresight();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_galvo_model_test passed\n");
    return 0;
}
