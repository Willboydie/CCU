#include "galvo_model.hpp"

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

void test_boresight_is_32768() {
    const ccu::GalvoAngle angle = ccu::galvo_dac_to_angle(ccu::kGalvoDacBoresight, ccu::kGalvoDacBoresight);
    check(angle.azimuth == 0.f, "boresight azimuth is 0");
    check(angle.elevation == 0.f, "boresight elevation is 0");

    const ccu::GalvoDac dac = ccu::galvo_angle_to_dac(0.f, 0.f);
    check_eq_u16(dac.x, ccu::kGalvoDacBoresight, "zero azimuth is DAC 32768");
    check_eq_u16(dac.y, ccu::kGalvoDacBoresight, "zero elevation is DAC 32768");
}

void test_round_trip() {
    const ccu::GalvoDac in{40000, 25000};
    const ccu::GalvoAngle angle = ccu::galvo_dac_to_angle(in.x, in.y);
    check_near(angle.azimuth, (40000 - 32768) * ccu::kGalvoRadPerDac, "azimuth scale");
    check_near(angle.elevation, (25000 - 32768) * ccu::kGalvoRadPerDac, "elevation scale");
    check(angle.azimuth > 0.f, "DAC above 32768 is positive azimuth");
    check(angle.elevation < 0.f, "DAC below 32768 is negative elevation");

    const ccu::GalvoDac out = ccu::galvo_angle_to_dac(angle.azimuth, angle.elevation);
    check_eq_u16(out.x, in.x, "DAC X round trip");
    check_eq_u16(out.y, in.y, "DAC Y round trip");

    const ccu::GalvoDac ends = ccu::galvo_angle_to_dac(
        ccu::galvo_dac_to_angle(0, 65535).azimuth,
        ccu::galvo_dac_to_angle(0, 65535).elevation);
    check_eq_u16(ends.x, 0, "DAC 0 round trip");
    check_eq_u16(ends.y, 65535, "DAC 65535 round trip");
}

void test_clamp() {
    const ccu::GalvoDac low = ccu::galvo_angle_to_dac(-4.f, -4.f);
    check_eq_u16(low.x, 0, "azimuth clamps at 0");
    check_eq_u16(low.y, 0, "elevation clamps at 0");

    const ccu::GalvoDac high = ccu::galvo_angle_to_dac(4.f, 4.f);
    check_eq_u16(high.x, 65535, "azimuth clamps at 65535");
    check_eq_u16(high.y, 65535, "elevation clamps at 65535");
}

void test_point_atan2_mirror_offset_zero() {
    const ccu::GalvoAngle ahead = ccu::galvo_point_to_angle(1.f, 0.f, 0.f);
    check_near(ahead.azimuth, 0.f, "straight ahead azimuth");
    check_near(ahead.elevation, 0.f, "straight ahead elevation");
    const ccu::GalvoDac boresight = ccu::galvo_angle_to_dac(ahead.azimuth, ahead.elevation);
    check_eq_u16(boresight.x, ccu::kGalvoDacBoresight, "ahead point is DAC boresight X");
    check_eq_u16(boresight.y, ccu::kGalvoDacBoresight, "ahead point is DAC boresight Y");

    constexpr float kHalfPi = std::numbers::pi_v<float> / 2.f;
    const ccu::GalvoAngle left = ccu::galvo_point_to_angle(0.f, 1.f, 0.f);
    check_near(left.azimuth, kHalfPi, "atan2 azimuth");
    check_near(left.elevation, 0.f, "level point elevation");

    const ccu::GalvoAngle up = ccu::galvo_point_to_angle(1.f, 0.f, 1.f);
    check_near(up.azimuth, 0.f, "up point azimuth");
    check_near(up.elevation, std::atan(1.f), "atan2 elevation");
}

}  // namespace

int main() {
    test_boresight_is_32768();
    test_round_trip();
    test_clamp();
    test_point_atan2_mirror_offset_zero();
    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("ccu_galvo_model_test passed\n");
    return 0;
}
