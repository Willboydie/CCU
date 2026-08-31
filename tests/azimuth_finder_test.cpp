#include "scan/AzimuthPointFinder.hpp"
#include "scan/DwellPointCloud.hpp"
#include "scan/Geometry.hpp"
#include "scan/PointCloud.hpp"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

namespace {

int g_fails = 0;

void check(bool cond, const char* what) {
    if (cond) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_fails;
}

void check_near(float got, float want, float tol, const char* what) {
    if (std::fabs(got - want) <= tol) return;
    std::fprintf(stderr, "FAIL: %s (got %f want %f)\n", what, got, want);
    ++g_fails;
}

LidarPoint make_pt(float az, float el, float range) {
    LidarPoint p;
    p.azimuth = az;
    p.elevation = el;
    p.range = range;
    p.valid = range > 0.0f;
    p.distance_cm = static_cast<uint16_t>(std::lround(range / kCmToMetres));
    boresight_to_mechanical(az, el, p.mechanical_x, p.mechanical_y);
    spherical_to_cartesian(range, az, el, p.x, p.y, p.z);
    return p;
}

PointCloud fill_strip(float r0, float r1, float az, float el_lo, float el_hi,
                      float step, float skip_lo = 0, float skip_hi = 0,
                      bool use_skip = false) {
    PointCloud cloud;
    for (float el = el_lo; el <= el_hi; el += step) {
        if (use_skip && el >= skip_lo && el <= skip_hi)
            continue;
        const float t = (el - el_lo) / (el_hi - el_lo);
        cloud.add(make_pt(az, el, r0 + t * (r1 - r0)));
    }
    return cloud;
}

void test_sort_left_to_right() {
    std::vector<TargetXY> pts = {{3.0f, -1.0f}, {3.0f, 1.0f}, {4.0f, 0.0f}};
    AzimuthPointFinder::sort_left_to_right(pts);
    check(pts[0].y > pts[1].y && pts[1].y > pts[2].y, "left-to-right is descending azimuth");
}

void test_bracket_picks_lowest_elevation() {
    AzimuthPointFinder finder;
    const auto& cfg = finder.config();
    PointCloud strip = fill_strip(2.5f, 5.5f, 0.0f, cfg.elev_min, cfg.elev_max, 0.002f);
    const StripAnalysis a = finder.analyse(3.2f, 0.0f, strip);
    check(a.hit.visibility == PointVisibility::Found, "smooth crossing is Found");
    check(a.hit.bin_index >= 0, "Found has a bin");
    check_near(a.hit.range, a.hit.r_target, cfg.range_match_tol_m, "chosen bin near r_target");

    int other = -1;
    for (std::size_t t = 1; t < a.bins.size(); ++t) {
        const ElevationBin& lo = a.bins[t - 1];
        const ElevationBin& hi = a.bins[t];
        if (!lo.occupied || !hi.occupied)
            continue;
        const bool straddle =
            (lo.residual <= 0.0f && hi.residual >= 0.0f) ||
            (lo.residual >= 0.0f && hi.residual <= 0.0f);
        if (!straddle)
            continue;
        if (std::fabs(lo.residual) > cfg.range_match_tol_m ||
            std::fabs(hi.residual) > cfg.range_match_tol_m)
            continue;
        other = hi.index;
        check(a.hit.bin_index == lo.index, "bracket uses the lower-elevation bin");
        check(a.hit.elevation <= hi.elevation, "chosen elevation is the lower of the pair");
        break;
    }
    check(other >= 0, "test found a bracketing pair");
}

void test_surface_in_front_is_not_visible() {
    AzimuthPointFinder finder;
    const auto& cfg = finder.config();
    PointCloud strip = fill_strip(1.0f, 1.0f, 0.0f, cfg.elev_min, cfg.elev_max, 0.002f);
    const PixelHit hit = finder.find(3.2f, 0.0f, strip);
    check(hit.visibility == PointVisibility::NotVisible, "nearer surface is NOT_VISIBLE");
}

void test_surface_behind_is_not_visible() {
    AzimuthPointFinder finder;
    const auto& cfg = finder.config();
    PointCloud strip = fill_strip(10.0f, 10.0f, 0.0f, cfg.elev_min, cfg.elev_max, 0.002f);
    const PixelHit hit = finder.find(3.2f, 0.0f, strip);
    check(hit.visibility == PointVisibility::NotVisible, "farther surface is NOT_VISIBLE");
}

void test_gap_at_crossing_is_not_visible() {
    AzimuthPointFinder finder;
    const auto& cfg = finder.config();
    PointCloud strip = fill_strip(2.5f, 5.5f, 0.0f, cfg.elev_min, cfg.elev_max, 0.002f,
                                  cfg.elev_min + 0.08f, cfg.elev_max - 0.08f, true);
    const StripAnalysis a = finder.analyse(3.2f, 0.0f, strip);
    check(a.hit.visibility == PointVisibility::NotVisible, "gap at crossing is NOT_VISIBLE");
}

void test_range_jump_at_crossing_is_not_visible() {
    AzimuthPointFinder finder;
    const auto& cfg = finder.config();
    PointCloud strip;
    for (float el = cfg.elev_min; el <= cfg.elev_max; el += 0.002f) {
        const float r = (el < 0.0f) ? 1.0f : 8.0f;
        strip.add(make_pt(0.0f, el, r));
    }
    const PixelHit hit = finder.find(3.2f, 0.0f, strip);
    check(hit.visibility == PointVisibility::NotVisible, "range jump at crossing is NOT_VISIBLE");
}

void test_azimuth_gate_drops_off_strip_samples() {
    AzimuthPointFinder finder;
    const auto& cfg = finder.config();
    PointCloud strip = fill_strip(2.5f, 5.5f, 0.4f, cfg.elev_min, cfg.elev_max, 0.002f);
    const StripAnalysis a = finder.analyse(3.2f, 0.0f, strip);
    check(a.gated_samples == 0, "samples far from target azimuth are ignored");
    check(a.hit.visibility == PointVisibility::NotVisible, "empty gated strip is NOT_VISIBLE");
}

void test_dwell_z_stub_and_binary_roundtrip() {
    DwellPointCloud d;
    d.set_centre(0.10f, -0.20f);
    d.set_centre_mechanical(3.64f, 3.19f);
    d.set_target_xy(3.0f, 0.25f);
    DwellParams p;
    p.amp_x = 0.1f;
    p.amp_y = 0.08f;
    p.freq_x = 2.0f;
    p.freq_y = 4.0f;
    p.duration_s = 1.5f;
    d.set_params(p);
    d.add(make_pt(0.10f, -0.20f, 3.50f));
    check(!d.estimate_z().has_value(), "estimate_z is empty until defined");

    const char* path = "dwell_roundtrip_test.bin";
    std::string err;
    check(d.save_binary(path, err), "save dwell binary");
    DwellPointCloud loaded;
    check(DwellPointCloud::load_binary(path, loaded, err), "load dwell binary");
    check(loaded.size() == 1, "loaded count");
    check_near(loaded.target_x(), 3.0f, 1e-5f, "target x");
    check_near(loaded.target_y(), 0.25f, 1e-5f, "target y");
    check_near(loaded.centre_azimuth(), 0.10f, 1e-5f, "centre az");
    check_near(loaded.params().amp_y, 0.08f, 1e-5f, "amp y");
    check_near(loaded.params().duration_s, 1.5f, 1e-5f, "duration");
    check(loaded[0].distance_cm == d[0].distance_cm, "distance_cm roundtrip");
    check_near(loaded[0].mechanical_x, d[0].mechanical_x, 1e-5f, "angle x roundtrip");
    std::remove(path);
}

}  // namespace

int main() {
    test_sort_left_to_right();
    test_bracket_picks_lowest_elevation();
    test_surface_in_front_is_not_visible();
    test_surface_behind_is_not_visible();
    test_gap_at_crossing_is_not_visible();
    test_range_jump_at_crossing_is_not_visible();
    test_azimuth_gate_drops_off_strip_samples();
    test_dwell_z_stub_and_binary_roundtrip();
    if (g_fails) {
        std::fprintf(stderr, "%d azimuth finder test(s) failed\n", g_fails);
        return 1;
    }
    std::printf("azimuth finder tests passed\n");
    return 0;
}
