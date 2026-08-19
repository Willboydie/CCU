#include "scan/RasterInfinityPath.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

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

void test_alpha_finite() {
    ScanPathParams p;
    std::string err;
    p.alpha = 0.0f;
    check(validate_scan_path_params(p, err), "ALPHA=0 accepted");
    p.alpha = 2.1f;
    check(validate_scan_path_params(p, err), "ALPHA>2 accepted as additive step");
    p.alpha = -0.01f;
    check(validate_scan_path_params(p, err), "negative ALPHA accepted");
}

void test_gap_grows_by_alpha() {
    ScanPathParams p;
    p.inc = 0.05f;
    p.delta = 4.0f;
    p.alpha = 0.05f;
    p.gap0 = 0.10f;
    p.amp_x = 0.05f;
    p.amp_y = 0.05f;
    std::string err;
    check(validate_scan_path_params(p, err), "params valid");

    const ScanPathResult r = generate_raster_infinity(p);
    check(r.rows >= 3, "enough rows to see ALPHA steps");
    check(r.row_gaps.size() >= 3, "recorded gaps");
    if (r.row_gaps.size() >= 3) {
        check_near(r.row_gaps[0], 0.10f, 1e-5f, "first gap is GAP0");
        check_near(r.row_gaps[1], 0.15f, 1e-5f, "second gap is GAP0+ALPHA");
        check_near(r.row_gaps[2], 0.20f, 1e-5f, "third gap is GAP0+2*ALPHA");
    }
    check(!r.centre.empty(), "has centre samples");
    check(!r.beam.empty(), "has beam samples");
}

void test_alpha_zero_is_uniform() {
    ScanPathParams p;
    p.inc = 0.04f;
    p.delta = 4.0f;
    p.alpha = 0.0f;
    p.gap0 = 0.12f;
    p.amp_x = 0.05f;
    p.amp_y = 0.05f;
    const ScanPathResult r = generate_raster_infinity(p);
    check(r.row_gaps.size() >= 2, "uniform raster has gaps");
    const int n = static_cast<int>(r.row_gaps.size());
    for (int i = 0; i < n - 1; ++i)
        check_near(r.row_gaps[static_cast<std::size_t>(i)], p.gap0, 1e-5f,
                   "ALPHA=0 keeps pitch = GAP0");
}

void test_reaches_top_bound() {
    ScanPathParams p;
    p.inc = 0.08f;
    p.delta = 4.0f;
    p.alpha = 0.02f;
    p.gap0 = 0.10f;
    p.amp_x = 0.05f;
    p.amp_y = 0.08f;
    std::string err;
    check(validate_scan_path_params(p, err), "top-bound params valid");
    const ScanPathResult r = generate_raster_infinity(p);
    check(!r.truncated, "reaches top without hitting sample cap");
    check(!r.centre.empty(), "has centres");
    if (!r.centre.empty())
        check_near(r.centre.back().y, r.cy_max, 1e-3f, "last centre is at top of centre FOV");
    float ymax = -1e9f;
    for (const ScanPathPoint& s : r.beam)
        ymax = std::max(ymax, s.y);
    check(ymax <= kScanYMax + 1e-4f, "beam stays at or below Y max");
    check(ymax >= r.cy_max - 1e-3f, "beam envelope reaches the top centre line");
}

void test_delta_rejects() {
    ScanPathParams p;
    std::string err;
    p.delta = 1.0f;
    check(!validate_scan_path_params(p, err), "DELTA=1 rejected");
    p.delta = 0.5f;
    check(!validate_scan_path_params(p, err), "DELTA<1 rejected");
    p.delta = 1.6f;
    check(validate_scan_path_params(p, err), "DELTA=1.6 accepted");
    check_near(p.delta, 2.0f, 1e-6f, "DELTA snaps to integer >= 2");
}

void test_gap0_rejects() {
    ScanPathParams p;
    std::string err;
    p.gap0 = 0.0f;
    check(!validate_scan_path_params(p, err), "GAP0=0 rejected");
    p.gap0 = -0.1f;
    check(!validate_scan_path_params(p, err), "GAP0<0 rejected");
    p.gap0 = 0.05f;
    check(validate_scan_path_params(p, err), "GAP0>0 accepted");
}

void test_delta_is_lissajous_stride() {
    ScanPathParams p;
    p.inc = 0.20f;
    p.delta = 5.0f;
    p.alpha = 0.0f;
    p.amp_x = 0.05f;
    p.amp_y = 0.05f;
    std::string err;
    check(validate_scan_path_params(p, err), "delta stride params valid");
    const int period = lissajous_per_centre(p);
    check(period == 5, "DELTA=5 -> 5 Lissajous samples per centre");

    const ScanPathResult r = generate_raster_infinity(p);
    check(r.dwells >= 2, "at least two centre updates");
    check(static_cast<int>(r.beam.size()) == r.dwells * period,
          "beam samples = centre updates * DELTA");
    if (r.centre.size() >= 2) {
        const float dt = 1.0f / kLissajousHz;
        check_near(r.centre[1].t - r.centre[0].t, static_cast<float>(period) * dt, 1e-5f,
                   "centre dt is DELTA Lissajous periods");
    }
}

}  // namespace

int main() {
    test_alpha_finite();
    test_delta_rejects();
    test_gap0_rejects();
    test_gap_grows_by_alpha();
    test_alpha_zero_is_uniform();
    test_reaches_top_bound();
    test_delta_is_lissajous_stride();
    if (g_fails) {
        std::fprintf(stderr, "%d scan path test(s) failed\n", g_fails);
        return 1;
    }
    std::printf("scan path tests passed\n");
    return 0;
}
