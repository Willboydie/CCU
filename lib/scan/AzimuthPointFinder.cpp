#include "scan/AzimuthPointFinder.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

float median_of(std::vector<float> v) {
    if (v.empty())
        return 0.0f;
    const std::size_t n = v.size();
    std::sort(v.begin(), v.end());
    if (n % 2 == 1)
        return v[n / 2];
    return 0.5f * (v[n / 2 - 1] + v[n / 2]);
}

}  // namespace

AzimuthPointFinder::AzimuthPointFinder(Config cfg) : cfg_(cfg) {}

void AzimuthPointFinder::sort_left_to_right(std::vector<TargetXY>& pts) {
    std::sort(pts.begin(), pts.end(), [](const TargetXY& a, const TargetXY& b) {
        return azimuth_of_xy(a.x, a.y) > azimuth_of_xy(b.x, b.y);
    });
}

PixelHit AzimuthPointFinder::find(float target_x, float target_y,
                                  const PointCloud& strip) const {
    return analyse(target_x, target_y, strip).hit;
}

StripAnalysis AzimuthPointFinder::analyse(float target_x, float target_y,
                                          const PointCloud& strip) const {
    StripAnalysis out;
    out.target_azimuth = azimuth_of_xy(target_x, target_y);
    out.target_horiz = horizontal_range_xy(target_x, target_y);
    out.hit.azimuth = out.target_azimuth;

    const float bin_w = (cfg_.pixel_bin_rad > 1e-6f) ? cfg_.pixel_bin_rad : kPixelBinRad;
    const float emin = cfg_.elev_min;
    const float emax = cfg_.elev_max;
    if (!(emax > emin)) {
        out.reason = "invalid elevation FOV";
        return out;
    }

    const int n = std::max(1, static_cast<int>(std::ceil((emax - emin) / bin_w)));
    std::vector<std::vector<float>> ranges(static_cast<std::size_t>(n));

    for (const auto& p : strip.points()) {
        if (!p.valid)
            continue;
        if (std::fabs(p.azimuth - out.target_azimuth) > cfg_.azimuth_gate_rad)
            continue;
        const int i = static_cast<int>(std::floor((p.elevation - emin) / bin_w));
        if (i < 0 || i >= n)
            continue;
        ranges[static_cast<std::size_t>(i)].push_back(p.range);
        ++out.gated_samples;
    }

    out.bins.resize(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        ElevationBin& b = out.bins[static_cast<std::size_t>(i)];
        b.index = i;
        b.elevation = emin + (static_cast<float>(i) + 0.5f) * bin_w;
        b.r_target = r_target_at_elevation(out.target_horiz, b.elevation);
        b.sample_count = static_cast<int>(ranges[static_cast<std::size_t>(i)].size());
        b.occupied = b.sample_count > 0;
        if (!b.occupied)
            continue;
        b.range = median_of(ranges[static_cast<std::size_t>(i)]);
        b.horiz_range = b.range * std::cos(b.elevation);
        b.residual = b.range - b.r_target;
    }

    std::vector<int> occ;
    occ.reserve(static_cast<std::size_t>(n));
    for (int i = 0; i < n; ++i) {
        if (out.bins[static_cast<std::size_t>(i)].occupied)
            occ.push_back(i);
    }

    auto close = [&](float e) { return std::fabs(e) <= cfg_.range_match_tol_m; };

    int best_single = -1;
    float best_single_e = 1e30f;
    for (int i : occ) {
        const float ae = std::fabs(out.bins[static_cast<std::size_t>(i)].residual);
        if (ae <= cfg_.range_match_tol_m && ae < best_single_e) {
            best_single_e = ae;
            best_single = i;
        }
    }

    int best_i = -1;
    int best_j = -1;
    float best_pair = 1e30f;
    bool bad_crossing = false;

    for (std::size_t t = 1; t < occ.size(); ++t) {
        const int i = occ[t - 1];
        const int j = occ[t];
        const ElevationBin& a = out.bins[static_cast<std::size_t>(i)];
        const ElevationBin& b = out.bins[static_cast<std::size_t>(j)];
        const bool gap = j > i + 1;
        const bool jump = std::fabs(a.range - b.range) > cfg_.range_jump_m;
        const bool straddle =
            (a.residual <= 0.0f && b.residual >= 0.0f) ||
            (a.residual >= 0.0f && b.residual <= 0.0f);
        if (!straddle)
            continue;
        if (gap || jump || !close(a.residual) || !close(b.residual)) {
            bad_crossing = true;
            continue;
        }
        const float score = std::min(std::fabs(a.residual), std::fabs(b.residual));
        if (score < best_pair) {
            best_pair = score;
            best_i = i;
            best_j = j;
        }
    }

    auto fill_hit = [&](int idx) {
        const ElevationBin& b = out.bins[static_cast<std::size_t>(idx)];
        out.hit.visibility = PointVisibility::Found;
        out.hit.bin_index = idx;
        out.hit.elevation = b.elevation;
        out.hit.range = b.range;
        out.hit.r_target = b.r_target;
        boresight_to_mechanical(out.target_azimuth, b.elevation,
                                out.hit.mechanical_x, out.hit.mechanical_y);
    };

    if (best_i >= 0) {
        const float ei = std::fabs(out.bins[static_cast<std::size_t>(best_i)].residual);
        const float ej = std::fabs(out.bins[static_cast<std::size_t>(best_j)].residual);
        fill_hit(ei <= ej ? best_i : best_j);
        out.reason = "bracket";
        return out;
    }
    if (best_single >= 0) {
        fill_hit(best_single);
        out.reason = "near bin";
        return out;
    }
    if (bad_crossing)
        out.reason = "discontinuity where r should be";
    else if (occ.empty())
        out.reason = "no valid samples on strip";
    else
        out.reason = "continuous range does not include r_target";
    return out;
}
