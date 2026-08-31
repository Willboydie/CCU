#pragma once

#include "Geometry.hpp"
#include "PointCloud.hpp"

#include <vector>

struct TargetXY {
    float x = 0;
    float y = 0;
};

enum class PointVisibility {
    Found,
    NotVisible,
};

// Result of searching one azimuth strip for the elevation whose measured
// range matches r_target(phi) = |xy| / cos(phi).
struct PixelHit {
    PointVisibility visibility = PointVisibility::NotVisible;
    int bin_index = -1;
    float azimuth = 0;      // commanded strip azimuth (boresight)
    float elevation = 0;    // chosen pixel centre (boresight)
    float mechanical_x = 0;
    float mechanical_y = 0;
    float range = 0;        // measured r at the pixel, metres
    float r_target = 0;     // expected r at that elevation, metres
};

struct ElevationBin {
    int index = -1;
    float elevation = 0;    // bin centre, boresight rad
    float range = 0;        // median spherical r, metres
    float horiz_range = 0;  // r * cos(elevation) — implied |(x, y)|
    float r_target = 0;
    float residual = 0;     // range - r_target
    int sample_count = 0;
    bool occupied = false;
};

struct StripAnalysis {
    PixelHit hit;
    std::vector<ElevationBin> bins;
    float target_azimuth = 0;
    float target_horiz = 0;   // |(x, y)| of the query point
    int gated_samples = 0;
    const char* reason = "";
};

class AzimuthPointFinder {
public:
    struct Config {
        float pixel_bin_rad = kPixelBinRad;
        // |r_meas - r_target| on both sides of a crossing must be within this
        // for the pair to count as "sufficiently close" (metres). TBD.
        float range_match_tol_m = 0.30f;
        // Adjacent occupied bins whose range differs by more than this, at
        // the elevation where r_target should appear, count as a discontinuity.
        float range_jump_m = 1.0f;
        // Samples farther than this from the strip azimuth are ignored.
        float azimuth_gate_rad = kPixelBinRad;
        float elev_min = kMechYMin - kMechNeutralY;
        float elev_max = kMechYMax - kMechNeutralY;
    };

    explicit AzimuthPointFinder(Config cfg = {});

    const Config& config() const { return cfg_; }

    // Left to right: x forward, y left, so theta = atan2(y, x) is positive
    // to the left. Sort is descending azimuth.
    static void sort_left_to_right(std::vector<TargetXY>& pts);

    // Bin the strip by elevation, then look for two neighbouring occupied
    // bins whose residuals (r_meas - r_target) straddle zero and are each
    // within range_match_tol_m. Of those candidates, the lowest-elevation
    // pixel is the hit.
    //
    // NOT_VISIBLE when:
    //   - a range jump or empty-bin gap sits where the crossing should be, or
    //   - the occupied bins form a continuous run that never contains r_target.
    PixelHit find(float target_x, float target_y, const PointCloud& strip) const;
    StripAnalysis analyse(float target_x, float target_y, const PointCloud& strip) const;

private:
    Config cfg_;
};
