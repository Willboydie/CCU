#pragma once

#include <string>
#include <vector>

// Mechanical angle limits (rad), matching the transceiver FOV.
inline constexpr float kScanXMin = 2.5f;
inline constexpr float kScanXMax = 4.2f;
inline constexpr float kScanYMin = 2.6f;
inline constexpr float kScanYMax = 3.9f;
inline constexpr float kScanNeutralX = 3.54f;
inline constexpr float kScanNeutralY = 3.39f;
inline constexpr float kScanAmpMax = 0.6f;

// Infinity (figure-8) Lissajous: fy = 2 * fx. The curve is sampled once per
// unit time step (1 / kLissajousHz seconds). DELTA is how many of those
// samples elapse between raster-centre updates.
inline constexpr float kInfinityFreqX = 2.0f;
inline constexpr float kInfinityFreqY = 4.0f;
inline constexpr float kLissajousHz = 180.0f;

struct ScanPathParams {
    float inc = 0.08f;   // centre step along a raster row (rad)
    float delta = 20.0f; // >1 — centre updates every DELTA Lissajous samples
    float alpha = 0.0f;  // rad added to the vertical gap after each row
    float gap0 = 0.08f;  // initial vertical pitch between raster rows (rad)
    float amp_x = 0.12f;
    float amp_y = 0.12f;
};

struct ScanPathPoint {
    float x = 0;
    float y = 0;
    float t = 0;
};

struct ScanPathResult {
    std::vector<ScanPathPoint> beam;
    std::vector<ScanPathPoint> centre;
    std::vector<float> row_gaps;  // vertical pitch used after each completed row
    int rows = 0;
    int dwells = 0;
    float duration = 0;
    float cx_min = 0;
    float cx_max = 0;
    float cy_min = 0;
    float cy_max = 0;
    bool truncated = false;
    std::string warning;
};

// Clamps amplitudes into range and snaps DELTA to an integer >= 2. Returns
// false and leaves `err` if INC/DELTA/ALPHA/GAP0 are invalid or the
// oscillation would not fit in the FOV.
bool validate_scan_path_params(ScanPathParams& p, std::string& err);

// Number of Lissajous samples between centre updates (DELTA, integer >= 2).
int lissajous_per_centre(const ScanPathParams& p);

// Coarse infinity scan whose centre walks a bidirectional X raster. The
// Lissajous is advanced every time step; the centre moves every DELTA of
// those steps. Vertical pitch starts at GAP0 and ALPHA is added after each
// horizontal pass. The path always continues until the beam envelope reaches
// the top of the Y angular bounds (unless a sample cap is hit).
ScanPathResult generate_raster_infinity(const ScanPathParams& p);
