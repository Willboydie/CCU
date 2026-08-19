#include "scan/RasterInfinityPath.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr int kMaxDwells = 20000;
constexpr int kMaxBeamPts = 120000;
constexpr float kEps = 1e-6f;

int centre_period(float delta) {
    return std::max(2, static_cast<int>(std::lround(delta)));
}

}  // namespace

int lissajous_per_centre(const ScanPathParams& p) {
    return centre_period(p.delta);
}

bool validate_scan_path_params(ScanPathParams& p, std::string& err) {
    err.clear();
    p.amp_x = std::clamp(p.amp_x, 0.0f, kScanAmpMax);
    p.amp_y = std::clamp(p.amp_y, 0.0f, kScanAmpMax);

    if (!(p.inc > 0.0f) || !std::isfinite(p.inc)) {
        err = "INC must be a positive finite value";
        return false;
    }
    if (!(p.delta > 1.0f) || !std::isfinite(p.delta)) {
        err = "DELTA must be a multiplier > 1 (Lissajous samples per centre update)";
        return false;
    }
    p.delta = static_cast<float>(centre_period(p.delta));
    if (!std::isfinite(p.alpha)) {
        err = "ALPHA must be finite (rad added to the vertical gap each row)";
        return false;
    }
    if (!(p.gap0 > 0.0f) || !std::isfinite(p.gap0)) {
        err = "GAP0 must be a positive finite value";
        return false;
    }
    if (!std::isfinite(p.amp_x) || !std::isfinite(p.amp_y)) {
        err = "AMPX/AMPY must be finite";
        return false;
    }

    const float cx_min = kScanXMin + p.amp_x;
    const float cx_max = kScanXMax - p.amp_x;
    const float cy_min = kScanYMin + p.amp_y;
    const float cy_max = kScanYMax - p.amp_y;
    if (cx_min > cx_max || cy_min > cy_max) {
        err = "AMPX/AMPY leave no room for a raster centre inside the FOV";
        return false;
    }
    return true;
}

ScanPathResult generate_raster_infinity(const ScanPathParams& p) {
    ScanPathResult out;
    out.cx_min = kScanXMin + p.amp_x;
    out.cx_max = kScanXMax - p.amp_x;
    out.cy_min = kScanYMin + p.amp_y;
    out.cy_max = kScanYMax - p.amp_y;

    const int samples_per_dwell = lissajous_per_centre(p);
    const float dt = 1.0f / kLissajousHz;

    float cx = out.cx_min;
    float cy = out.cy_min;
    float dir = 1.0f;
    float gap = p.gap0;
    float t = 0.0f;
    int row = 1;

    out.beam.reserve(static_cast<std::size_t>(
        std::min(kMaxBeamPts, kMaxDwells * samples_per_dwell)));
    out.centre.reserve(1024);

    auto dwell = [&]() -> bool {
        if (static_cast<int>(out.centre.size()) >= kMaxDwells) {
            out.warning = "stopped at dwell cap";
            out.truncated = true;
            return false;
        }
        if (static_cast<int>(out.beam.size()) + samples_per_dwell > kMaxBeamPts) {
            out.warning = "stopped at sample cap — increase INC or reduce DELTA";
            out.truncated = true;
            return false;
        }
        out.centre.push_back({cx, cy, t});
        for (int i = 0; i < samples_per_dwell; ++i) {
            const float x = cx + p.amp_x * std::sin(2.0f * kPi * kInfinityFreqX * t);
            const float y = cy + p.amp_y * std::sin(2.0f * kPi * kInfinityFreqY * t);
            out.beam.push_back({x, y, t});
            t += dt;
        }
        return true;
    };

    if (!dwell()) {
        out.dwells = static_cast<int>(out.centre.size());
        out.duration = t;
        out.rows = row;
        return out;
    }

    while (true) {
        const float next_x = cx + dir * p.inc;
        const bool past_x = (dir > 0.0f) ? (next_x > out.cx_max + kEps)
                                         : (next_x < out.cx_min - kEps);
        if (!past_x) {
            cx = std::clamp(next_x, out.cx_min, out.cx_max);
            if (!dwell())
                break;
            continue;
        }

        // End of a horizontal pass. Climb toward the top of the FOV: by `gap`
        // when it fits, otherwise by whatever room is left so the beam envelope
        // always reaches kScanYMax.
        if (cy >= out.cy_max - kEps)
            break;

        const float room = out.cy_max - cy;
        float remaining = (gap > kEps) ? std::min(gap, room) : room;
        out.row_gaps.push_back(remaining);
        bool stopped = false;
        while (remaining > kEps) {
            const float step = std::min(p.inc, remaining);
            cy += step;
            remaining -= step;
            if (!dwell()) {
                stopped = true;
                break;
            }
        }
        if (stopped)
            break;

        gap += p.alpha;
        dir = -dir;
        ++row;
    }

    out.rows = row;
    out.dwells = static_cast<int>(out.centre.size());
    out.duration = t;
    return out;
}
