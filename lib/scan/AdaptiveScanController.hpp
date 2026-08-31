#pragma once

#include "AzimuthPointFinder.hpp"
#include "DwellPointCloud.hpp"
#include "Geometry.hpp"
#include "PointCloud.hpp"

#include "../tx/LidarController.hpp"

#include <chrono>
#include <vector>

// Orchestrates Point2Point strip sweeps, AzimuthPointFinder, and a dwell
// Lissajous. Gate 4 will add a control-stream capture buffer on
// LidarController (start/stop recording of 1 kHz ControlPayload) so the
// strip and dwell clouds are complete rather than polled.
class AdaptiveScanController {
public:
    struct Config {
        AzimuthPointFinder::Config finder;
        DwellParams dwell;
        std::chrono::milliseconds move_timeout{5000};
        std::chrono::milliseconds settle_timeout{500};
    };

    struct PointResult {
        float x = 0;
        float y = 0;
        float z = 0;
        bool visible = false;
        PixelHit hit;
        DwellPointCloud dwell;
    };

    AdaptiveScanController(LidarController& lidar, Config cfg = {});

    const Config& config() const { return cfg_; }
    void set_dwell_params(const DwellParams& p);

    // Sweep one azimuth through the elevation FOV:
    //   P2P to strip start, then P2P to strip end, keep every sample in between.
    PointCloud sweep_azimuth_strip(float azimuth);

    PointResult find_one(float x, float y);
    std::vector<PointResult> find_all(std::vector<TargetXY> targets);

private:
    bool wait_at_mechanical(float mech_x, float mech_y,
                            std::chrono::milliseconds timeout);
    PointCloud capture_while_moving(float mech_x, float mech_y);
    DwellPointCloud dwell_at(const PixelHit& hit);

    LidarController& lidar_;
    AzimuthPointFinder finder_;
    Config cfg_;
};
