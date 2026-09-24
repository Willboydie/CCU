// H2 bench. Opens the orientation port, logs each 0xA0 frame, then scores
// rate, zero roll/pitch/IMU yaw, and a filtered flick of the stage.
//
// Hold the base still for the first window. When asked, flick it once and hold.

#include "angle.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

namespace {

volatile std::sig_atomic_t quit = 0;

void on_signal(int) { quit = 1; }

std::string trim(std::string s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    std::size_t i = 0;
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    return s.substr(i);
}

bool load_angle_port(const char* path, std::string& out) {
    std::ifstream in(path);
    if (!in) {
        std::fprintf(stderr, "cannot read %s\n", path);
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const std::size_t eq = line.find('=');
        if (eq == std::string::npos) continue;
        if (trim(line.substr(0, eq)) == "angle") out = trim(line.substr(eq + 1));
    }
    if (out.empty()) {
        std::fprintf(stderr, "%s needs angle=\n", path);
        return false;
    }
    return true;
}

float wrap_pi(float d) {
    constexpr float kPi = 3.14159265f;
    constexpr float kTwoPi = 6.28318530f;
    while (d > kPi) d -= kTwoPi;
    while (d < -kPi) d += kTwoPi;
    return d;
}

bool collect_until(std::chrono::steady_clock::time_point deadline,
                   std::vector<ccu::AngleSample>& out) {
    while (!quit && std::chrono::steady_clock::now() < deadline) {
        ccu::angle_poll();
        ccu::AngleSample sample;
        while (ccu::angle_take(sample)) out.push_back(sample);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return !quit;
}

long long median_dt(const std::vector<ccu::AngleSample>& samples) {
    std::vector<long long> dts;
    dts.reserve(samples.size());
    for (const ccu::AngleSample& sample : samples) {
        if (sample.dt_ms > 0) dts.push_back(sample.dt_ms);
    }
    if (dts.empty()) return -1;
    std::sort(dts.begin(), dts.end());
    return dts[dts.size() / 2];
}

float max_abs_tilt(const std::vector<ccu::AngleSample>& samples, float& roll, float& pitch,
                   float& imu_yaw) {
    roll = 0.f;
    pitch = 0.f;
    imu_yaw = 0.f;
    for (const ccu::AngleSample& sample : samples) {
        roll = std::max(roll, std::fabs(sample.roll));
        pitch = std::max(pitch, std::fabs(sample.pitch));
        imu_yaw = std::max(imu_yaw, std::fabs(sample.imu_yaw));
    }
    return std::max(roll, std::max(pitch, imu_yaw));
}

struct FlickScore {
    float travel = 0.f;
    float max_step = 0.f;
    int span = 0;
};

FlickScore score_flick(const std::vector<ccu::AngleSample>& samples) {
    FlickScore best;
    const int n = static_cast<int>(samples.size());
    for (int start = 0; start + 1 < n; ++start) {
        float acc = 0.f;
        float local_max = 0.f;
        int quiet = 0;
        for (int end = start + 1; end < n; ++end) {
            const float step = wrap_pi(samples[end].stage - samples[end - 1].stage);
            acc += step;
            const float mag = std::fabs(step);
            if (mag > local_max) local_max = mag;
            if (std::fabs(acc) > best.travel) {
                best.travel = std::fabs(acc);
                best.max_step = local_max;
                best.span = end - start;
            }
            if (mag < 0.01f) {
                ++quiet;
                if (quiet >= 5 && std::fabs(acc) >= 0.4f) break;
            } else {
                quiet = 0;
            }
        }
    }
    return best;
}

const char* mark(bool ok) { return ok ? "PASS" : "FAIL"; }

}  // namespace

int main(int argc, char** argv) {
    const char* cfg = argc > 1 ? argv[1] : "ccu_ports.cfg";
    std::string port;
    if (std::strncmp(cfg, "/dev/", 5) == 0) {
        port = cfg;
    } else if (!load_angle_port(cfg, port)) {
        return 1;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    if (!ccu::angle_open(port.c_str())) return 1;

    std::printf("H2 angle bench\n");
    std::printf("Hold the base still.\n");
    std::fflush(stdout);

    const auto boot_end = std::chrono::steady_clock::now() + std::chrono::seconds(8);
    std::vector<ccu::AngleSample> still;
    while (!quit && std::chrono::steady_clock::now() < boot_end && still.empty()) {
        ccu::angle_poll();
        ccu::AngleSample sample;
        if (ccu::angle_take(sample)) still.push_back(sample);
        else std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    if (quit || still.empty()) {
        std::printf("H2 FAIL: %s\n", quit ? "interrupted" : "no orientation frames");
        std::fflush(stdout);
        ccu::angle_close();
        return 1;
    }

    const auto still_end = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    if (!collect_until(still_end, still)) {
        std::printf("H2 FAIL: interrupted\n");
        std::fflush(stdout);
        ccu::angle_close();
        return 1;
    }

    const long long dt_ms = median_dt(still);
    const bool rate_ok = still.size() >= 15 && dt_ms >= 70 && dt_ms <= 150;
    const double hz = dt_ms > 0 ? 1000.0 / static_cast<double>(dt_ms) : 0.0;
    std::printf("H2 rate samples=%zu median_dt_ms=%lld hz=%.1f %s\n", still.size(), dt_ms, hz,
                mark(rate_ok));

    float roll = 0.f;
    float pitch = 0.f;
    float imu_yaw = 0.f;
    const float tilt = max_abs_tilt(still, roll, pitch, imu_yaw);
    const bool zeros_so_far = tilt < 1.0e-4f;
    std::printf("H2 zeros roll=%.4f pitch=%.4f imu_yaw=%.4f %s\n", static_cast<double>(roll),
                static_cast<double>(pitch), static_cast<double>(imu_yaw), mark(zeros_so_far));
    std::printf("Flick the base once, sharply, and hold the new angle.\n");
    std::fflush(stdout);

    std::vector<ccu::AngleSample> flick;
    const auto flick_end = std::chrono::steady_clock::now() + std::chrono::seconds(45);
    if (!collect_until(flick_end, flick)) {
        std::printf("H2 FAIL: interrupted\n");
        std::fflush(stdout);
        ccu::angle_close();
        return 1;
    }

    float flick_roll = 0.f;
    float flick_pitch = 0.f;
    float flick_yaw = 0.f;
    const float flick_tilt = max_abs_tilt(flick, flick_roll, flick_pitch, flick_yaw);
    const bool zeros_ok = zeros_so_far && flick_tilt < 1.0e-4f;
    if (!zeros_ok) {
        std::printf("H2 zeros roll=%.4f pitch=%.4f imu_yaw=%.4f FAIL\n",
                    static_cast<double>(std::max(roll, flick_roll)),
                    static_cast<double>(std::max(pitch, flick_pitch)),
                    static_cast<double>(std::max(imu_yaw, flick_yaw)));
    }

    const FlickScore scored = score_flick(flick);
    const bool filter_ok =
        scored.travel >= 0.4f && scored.span >= 8 && scored.max_step < 0.5f * scored.travel;
    std::printf("H2 filter travel_rad=%.3f max_step_rad=%.3f samples=%d %s\n",
                static_cast<double>(scored.travel), static_cast<double>(scored.max_step),
                scored.span, mark(filter_ok));
    if (!filter_ok && scored.travel < 0.4f) {
        std::printf("H2 filter: no flick seen\n");
    }

    const bool pass = rate_ok && zeros_ok && filter_ok;
    std::printf("H2 %s\n", pass ? "PASS" : "FAIL");
    std::fflush(stdout);
    ccu::angle_close();
    return pass ? 0 : 1;
}
