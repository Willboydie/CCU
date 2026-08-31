#include "ui/AzimuthFinderApplet.hpp"

#include "scan/AzimuthPointFinder.hpp"
#include "scan/DwellPointCloud.hpp"
#include "scan/Geometry.hpp"
#include "scan/PointCloud.hpp"
#include "utils/protocol.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {

void glfw_error(int, const char* desc) {
    std::fprintf(stderr, "GLFW: %s\n", desc ? desc : "");
}

bool wait_pose(LidarController& c, float mx, float my, float tol,
               std::chrono::milliseconds timeout) {
    const auto t0 = std::chrono::steady_clock::now();
    if (!c.wait_for_status(
            [&](const StatusPayload& s) {
                return std::fabs(s.p2p_target_x - mx) < 0.005f &&
                       std::fabs(s.p2p_target_y - my) < 0.005f;
            },
            timeout))
        return false;
    const auto used = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - t0);
    auto remain = timeout - used;
    if (remain.count() < 50)
        remain = std::chrono::milliseconds(50);
    return c.wait_for_control(
        [&](const ControlPayload& p) {
            return std::hypot(p.mechanical_angle_x - mx, p.mechanical_angle_y - my) < tol;
        },
        remain);
}

bool ensure_p2p(LidarController& c, std::string& err) {
    using ms = std::chrono::milliseconds;
    if (!c.wait_for_status([](const StatusPayload&) { return true; }, ms(2000))) {
        err = "No status from device";
        return false;
    }
    if (!c.set_enabled(true) || !c.wait_enabled(true, ms(3000))) {
        err = "Failed to enable motors";
        return false;
    }
    const uint8_t mode = c.status().scan_mode;
    if (mode == kScanModeCoarse) {
        c.stop_coarse_scanning();
        if (!c.wait_until_idle(ms(8000))) {
            err = "Timed out leaving coarse scan";
            return false;
        }
    }
    if (c.status().scan_mode != kScanModePoint2Point) {
        if (!c.scan_engine_on() || !c.wait_scan_mode(kScanModePoint2Point, ms(5000))) {
            err = "Failed to enter point-to-point";
            return false;
        }
    }
    return true;
}

float fit_amp(float centre, float amp, float lo, float hi) {
    amp = std::clamp(amp, 0.0f, kMechAmpMax);
    const float room = std::min(centre - lo, hi - centre);
    return std::max(0.0f, std::min(amp, room));
}

bool dwell_at_pixel(LidarController& c, const PixelHit& hit, DwellParams p,
                    float pose_tol, DwellPointCloud& cloud, std::string& err) {
    using ms = std::chrono::milliseconds;
    p.amp_x = fit_amp(hit.mechanical_x, p.amp_x, kMechXMin, kMechXMax);
    p.amp_y = fit_amp(hit.mechanical_y, p.amp_y, kMechYMin, kMechYMax);
    if (p.duration_s < 0.05f)
        p.duration_s = 0.05f;

    if (!c.set_point_to_point_target(hit.mechanical_x, hit.mechanical_y) ||
        !wait_pose(c, hit.mechanical_x, hit.mechanical_y, pose_tol, ms(8000))) {
        err = "Timed out moving to pixel centre";
        return false;
    }

    if (!(c.set_x_amplitude(p.amp_x) && c.set_y_amplitude(p.amp_y) &&
          c.set_x_frequency(p.freq_x) && c.set_y_frequency(p.freq_y) &&
          c.set_x_phase(p.phase_x) && c.set_y_phase(p.phase_y))) {
        err = "Failed to set coarse scan params";
        return false;
    }

    if (!c.begin_coarse_scanning()) {
        err = "BEGIN_COARSE_SCANNING rejected (amplitude vs FOV?)";
        return false;
    }
    if (!c.wait_scan_mode(kScanModeCoarse, ms(3000))) {
        err = "Did not enter coarse scan";
        return false;
    }
    if (!c.wait_until_idle(ms(8000))) {
        err = "Timed out waiting for coarse wind-up";
        return false;
    }

    c.start_control_capture();
    std::this_thread::sleep_for(
        std::chrono::duration_cast<ms>(std::chrono::duration<float>(p.duration_s)));
    auto samples = c.stop_control_capture();

    c.stop_coarse_scanning();
    c.wait_until_idle(ms(8000));

    cloud.set_centre(hit.azimuth, hit.elevation);
    cloud.set_centre_mechanical(hit.mechanical_x, hit.mechanical_y);
    cloud.set_params(p);
    cloud.append_from_control(samples);
    return true;
}

std::filesystem::path executable_dir() {
#ifdef _WIN32
    char buf[MAX_PATH]{};
    GetModuleFileNameA(nullptr, buf, MAX_PATH);
    return std::filesystem::path(buf).parent_path();
#else
    return std::filesystem::current_path();
#endif
}

std::string dwell_output_path(float x, float y) {
    const auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    char name[160];
    std::snprintf(name, sizeof(name),
                  "dwell_x%.2f_y%.2f_%04d%02d%02d_%02d%02d%02d.bin",
                  x, y, tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    const auto dir = executable_dir() / "dwells";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return (dir / name).string();
}

}  // namespace

struct AzimuthFinderApplet::Impl {
    Host& host;
    std::vector<std::string> ports;
    int port_idx = 0;
    bool tried_initial = false;
    std::string last_error;
    std::string status = "Connect, set XY, then Sweep strip.";

    float target_x = 3.0f;
    float target_y = 0.0f;
    float pose_tol = 0.04f;
    float p2p_vmax = 1.0f;  // rad/s, SET_VELOCITY_MAX
    AzimuthPointFinder::Config finder_cfg;
    DwellParams dwell;

    std::atomic<bool> busy{false};
    std::thread worker;

    std::mutex mu;
    bool have_result = false;
    PointCloud strip;
    StripAnalysis analysis;
    DwellPointCloud dwell_cloud;
    std::string dwell_path;

    explicit Impl(Host& h) : host(h) { refresh_ports(); }

    ~Impl() { join_worker(); }

    void join_worker() {
        if (worker.joinable())
            worker.join();
    }

    void refresh_ports() {
        ports = host.list_ports ? host.list_ports() : std::vector<std::string>{};
        port_idx = 0;
        if (!host.initial_port.empty()) {
            for (int i = 0; i < static_cast<int>(ports.size()); ++i) {
                if (ports[static_cast<std::size_t>(i)] == host.initial_port) {
                    port_idx = i;
                    break;
                }
            }
        }
    }

    bool connected() const { return host.connected && host.connected(); }

    std::shared_ptr<LidarController> controller() const {
        return host.controller ? host.controller() : nullptr;
    }

    bool send_cmd(const std::function<bool(LidarController&)>& fn, const char* fail) {
        auto c = controller();
        if (!c) {
            last_error = "Not connected";
            return false;
        }
        if (!fn(*c)) {
            last_error = fail;
            return false;
        }
        last_error.clear();
        return true;
    }

    float clamped_vmax() const {
        return std::clamp(p2p_vmax, 0.01f, 15.0f);
    }

    void try_initial_connect() {
        if (tried_initial) return;
        tried_initial = true;
        if (host.initial_port.empty() || connected()) return;
        std::string err;
        if (host.connect && host.connect(host.initial_port, err))
            status = "Connected " + host.initial_port;
        else
            last_error = err.empty() ? "Initial connect failed" : err;
    }

    void start_sweep() {
        if (busy.load() || !connected())
            return;
        join_worker();
        busy.store(true);
        status = "Sweeping azimuth strip...";
        last_error.clear();
        const float tx = target_x;
        const float ty = target_y;
        const float tol = pose_tol;
        const float vmax = clamped_vmax();
        p2p_vmax = vmax;
        const AzimuthPointFinder::Config cfg = finder_cfg;
        const DwellParams dwell_p = dwell;
        worker = std::thread([this, tx, ty, tol, vmax, cfg, dwell_p] {
            run_sweep(tx, ty, tol, vmax, cfg, dwell_p);
            busy.store(false);
        });
    }

    void run_sweep(float tx, float ty, float tol, float vmax,
                   AzimuthPointFinder::Config cfg, DwellParams dwell_p) {
        using ms = std::chrono::milliseconds;
        auto fail = [&](const std::string& msg) {
            std::lock_guard<std::mutex> lock(mu);
            last_error = msg;
            status = msg;
            have_result = false;
        };

        auto ctl = host.controller ? host.controller() : nullptr;
        if (!ctl) {
            fail("Not connected");
            return;
        }

        std::string err;
        if (!ensure_p2p(*ctl, err)) {
            fail(err);
            return;
        }
        if (!ctl->set_velocity_max(vmax)) {
            fail("Failed to set P2P max speed");
            return;
        }

        const float az = azimuth_of_xy(tx, ty);
        float mech_x = 0, mech_y = 0;
        boresight_to_mechanical(az, 0.0f, mech_x, mech_y);
        if (mech_x < kMechXMin || mech_x > kMechXMax) {
            fail("Target azimuth is outside the mechanical X FOV");
            return;
        }

        const float y0 = kMechYMin;
        const float y1 = kMechYMax;
        if (!ctl->set_point_to_point_target(mech_x, y0) ||
            !wait_pose(*ctl, mech_x, y0, tol, ms(8000))) {
            fail("Timed out moving to strip start");
            return;
        }
        std::this_thread::sleep_for(ms(80));

        ctl->start_control_capture();
        if (!ctl->set_point_to_point_target(mech_x, y1) ||
            !wait_pose(*ctl, mech_x, y1, tol, ms(8000))) {
            ctl->stop_control_capture();
            fail("Timed out sweeping strip to end");
            return;
        }
        std::this_thread::sleep_for(ms(40));
        auto captured = ctl->stop_control_capture();
        const bool overflow = ctl->capture_overflow();

        PointCloud cloud;
        cloud.append_from_control(captured);

        AzimuthPointFinder finder(cfg);
        StripAnalysis a = finder.analyse(tx, ty, cloud);

        DwellPointCloud dwell_out;
        std::string saved_path;
        std::string dwell_err;
        bool dwelled = false;

        if (a.hit.visibility == PointVisibility::Found) {
            {
                std::lock_guard<std::mutex> lock(mu);
                strip = cloud;
                analysis = a;
                have_result = true;
                status = "FOUND — moving to pixel and dwelling...";
                last_error.clear();
            }
            if (!dwell_at_pixel(*ctl, a.hit, dwell_p, tol, dwell_out, dwell_err)) {
                std::lock_guard<std::mutex> lock(mu);
                last_error = dwell_err;
                status = dwell_err;
                return;
            }
            dwelled = true;
            dwell_out.set_target_xy(tx, ty);
            saved_path = dwell_output_path(tx, ty);
            std::string save_err;
            if (!dwell_out.save_binary(saved_path, save_err)) {
                dwell_err = save_err;
                saved_path.clear();
            }
        }

        char buf[384];
        if (a.hit.visibility == PointVisibility::Found) {
            if (dwelled && !saved_path.empty()) {
                std::snprintf(buf, sizeof(buf),
                              "FOUND  bin %d  el %.3f rad  | dwell %d samples  saved %s",
                              a.hit.bin_index, a.hit.elevation,
                              static_cast<int>(dwell_out.size()), saved_path.c_str());
            } else if (dwelled) {
                std::snprintf(buf, sizeof(buf),
                              "FOUND  bin %d  | dwell %d samples  save failed",
                              a.hit.bin_index, static_cast<int>(dwell_out.size()));
            } else {
                std::snprintf(buf, sizeof(buf),
                              "FOUND  bin %d  el %.3f rad  r %.2f m  (%s, %d samples)",
                              a.hit.bin_index, a.hit.elevation, a.hit.range,
                              a.reason, a.gated_samples);
            }
        } else {
            std::snprintf(buf, sizeof(buf),
                          "NOT_VISIBLE  %s  (%d gated samples%s)",
                          a.reason, a.gated_samples, overflow ? ", capture truncated" : "");
        }

        std::lock_guard<std::mutex> lock(mu);
        strip = std::move(cloud);
        analysis = std::move(a);
        dwell_cloud = std::move(dwell_out);
        dwell_path = std::move(saved_path);
        have_result = true;
        status = buf;
        if (!dwell_err.empty())
            last_error = dwell_err;
        else if (overflow)
            last_error = "Capture hit 50k sample cap";
        else
            last_error.clear();
    }

    void draw_connection() {
        ImGui::BeginChild("connection", ImVec2(0, 56), ImGuiChildFlags_Border);
        if (ImGui::BeginCombo("Port",
                              ports.empty() ? "(none)" : ports[static_cast<std::size_t>(port_idx)].c_str())) {
            for (int i = 0; i < static_cast<int>(ports.size()); ++i) {
                const bool sel = (i == port_idx);
                if (ImGui::Selectable(ports[static_cast<std::size_t>(i)].c_str(), sel))
                    port_idx = i;
                if (sel) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        ImGui::SameLine();
        if (ImGui::Button("Refresh")) refresh_ports();
        ImGui::SameLine();
        const bool sweeping = busy.load();
        if (connected()) {
            if (ImGui::Button("Disconnect") && host.disconnect && !sweeping) {
                host.disconnect();
                status = "Disconnected";
            }
        } else if (ImGui::Button("Connect")) {
            if (ports.empty()) {
                last_error = "No serial ports";
            } else {
                std::string err;
                const std::string& port = ports[static_cast<std::size_t>(port_idx)];
                if (host.connect && host.connect(port, err)) {
                    last_error.clear();
                    status = "Connected " + port;
                } else {
                    last_error = err.empty() ? "Connect failed" : err;
                }
            }
        }
        ImGui::Text("Link: %s", connected()
            ? (host.port_name ? host.port_name().c_str() : "up")
            : "down");
        ImGui::EndChild();
    }

    void draw_controls() {
        ImGui::BeginChild("controls", ImVec2(380, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Azimuth strip finder");
        ImGui::Separator();
        ImGui::TextWrapped(
            "Sweeps elevation at the target azimuth in Point2Point, then plots "
            "implied |(x,y)| = r cos(phi) against elevation. On a find, dwells a "
            "Lissajous about that pixel and writes the raw angle-distance cloud.");

        ImGui::Spacing();
        ImGui::TextDisabled("System");
        bool have_status = false;
        bool motors_on = false;
        if (auto c = controller()) {
            have_status = c->has_status();
            if (have_status)
                motors_on = c->status().enabled != 0;
        }
        const bool on = connected() && !busy.load();
        if (!on) ImGui::BeginDisabled();
        if (ImGui::Button("Enable", ImVec2(90, 0)))
            send_cmd([](LidarController& c) { return c.set_enabled(true); }, "Enable failed");
        ImGui::SameLine();
        if (ImGui::Button("Disable", ImVec2(90, 0)))
            send_cmd([](LidarController& c) { return c.set_enabled(false); }, "Disable failed");
        if (!on) ImGui::EndDisabled();
        ImGui::SameLine();
        if (!have_status)
            ImGui::TextDisabled("enabled  ?");
        else if (motors_on)
            ImGui::TextColored(ImVec4(0.35f, 0.9f, 0.45f, 1), "enabled");
        else
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1), "disabled");

        ImGui::Spacing();
        ImGui::TextDisabled("Target XY (m)");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("X", &target_x, 0.1f, 0.5f, "%.3f");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("Y", &target_y, 0.1f, 0.5f, "%.3f");
        ImGui::TextDisabled("azimuth %.3f rad   |(x,y)| %.3f m",
                            azimuth_of_xy(target_x, target_y),
                            horizontal_range_xy(target_x, target_y));

        ImGui::Spacing();
        ImGui::TextDisabled("Finder");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("match tol (m)", &finder_cfg.range_match_tol_m, 0.05f, 0.1f, "%.2f");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("jump (m)", &finder_cfg.range_jump_m, 0.1f, 0.5f, "%.2f");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("pose tol (rad)", &pose_tol, 0.005f, 0.02f, "%.3f");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("P2P vmax (rad/s)", &p2p_vmax, 0.1f, 0.5f, "%.2f");
        ImGui::SetItemTooltip("SET_VELOCITY_MAX. Clamped to 0.01..15 rad/s.");
        ImGui::SameLine();
        if (!on) ImGui::BeginDisabled();
        if (ImGui::Button("Set vmax")) {
            p2p_vmax = clamped_vmax();
            if (send_cmd([&](LidarController& c) { return c.set_velocity_max(p2p_vmax); },
                         "SET_VELOCITY_MAX failed"))
                status = "P2P vmax set";
        }
        if (!on) ImGui::EndDisabled();

        ImGui::Spacing();
        ImGui::TextDisabled("Dwell  (Lissajous about the pixel)");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("AMPX (rad)", &dwell.amp_x, 0.01f, 0.05f, "%.3f");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("AMPY (rad)", &dwell.amp_y, 0.01f, 0.05f, "%.3f");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("freq X (Hz)", &dwell.freq_x, 0.1f, 0.5f, "%.2f");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("freq Y (Hz)", &dwell.freq_y, 0.1f, 0.5f, "%.2f");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("phase X (rad)", &dwell.phase_x, 0.1f, 0.5f, "%.2f");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("phase Y (rad)", &dwell.phase_y, 0.1f, 0.5f, "%.2f");
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("dwell period (s)", &dwell.duration_s, 0.1f, 0.5f, "%.2f");
        ImGui::SetItemTooltip("How long to capture after the coarse scan is at full amplitude.");

        ImGui::Spacing();
        const bool sweeping = busy.load();
        const bool can_sweep = connected() && !sweeping;
        if (!can_sweep) ImGui::BeginDisabled();
        if (ImGui::Button("Sweep strip", ImVec2(160, 0)))
            start_sweep();
        if (!can_sweep) ImGui::EndDisabled();

        ImGui::Spacing();
        std::string local_status;
        std::string local_err;
        bool found = false;
        bool have = false;
        {
            std::lock_guard<std::mutex> lock(mu);
            local_status = status;
            local_err = last_error;
            have = have_result;
            found = have && analysis.hit.visibility == PointVisibility::Found;
        }
        if (sweeping)
            ImGui::TextColored(ImVec4(1.0f, 0.85f, 0.3f, 1), "Sweeping...");
        else if (have && found)
            ImGui::TextColored(ImVec4(0.35f, 0.9f, 0.45f, 1), "FOUND");
        else if (have)
            ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.35f, 1), "NOT_VISIBLE");
        ImGui::TextWrapped("%s", local_status.c_str());
        if (!local_err.empty())
            ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "%s", local_err.c_str());

        if (have) {
            std::lock_guard<std::mutex> lock(mu);
            ImGui::Spacing();
            ImGui::TextDisabled("Strip");
            ImGui::Text("captured %d   gated %d   bins %d",
                        static_cast<int>(strip.size()), analysis.gated_samples,
                        static_cast<int>(analysis.bins.size()));
            if (found) {
                ImGui::Text("pixel  el %.4f  mech (%.3f, %.3f)",
                            analysis.hit.elevation, analysis.hit.mechanical_x,
                            analysis.hit.mechanical_y);
                ImGui::Text("dwell samples %d", static_cast<int>(dwell_cloud.size()));
                if (!dwell_path.empty())
                    ImGui::TextWrapped("file  %s", dwell_path.c_str());
            }
        }
        ImGui::EndChild();
    }

    void draw_plot() {
        ImGui::BeginChild("plot", ImVec2(0, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("r = |(x, y)|  vs  elevation");
        ImGui::SameLine();
        ImGui::TextDisabled("dots = samples   squares = bins   yellow = target");
        ImGui::Separator();

        PointCloud local_strip;
        StripAnalysis local{};
        bool have = false;
        {
            std::lock_guard<std::mutex> lock(mu);
            have = have_result;
            if (have) {
                local_strip = strip;
                local = analysis;
            }
        }

        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const float left = 52.0f;
        const float bottom = 36.0f;
        const float top = 10.0f;
        const float right = 16.0f;
        const ImVec2 size(std::max(120.0f, avail.x), std::max(120.0f, avail.y - 8.0f));
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 p1(p0.x + size.x, p0.y + size.y);
        dl->AddRectFilled(p0, p1, IM_COL32(18, 20, 26, 255));

        const float el0 = finder_cfg.elev_min;
        const float el1 = finder_cfg.elev_max;
        float r0 = 0.0f;
        float r1 = std::max(1.0f, horizontal_range_xy(target_x, target_y) * 1.4f);
        if (have) {
            r1 = std::max(r1, local.target_horiz * 1.2f);
            for (const auto& p : local_strip.points()) {
                if (!p.valid) continue;
                r1 = std::max(r1, p.range * std::cos(p.elevation) * 1.05f);
            }
            for (const auto& b : local.bins) {
                if (b.occupied)
                    r1 = std::max(r1, b.horiz_range * 1.05f);
            }
        }

        const ImVec2 origin(p0.x + left, p1.y - bottom);
        const ImVec2 plot_tr(p1.x - right, p0.y + top);
        const ImVec2 plot_min(origin.x, plot_tr.y);
        const ImVec2 plot_max(plot_tr.x, origin.y);
        const float pw = plot_max.x - plot_min.x;
        const float ph = plot_max.y - plot_min.y;

        auto map = [&](float el, float r) {
            const float u = (el - el0) / (el1 - el0);
            const float v = (r - r0) / (r1 - r0);
            return ImVec2(origin.x + u * pw, origin.y - v * ph);
        };

        dl->AddRect(plot_min, plot_max, IM_COL32(70, 80, 95, 255));
        dl->AddLine(map(0.0f, r0), map(0.0f, r1), IM_COL32(70, 90, 70, 140));

        const float target_h = have ? local.target_horiz : horizontal_range_xy(target_x, target_y);
        dl->AddLine(map(el0, target_h), map(el1, target_h), IM_COL32(240, 210, 70, 220), 1.6f);

        if (have) {
            for (const auto& p : local_strip.points()) {
                if (!p.valid) continue;
                if (std::fabs(p.azimuth - local.target_azimuth) > finder_cfg.azimuth_gate_rad)
                    continue;
                const float rho = p.range * std::cos(p.elevation);
                dl->AddCircleFilled(map(p.elevation, rho), 1.6f, IM_COL32(90, 170, 220, 140));
            }
            for (const auto& b : local.bins) {
                if (!b.occupied) continue;
                const ImVec2 q = map(b.elevation, b.horiz_range);
                dl->AddRectFilled(ImVec2(q.x - 3.0f, q.y - 3.0f),
                                  ImVec2(q.x + 3.0f, q.y + 3.0f),
                                  IM_COL32(220, 230, 245, 230));
            }
            if (local.hit.visibility == PointVisibility::Found && local.hit.bin_index >= 0) {
                const ElevationBin& b =
                    local.bins[static_cast<std::size_t>(local.hit.bin_index)];
                const ImVec2 q = map(b.elevation, b.horiz_range);
                dl->AddLine(map(b.elevation, r0), map(b.elevation, r1),
                            IM_COL32(80, 220, 120, 200), 1.4f);
                dl->AddCircle(q, 10.0f, IM_COL32(80, 220, 120, 255), 0, 2.4f);
                dl->AddCircleFilled(q, 5.0f, IM_COL32(80, 220, 120, 255));
            } else {
                dl->AddRect(plot_min, plot_max, IM_COL32(220, 70, 60, 180), 0.0f, 0, 2.0f);
            }
        }

        char label[64];
        std::snprintf(label, sizeof(label), "%.2f", r1);
        dl->AddText(ImVec2(p0.x + 6.0f, plot_min.y), IM_COL32(160, 160, 170, 255), label);
        std::snprintf(label, sizeof(label), "%.2f", r0);
        dl->AddText(ImVec2(p0.x + 6.0f, origin.y - 14.0f), IM_COL32(160, 160, 170, 255), label);
        std::snprintf(label, sizeof(label), "%.2f", el0);
        dl->AddText(ImVec2(origin.x, origin.y + 6.0f), IM_COL32(160, 160, 170, 255), label);
        std::snprintf(label, sizeof(label), "%.2f", el1);
        dl->AddText(ImVec2(plot_max.x - 40.0f, origin.y + 6.0f), IM_COL32(160, 160, 170, 255), label);
        dl->AddText(ImVec2(origin.x + pw * 0.35f, origin.y + 18.0f),
                    IM_COL32(140, 145, 155, 255), "elevation (rad)");
        dl->AddText(ImVec2(p0.x + 4.0f, origin.y - ph * 0.55f),
                    IM_COL32(140, 145, 155, 255), "m");

        ImGui::Dummy(size);
        ImGui::EndChild();
    }

    void draw() {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("Azimuth Point Finder", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings);
        draw_connection();
        draw_controls();
        ImGui::SameLine();
        draw_plot();
        ImGui::End();
    }
};

AzimuthFinderApplet::AzimuthFinderApplet(Host host)
    : host_(std::move(host)), impl_(std::make_unique<Impl>(host_)) {}

AzimuthFinderApplet::~AzimuthFinderApplet() = default;

int AzimuthFinderApplet::run(std::atomic<bool>* keep_running) {
    glfwSetErrorCallback(glfw_error);
    if (!glfwInit()) {
        std::fprintf(stderr, "glfwInit failed\n");
        return 1;
    }

#if defined(__APPLE__)
    const char* glsl = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#else
    const char* glsl = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
#endif

    GLFWwindow* window = glfwCreateWindow(1280, 800, "Azimuth Point Finder", nullptr, nullptr);
    if (!window) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.0f;
    style.FrameRounding = 3.0f;

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl);

    impl_->try_initial_connect();

    while (!glfwWindowShouldClose(window)) {
        if (keep_running && !keep_running->load())
            glfwSetWindowShouldClose(window, GLFW_TRUE);

        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        impl_->draw();
        ImGui::Render();

        int w = 0, h = 0;
        glfwGetFramebufferSize(window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
