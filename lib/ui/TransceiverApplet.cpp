#include "ui/TransceiverApplet.hpp"

#include "utils/cmd_refs.hpp"
#include "utils/protocol.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace {

constexpr float kNeutralX = 3.54f;
constexpr float kNeutralY = 3.39f;
constexpr float kXMin = 2.5f;
constexpr float kXMax = 4.2f;
constexpr float kYMin = 2.6f;
constexpr float kYMax = 3.9f;
constexpr float kAmpMax = 0.6f;

void glfw_error(int, const char* desc) {
    std::fprintf(stderr, "GLFW: %s\n", desc ? desc : "");
}

ImVec2 map_xy(float x, float y, ImVec2 origin, ImVec2 size) {
    const float u = (x - kXMin) / (kXMax - kXMin);
    const float v = 1.0f - (y - kYMin) / (kYMax - kYMin);
    return ImVec2(origin.x + u * size.x, origin.y + v * size.y);
}

ImU32 dist_color(uint16_t cm) {
    const float t = std::clamp(cm / 800.0f, 0.0f, 1.0f);
    return IM_COL32(
        static_cast<int>(40 + 200 * t),
        static_cast<int>(180 - 80 * t),
        static_cast<int>(220 - 180 * t),
        220);
}

}  // namespace

struct TransceiverApplet::Impl {
    Host& host;
    GLFWwindow* window = nullptr;

    std::vector<std::string> ports;
    int port_idx = 0;
    bool tried_initial = false;
    std::string banner;
    std::string last_error;

    float p2p_x = kNeutralX;
    float p2p_y = kNeutralY;
    float coarse_x = kNeutralX;
    float coarse_y = kNeutralY;
    float amp_x = 0.2f;
    float amp_y = 0.2f;
    float freq_x = 1.5f;
    float freq_y = 3.0f;
    float phase_x = 0.0f;
    float phase_y = 0.0f;
    float direct_x = kNeutralX;
    float direct_y = kNeutralY;
    float ldm_preset = 0.0f;
    float gain = 0.0f;

    std::vector<float> dist_hist;
    struct Sample {
        float x = 0;
        float y = 0;
        uint16_t d = 0;
    };
    std::deque<Sample> trail;

    std::deque<std::string> log;
    uint64_t last_ctrl_n = 0;
    uint64_t last_stat_n = 0;
    double last_hz_t = 0.0;
    float ctrl_hz = 0.0f;
    float stat_hz = 0.0f;

    explicit Impl(Host& h) : host(h) {}

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

    void push_log(std::string line) {
        log.push_back(std::move(line));
        while (log.size() > 200)
            log.pop_front();
    }

    template <typename Fn>
    void with_ctl(Fn&& fn) {
        if (!host.with_controller) {
            fn(static_cast<LidarController*>(nullptr));
            return;
        }
        host.with_controller([&](LidarController* c) { fn(c); });
    }

    bool connected() const {
        return host.connected && host.connected();
    }

    void note_cmd(const char* name, bool ok) {
        if (ok)
            push_log(std::string(name) + " sent");
        else {
            last_error = std::string(name) + " failed";
            push_log(last_error);
        }
    }

    void drain_text() {
        with_ctl([&](LidarController* c) {
            if (!c) return;
            for (auto& line : c->take_text())
                push_log(std::move(line));
        });
    }

    void sample_telemetry() {
        with_ctl([&](LidarController* c) {
            if (!c || !c->has_control()) return;
            const ControlPayload p = c->control();
            dist_hist.push_back(static_cast<float>(p.distance));
            if (dist_hist.size() > 256)
                dist_hist.erase(dist_hist.begin());
            trail.push_back({p.associated_angle_x, p.associated_angle_y, p.distance});
            if (trail.size() > 1024)
                trail.pop_front();
        });
    }

    void update_hz() {
        const double now = glfwGetTime();
        if (last_hz_t == 0.0) {
            last_hz_t = now;
            if (host.control_count) last_ctrl_n = host.control_count();
            if (host.status_count) last_stat_n = host.status_count();
            return;
        }
        const double dt = now - last_hz_t;
        if (dt < 0.4) return;
        const uint64_t cn = host.control_count ? host.control_count() : 0;
        const uint64_t sn = host.status_count ? host.status_count() : 0;
        ctrl_hz = static_cast<float>((cn - last_ctrl_n) / dt);
        stat_hz = static_cast<float>((sn - last_stat_n) / dt);
        last_ctrl_n = cn;
        last_stat_n = sn;
        last_hz_t = now;
    }

    void try_initial_connect() {
        if (tried_initial) return;
        tried_initial = true;
        if (host.initial_port.empty() || connected()) return;
        std::string err;
        if (host.connect && host.connect(host.initial_port, err)) {
            banner = "Connected " + host.initial_port;
            push_log(banner);
        } else {
            last_error = err.empty() ? "Initial connect failed" : err;
            push_log(last_error);
        }
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
        if (connected()) {
            if (ImGui::Button("Disconnect") && host.disconnect) {
                host.disconnect();
                trail.clear();
                dist_hist.clear();
                banner = "Disconnected";
                push_log(banner);
            }
        } else if (ImGui::Button("Connect")) {
            if (ports.empty()) {
                last_error = "No serial ports";
            } else {
                std::string err;
                const std::string& port = ports[static_cast<std::size_t>(port_idx)];
                if (host.connect && host.connect(port, err)) {
                    last_error.clear();
                    banner = "Connected " + port;
                    push_log(banner);
                } else {
                    last_error = err.empty() ? "Connect failed" : err;
                    push_log(last_error);
                }
            }
        }

        ImGui::Text("Link: %s", connected()
            ? (host.port_name ? host.port_name().c_str() : "up")
            : "down");
        ImGui::SameLine(220);
        ImGui::Text("ctrl %.0f Hz  status %.1f Hz", ctrl_hz, stat_hz);
        if (!last_error.empty()) {
            ImGui::SameLine(460);
            ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "%s", last_error.c_str());
        }
        ImGui::EndChild();
    }

    void flag_pill(const char* label, bool on, const ImVec4& on_color) {
        ImGui::Text("%s", label);
        ImGui::SameLine();
        ImGui::TextColored(on ? on_color : ImVec4(0.55f, 0.55f, 0.55f, 1),
                           on ? "YES" : "no");
    }

    void draw_status(bool have, const StatusPayload& s) {
        ImGui::BeginChild("status", ImVec2(0, 168), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Status  (0xAA @ 10 Hz)");
        ImGui::Separator();
        if (!have) {
            ImGui::TextDisabled("waiting for 0xAA packets...");
            ImGui::EndChild();
            return;
        }
        flag_pill("enabled", s.enabled != 0, ImVec4(0.3f, 0.9f, 0.4f, 1));
        ImGui::SameLine(160);
        flag_pill("debug", s.debug_mode != 0, ImVec4(0.4f, 0.7f, 1, 1));
        ImGui::SameLine(300);
        flag_pill("fault", s.has_fault != 0, ImVec4(1, 0.35f, 0.3f, 1));

        const bool busy = s.dynamic_routine != kRoutineNone;
        ImGui::Text("scan  %s", scan_mode_name(s.scan_mode));
        ImGui::SameLine(160);
        ImGui::TextColored(
            busy ? ImVec4(1.0f, 0.8f, 0.2f, 1) : ImVec4(0.75f, 0.75f, 0.75f, 1),
            "routine  %s", dynamic_routine_name(s.dynamic_routine));

        ImGui::Text("coarse centre  (%.3f, %.3f) rad", s.coarse_centre_x, s.coarse_centre_y);
        ImGui::Text("p2p target     (%.3f, %.3f) rad", s.p2p_target_x, s.p2p_target_y);
        ImGui::EndChild();
    }

    void draw_control(bool have, const ControlPayload& p) {
        ImGui::BeginChild("control", ImVec2(0, 168), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Control  (0x55 @ 1 kHz)");
        ImGui::Separator();
        if (!have) {
            ImGui::TextDisabled("waiting for 0x55 packets...");
            ImGui::EndChild();
            return;
        }
        ImGui::Text("distance           %u cm", static_cast<unsigned>(p.distance));
        ImGui::Text("mechanical (X, Y)  (%.3f, %.3f) rad",
                    p.mechanical_angle_x, p.mechanical_angle_y);
        ImGui::Text("associated (X, Y)  (%.3f, %.3f) rad",
                    p.associated_angle_x, p.associated_angle_y);
        ImGui::Text("target     (X, Y)  (%.3f, %.3f) rad",
                    p.target_angle_x, p.target_angle_y);
        if (!dist_hist.empty()) {
            ImGui::PlotLines("##dist", dist_hist.data(),
                             static_cast<int>(dist_hist.size()), 0, "distance cm",
                             0.0f, 2000.0f, ImVec2(-1, 48));
        }
        ImGui::EndChild();
    }

    void draw_scan() {
        ImGui::BeginChild("scan", ImVec2(0, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Associated pointing  (point-cloud XY)");
        ImGui::Separator();
        const ImVec2 p0 = ImGui::GetCursorScreenPos();
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        const float side = std::max(80.0f, std::min(avail.x - 8.0f, avail.y - 8.0f));
        const ImVec2 size(side, side);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        const ImVec2 p1(p0.x + size.x, p0.y + size.y);
        dl->AddRectFilled(p0, p1, IM_COL32(18, 20, 26, 255));
        dl->AddRect(p0, p1, IM_COL32(70, 80, 95, 255));

        const ImVec2 neu = map_xy(kNeutralX, kNeutralY, p0, size);
        dl->AddLine(ImVec2(neu.x, p0.y), ImVec2(neu.x, p1.y), IM_COL32(70, 90, 70, 180));
        dl->AddLine(ImVec2(p0.x, neu.y), ImVec2(p1.x, neu.y), IM_COL32(70, 90, 70, 180));

        for (const Sample& s : trail)
            dl->AddCircleFilled(map_xy(s.x, s.y, p0, size), 1.6f, dist_color(s.d));

        with_ctl([&](LidarController* c) {
            if (!c) return;
            const bool have_ctrl = c->has_control();
            ControlPayload p{};
            if (have_ctrl) {
                p = c->control();
                dl->AddCircleFilled(
                    map_xy(p.mechanical_angle_x, p.mechanical_angle_y, p0, size),
                    5.0f, IM_COL32(255, 220, 80, 255));
                dl->AddCircle(
                    map_xy(p.target_angle_x, p.target_angle_y, p0, size),
                    7.0f, IM_COL32(120, 200, 255, 255), 0, 1.5f);
            }
            if (c->has_status()) {
                const StatusPayload s = c->status();
                dl->AddCircle(
                    map_xy(s.p2p_target_x, s.p2p_target_y, p0, size),
                    9.0f, IM_COL32(255, 140, 80, 220),
                    0, 1.5f);
            }
        });

        ImGui::Dummy(size);
        ImGui::TextDisabled("dot = mechanical   ring = FOC target   orange = p2p");
        ImGui::EndChild();
    }

    bool cmd_btn(const char* label, bool enabled,
                 const std::function<bool(LidarController&)>& fn) {
        if (!enabled) ImGui::BeginDisabled();
        const bool hit = ImGui::Button(label, ImVec2(168, 0));
        if (!enabled) ImGui::EndDisabled();
        if (hit) {
            bool ok = false;
            with_ctl([&](LidarController* c) {
                if (c) ok = fn(*c);
            });
            note_cmd(label, ok);
        }
        return hit;
    }

    void draw_commands(bool on) {
        ImGui::BeginChild("commands", ImVec2(0, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Commands");
        ImGui::Separator();

        ImGui::TextDisabled("System");
        cmd_btn("Enable", on, [](LidarController& c) { return c.set_enabled(true); });
        ImGui::SameLine();
        cmd_btn("Disable", on, [](LidarController& c) { return c.set_enabled(false); });
        ImGui::SameLine();
        cmd_btn("Debug on", on, [](LidarController& c) { return c.set_debug_mode(true); });
        ImGui::SameLine();
        cmd_btn("Debug off", on, [](LidarController& c) { return c.set_debug_mode(false); });
        cmd_btn("Toggle ANGLE/VEL", on, [](LidarController& c) { return c.toggle_control_mode(); });
        ImGui::SameLine();
        cmd_btn("Get errors", on, [](LidarController& c) { return c.get_error_msg(); });

        ImGui::Spacing();
        ImGui::TextDisabled("ScanEngine  (ON/OFF select path; motor enable starts target updates)");
        cmd_btn("SE ON", on, [](LidarController& c) { return c.scan_engine_on(); });
        ImGui::SameLine();
        cmd_btn("SE OFF", on, [](LidarController& c) { return c.scan_engine_off(); });
        ImGui::SameLine();
        cmd_btn("Seek neutral", on, [](LidarController& c) { return c.seek_neutral(); });
        cmd_btn("Begin coarse", on, [](LidarController& c) { return c.begin_coarse_scanning(); });
        ImGui::SameLine();
        cmd_btn("Stop coarse", on, [](LidarController& c) { return c.stop_coarse_scanning(); });

        ImGui::Spacing();
        ImGui::TextDisabled("Point-to-point target  [%.1f..%.1f] / [%.1f..%.1f] rad",
                            kXMin, kXMax, kYMin, kYMax);
        ImGui::SetNextItemWidth(120);
        ImGui::InputFloat("P2P X", &p2p_x, 0.01f, 0.1f, "%.3f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        ImGui::InputFloat("P2P Y", &p2p_y, 0.01f, 0.1f, "%.3f");
        ImGui::SameLine();
        cmd_btn("Send P2P", on, [&](LidarController& c) {
            return c.set_point_to_point_target(p2p_x, p2p_y);
        });
        ImGui::SameLine();
        if (ImGui::Button("Neutral pose")) {
            p2p_x = kNeutralX;
            p2p_y = kNeutralY;
        }

        ImGui::Spacing();
        ImGui::TextDisabled("Coarse scan");
        ImGui::SetNextItemWidth(120);
        ImGui::InputFloat("Centre X", &coarse_x, 0.01f, 0.1f, "%.3f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        ImGui::InputFloat("Centre Y", &coarse_y, 0.01f, 0.1f, "%.3f");
        ImGui::SameLine();
        cmd_btn("Update centre", on, [&](LidarController& c) {
            return c.update_coarse_scan_centre(coarse_x, coarse_y);
        });

        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("Ax", &amp_x, 0.01f, 0.05f, "%.3f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("Ay", &amp_y, 0.01f, 0.05f, "%.3f");
        ImGui::SameLine();
        cmd_btn("Set amp", on, [&](LidarController& c) {
            amp_x = std::clamp(amp_x, 0.0f, kAmpMax);
            amp_y = std::clamp(amp_y, 0.0f, kAmpMax);
            return c.set_x_amplitude(amp_x) && c.set_y_amplitude(amp_y);
        });

        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("Fx", &freq_x, 0.05f, 0.2f, "%.3f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("Fy", &freq_y, 0.05f, 0.2f, "%.3f");
        ImGui::SameLine();
        cmd_btn("Set freq", on, [&](LidarController& c) {
            return c.set_x_frequency(freq_x) && c.set_y_frequency(freq_y);
        });

        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("Px", &phase_x, 0.05f, 0.2f, "%.3f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("Py", &phase_y, 0.05f, 0.2f, "%.3f");
        ImGui::SameLine();
        cmd_btn("Set phase", on, [&](LidarController& c) {
            return c.set_x_phase(phase_x) && c.set_y_phase(phase_y);
        });
        

        if (ImGui::CollapsingHeader("LDM / lifecycle")) {
            ImGui::SetNextItemWidth(80);
            ImGui::InputFloat("Preset", &ldm_preset, 1.0f, 1.0f, "%.0f");
            ImGui::SameLine();
            cmd_btn("Reconfigure LDM", on, [&](LidarController& c) {
                return c.reconfigure_ldm(ldm_preset);
            });
            ImGui::TextDisabled("LDM reconfigure only while disabled");
            cmd_btn("Sleep", on, [](LidarController& c) { return c.sleep(); });
            ImGui::SameLine();
            cmd_btn("Wake", on, [](LidarController& c) { return c.wake(); });
            ImGui::SameLine();
            cmd_btn("Restart", on, [](LidarController& c) { return c.restart(); });
            ImGui::SameLine();
            cmd_btn("Bootloader", on, [](LidarController& c) { return c.enter_bootloader(); });
        }

        if (ImGui::CollapsingHeader("Controller gains")) {
            ImGui::SetNextItemWidth(100);
            ImGui::InputFloat("value", &gain, 0.01f, 0.1f, "%.4f");
            const struct {
                const char* name;
                uint8_t cmd;
            } gains[] = {
                {"Vel P X", kCmdVelocityPXSet}, {"Vel I X", kCmdVelocityIXSet},
                {"Vel D X", kCmdVelocityDXSet}, {"Vel P Y", kCmdVelocityPYSet},
                {"Vel I Y", kCmdVelocityIYSet}, {"Vel D Y", kCmdVelocityDYSet},
                {"Ang P X", kCmdAnglePXSet}, {"Ang I X", kCmdAngleIXSet},
                {"Ang D X", kCmdAngleDXSet}, {"Ang P Y", kCmdAnglePYSet},
                {"Ang I Y", kCmdAngleIYSet}, {"Ang D Y", kCmdAngleDYSet},
                {"Vel LPF", kCmdVelocityLpfTauSet}, {"Ang LPF", kCmdAngleLpfTauSet},
                {"FOC LPF", kCmdFocVoltageLpfTauSet}, {"Vel FF", kCmdVelocityFfSet},
                {"Acc FF", kCmdAccelerationFfSet},
            };
            for (int i = 0; i < static_cast<int>(std::size(gains)); ++i) {
                if (i % 3 != 0) ImGui::SameLine();
                cmd_btn(gains[i].name, on, [&, i](LidarController& c) {
                    return c.send_f32(gains[i].cmd, {gain});
                });
            }
        }

        ImGui::EndChild();
    }

    void draw_log() {
        ImGui::BeginChild("log", ImVec2(0, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Device log  (debug 0xFF)");
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear")) log.clear();
        ImGui::Separator();
        ImGui::BeginChild("logbody");
        for (const auto& line : log) {
            if (line.starts_with("[debug] "))
                ImGui::TextColored(ImVec4(0.55f, 0.82f, 1.0f, 1), "%s", line.c_str());
            else if (line.starts_with("[text] "))
                ImGui::TextColored(ImVec4(1.0f, 0.78f, 0.42f, 1), "%s", line.c_str());
            else
                ImGui::TextUnformatted(line.c_str());
        }
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 8)
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
        ImGui::EndChild();
    }

    void draw() {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("LiDAR Transceiver", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings);

        draw_connection();

        bool on = false;
        bool have_status = false;
        bool have_control = false;
        StatusPayload status{};
        ControlPayload control{};
        with_ctl([&](LidarController* c) {
            on = c != nullptr;
            if (!c) return;
            have_status = c->has_status();
            have_control = c->has_control();
            if (have_status) status = c->status();
            if (have_control) control = c->control();
        });

        ImGui::BeginChild("mid", ImVec2(0, 176));
        ImGui::BeginChild("midL", ImVec2(ImGui::GetContentRegionAvail().x * 0.5f - 4, 0));
        draw_status(have_status, status);
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("midR", ImVec2(0, 0));
        draw_control(have_control, control);
        ImGui::EndChild();
        ImGui::EndChild();

        const float bottom = 150.0f;
        const ImVec2 rest = ImGui::GetContentRegionAvail();
        ImGui::BeginChild("lower", ImVec2(0, rest.y - bottom - 4));
        ImGui::BeginChild("scanwrap", ImVec2(rest.x * 0.38f, 0));
        draw_scan();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("cmdwrap", ImVec2(0, 0));
        draw_commands(on);
        ImGui::EndChild();
        ImGui::EndChild();

        draw_log();
        ImGui::End();
    }
};

TransceiverApplet::TransceiverApplet(Host host)
    : host_(std::move(host))
    , impl_(std::make_unique<Impl>(host_)) {}

TransceiverApplet::~TransceiverApplet() = default;

int TransceiverApplet::run(std::atomic<bool>* keep_running) {
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

    impl_->window = glfwCreateWindow(1280, 800, "LiDAR Transceiver", nullptr, nullptr);
    if (!impl_->window) {
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(impl_->window);
    glfwSwapInterval(1);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 0.0f;
    style.FrameRounding = 3.0f;

    ImGui_ImplGlfw_InitForOpenGL(impl_->window, true);
    ImGui_ImplOpenGL3_Init(glsl);

    impl_->refresh_ports();
    impl_->try_initial_connect();

    while (!glfwWindowShouldClose(impl_->window)) {
        if (keep_running && !keep_running->load())
            glfwSetWindowShouldClose(impl_->window, GLFW_TRUE);

        glfwPollEvents();
        impl_->drain_text();
        impl_->sample_telemetry();
        impl_->update_hz();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        impl_->draw();
        ImGui::Render();

        int w = 0, h = 0;
        glfwGetFramebufferSize(impl_->window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.08f, 0.08f, 0.10f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(impl_->window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(impl_->window);
    impl_->window = nullptr;
    glfwTerminate();
    return 0;
}
