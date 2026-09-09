#include "ui/ProjectorApplet.hpp"

#include "utils/projector_protocol.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdio>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr int kDutyMin = 15;
constexpr int kDutyMax = 99;
constexpr int kCentreDefault =
    (static_cast<int>(kProjPathDacMin) + static_cast<int>(kProjPathDacMax)) / 2;
constexpr int kRadiusDefault = 4000;
constexpr int kPathIncrementDefault = 64;
constexpr int kStreamRateMinKhz = 1;
constexpr int kStreamRateMaxKhz = 100;
constexpr int kStreamRateDefaultKhz = 20;

void glfw_error(int, const char* desc) {
    std::fprintf(stderr, "GLFW: %s\n", desc ? desc : "");
}

}  // namespace

struct ProjectorApplet::Impl {
    Host& host;
    GLFWwindow* window = nullptr;

    std::vector<std::string> ports;
    int port_idx = 0;
    bool tried_initial = false;
    std::string banner;
    std::string last_error;
    int increment = 256;
    int duty_percent = 50;
    int centre_x = kCentreDefault;
    int centre_y = kCentreDefault;
    int radius = kRadiusDefault;
    int path_increment = kPathIncrementDefault;
    int stream_rate_khz = kStreamRateDefaultKhz;
    int stream_cx = kCentreDefault;
    int stream_cy = kCentreDefault;
    int stream_r = kRadiusDefault;

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

    template <typename Fn>
    void with_ctl(Fn&& fn) {
        if (!host.with_controller) {
            fn(static_cast<ProjectorController*>(nullptr));
            return;
        }
        host.with_controller([&](ProjectorController* c) { fn(c); });
    }

    bool connected() const {
        return host.connected && host.connected();
    }

    void note_cmd(const char* name, bool ok) {
        if (ok)
            last_error.clear();
        else
            last_error = std::string(name) + " failed";
    }

    void drain_text() {
        with_ctl([&](ProjectorController* c) {
            if (c) (void)c->take_text();
        });
    }

    void poll_status() {
        with_ctl([&](ProjectorController* c) {
            if (c) c->request_status();
        });
    }

    void try_initial_connect() {
        if (tried_initial) return;
        tried_initial = true;
        if (host.initial_port.empty() || connected()) return;
        std::string err;
        if (host.connect && host.connect(host.initial_port, err)) {
            banner = "Connected " + host.initial_port;
        } else {
            last_error = err.empty() ? "Initial connect failed" : err;
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
                with_ctl([&](ProjectorController* c) {
                    if (c) (void)c->stop_path_stream();
                });
                host.disconnect();
                banner = "Disconnected";
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
                } else {
                    last_error = err.empty() ? "Connect failed" : err;
                }
            }
        }

        ImGui::Text("Link: %s", connected()
            ? (host.port_name ? host.port_name().c_str() : "up")
            : "down");
        if (!last_error.empty()) {
            ImGui::SameLine(220);
            ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "%s", last_error.c_str());
        }
        ImGui::EndChild();
    }

    void draw_status(bool have, const ProjectorStatus& s) {
        ImGui::BeginChild("status", ImVec2(0, 88), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Device status");
        ImGui::Separator();
        if (!have) {
            ImGui::TextDisabled("waiting for status...");
            ImGui::EndChild();
            return;
        }
        ImGui::Text("power %s   mode %s   enabled %s   projecting %s",
                    proj_power_name(s.power_state),
                    proj_mode_name(s.project_mode),
                    s.enabled ? "yes" : "no",
                    s.projecting ? "yes" : "no");
        ImGui::Text("duty %u%%   stream %s   meas %u kHz   fault %s",
                    static_cast<unsigned>(s.laser_duty_cycle),
                    proj_stream_name(s.stream_state),
                    static_cast<unsigned>(s.points_frequency),
                    s.has_fault ? "YES" : "no");
        ImGui::EndChild();
    }

    void clamp_path_fields() {
        centre_x = std::clamp(centre_x, static_cast<int>(kProjPathDacMin),
                              static_cast<int>(kProjPathDacMax));
        centre_y = std::clamp(centre_y, static_cast<int>(kProjPathDacMin),
                              static_cast<int>(kProjPathDacMax));
        const int max_r = max_circle_radius(centre_x, centre_y);
        radius = std::clamp(radius, 0, max_r);
        path_increment = std::clamp(path_increment, 1, static_cast<int>(kProjDacMax));
        stream_rate_khz = std::clamp(stream_rate_khz, kStreamRateMinKhz, kStreamRateMaxKhz);
        duty_percent = std::clamp(duty_percent, kDutyMin, kDutyMax);
        increment = std::clamp(increment, 1, static_cast<int>(kProjDacMax));
    }

    void draw_mode_toggle(bool on, bool have_status, uint8_t mode, bool streaming) {
        ImGui::TextUnformatted("Project mode");
        ImGui::SameLine();
        const bool can = on && have_status && !streaming;
        const bool path = mode == kProjModePath;

        if (!can) ImGui::BeginDisabled();

        if (path)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.45f, 0.28f, 1));
        if (ImGui::Button("PATH", ImVec2(88, 28)) && !path) {
            bool ok = false;
            with_ctl([&](ProjectorController* c) {
                if (c) ok = c->set_project_mode(kProjModePath);
            });
            note_cmd("PATH", ok);
        }
        if (path) ImGui::PopStyleColor();

        ImGui::SameLine();
        if (!path)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.45f, 0.28f, 1));
        if (ImGui::Button("POINT", ImVec2(88, 28)) && path) {
            bool ok = false;
            with_ctl([&](ProjectorController* c) {
                if (c) ok = c->set_project_mode(kProjModePoint);
            });
            note_cmd("POINT", ok);
        }
        if (!path) ImGui::PopStyleColor();

        if (!can) ImGui::EndDisabled();
        if (streaming) {
            ImGui::SameLine();
            ImGui::TextDisabled("stop stream to change mode");
        }
    }

    void draw_duty(bool on) {
        if (!on) ImGui::BeginDisabled();
        ImGui::SetNextItemWidth(220);
        ImGui::SliderInt("Duty %", &duty_percent, kDutyMin, kDutyMax);
        ImGui::SameLine();
        if (ImGui::Button("Set##duty", ImVec2(72, 0))) {
            bool ok = false;
            with_ctl([&](ProjectorController* c) {
                if (c) ok = c->set_laser_duty_cycle(static_cast<uint16_t>(duty_percent));
            });
            note_cmd("Duty", ok);
        }
        if (!on) ImGui::EndDisabled();
    }

    bool pad_btn(const char* label, bool enabled, int dx, int dy) {
        if (!enabled) ImGui::BeginDisabled();
        const bool hit = ImGui::Button(label, ImVec2(72, 40));
        if (!enabled) ImGui::EndDisabled();
        if (!hit) return false;
        bool ok = false;
        with_ctl([&](ProjectorController* c) {
            if (c) ok = c->nudge(dx, dy);
        });
        note_cmd(label, ok);
        return hit;
    }

    void draw_point_controls(bool on, bool have_status, bool pointing,
                             uint16_t x, uint16_t y) {
        ImGui::TextUnformatted("Point mode");
        ImGui::Separator();

        const bool can_arm = on && have_status;
        const bool can_nudge = on && pointing;

        if (!can_arm) ImGui::BeginDisabled();
        const char* arm_label = pointing ? "Disable" : "Enable";
        if (ImGui::Button(arm_label, ImVec2(100, 32))) {
            bool ok = false;
            with_ctl([&](ProjectorController* c) {
                if (!c) return;
                ok = pointing ? c->disable_pointing()
                              : c->enable_pointing(static_cast<uint16_t>(duty_percent));
            });
            note_cmd(arm_label, ok);
        }
        if (!can_arm) ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        ImGui::InputInt("Increment", &increment);
        ImGui::SetItemTooltip(
            "DAC steps applied by each pad press. Commanded position is held "
            "on the host; the device is updated with SET_POINT.");

        ImGui::Spacing();
        const int inc = increment;
        const float btn_w = 72.0f;
        ImGui::Dummy(ImVec2(btn_w, 0));
        ImGui::SameLine();
        pad_btn("Up", can_nudge, 0, inc);

        pad_btn("Left", can_nudge, inc, 0);
        ImGui::SameLine();
        ImGui::Dummy(ImVec2(btn_w, 0));
        ImGui::SameLine();
        pad_btn("Right", can_nudge, -inc, 0);

        ImGui::Dummy(ImVec2(btn_w, 0));
        ImGui::SameLine();
        pad_btn("Down", can_nudge, 0, -inc);

        ImGui::Spacing();
        ImGui::Text("Commanded  X %u   Y %u   (centre %u)",
                    static_cast<unsigned>(x), static_cast<unsigned>(y),
                    static_cast<unsigned>(kProjDacCentre));
    }

    void draw_path_controls(bool on, bool have_status, bool streaming) {
        ImGui::TextUnformatted("Path mode");
        ImGui::Separator();
        ImGui::TextDisabled("Circle DAC range %u–%u",
                            static_cast<unsigned>(kProjPathDacMin),
                            static_cast<unsigned>(kProjPathDacMax));

        const bool can_geom = on && have_status;
        const bool can_setup = on && have_status && !streaming;
        const bool can_run = on && have_status;
        const int max_r = max_circle_radius(centre_x, centre_y);

        if (!can_geom) ImGui::BeginDisabled();
        ImGui::SetNextItemWidth(120);
        ImGui::InputInt("Centre X", &centre_x);
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        ImGui::InputInt("Centre Y", &centre_y);

        ImGui::SetNextItemWidth(120);
        ImGui::InputInt("Radius", &radius);
        ImGui::SameLine();
        ImGui::TextDisabled("max %d", max_r);
        if (!can_geom) ImGui::EndDisabled();

        if (!can_setup) ImGui::BeginDisabled();
        ImGui::SetNextItemWidth(120);
        ImGui::InputInt("Path increment", &path_increment);
        ImGui::SetItemTooltip(
            "Arc step between successive circle points, in DAC counts.");

        ImGui::SetNextItemWidth(120);
        ImGui::InputInt("Stream rate (kHz)", &stream_rate_khz);
        ImGui::SetItemTooltip("SET_PLAYBACK_SPEED: point rate in kHz (1–100).");
        if (!can_setup) ImGui::EndDisabled();

        clamp_path_fields();
        if (streaming &&
            (centre_x != stream_cx || centre_y != stream_cy || radius != stream_r)) {
            stream_cx = centre_x;
            stream_cy = centre_y;
            stream_r = radius;
            auto pts = generate_circle_path(centre_x, centre_y, radius,
                                            path_increment);
            with_ctl([&](ProjectorController* c) {
                if (c) (void)c->update_path_stream(std::move(pts));
            });
        }
        ImGui::Text("Circle  %d points   %d DAC/step   %d kHz",
                    circle_path_point_count(radius, path_increment),
                    path_increment, stream_rate_khz);

        ImGui::Spacing();
        if (!can_run) ImGui::BeginDisabled();
        const char* run_label = streaming ? "Stop" : "Start";
        if (ImGui::Button(run_label, ImVec2(120, 32))) {
            bool ok = false;
            with_ctl([&](ProjectorController* c) {
                if (!c) return;
                if (streaming) {
                    ok = c->stop_path_stream();
                } else {
                    stream_cx = centre_x;
                    stream_cy = centre_y;
                    stream_r = radius;
                    auto pts = generate_circle_path(centre_x, centre_y, radius,
                                                    path_increment);
                    ok = c->start_path_stream(std::move(pts),
                                              static_cast<uint16_t>(stream_rate_khz),
                                              static_cast<uint16_t>(duty_percent));
                }
            });
            note_cmd(run_label, ok);
        }
        if (!can_run) ImGui::EndDisabled();
        if (streaming) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(0.45f, 0.9f, 0.5f, 1), "streaming circle");
        }
    }

    void draw_controls(bool on, bool have_status, bool pointing, bool streaming,
                       uint8_t mode, uint16_t x, uint16_t y) {
        ImGui::BeginChild("controls", ImVec2(0, 0), ImGuiChildFlags_Border);
        clamp_path_fields();
        draw_mode_toggle(on, have_status, mode, streaming);
        ImGui::Spacing();
        draw_duty(on);
        ImGui::Spacing();

        if (mode == kProjModePath)
            draw_path_controls(on, have_status, streaming);
        else
            draw_point_controls(on, have_status, pointing, x, y);

        ImGui::EndChild();
    }

    void draw() {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("Projector", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings);

        draw_connection();

        bool on = false;
        bool have_status = false;
        bool pointing = false;
        bool streaming = false;
        uint16_t x = kProjDacCentre;
        uint16_t y = kProjDacCentre;
        ProjectorStatus status{};
        with_ctl([&](ProjectorController* c) {
            on = c != nullptr;
            if (!c) return;
            have_status = c->has_status();
            pointing = c->pointing();
            streaming = c->streaming();
            x = c->x();
            y = c->y();
            if (have_status) status = c->status();
        });

        draw_status(have_status, status);
        const uint8_t mode = have_status ? status.project_mode : kProjModePath;
        draw_controls(on, have_status, pointing, streaming, mode, x, y);
        ImGui::End();
    }
};

ProjectorApplet::ProjectorApplet(Host host)
    : host_(std::move(host))
    , impl_(std::make_unique<Impl>(host_)) {}

ProjectorApplet::~ProjectorApplet() = default;

int ProjectorApplet::run(std::atomic<bool>* keep_running) {
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

    impl_->window = glfwCreateWindow(620, 680, "Projector", nullptr, nullptr);
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
        impl_->poll_status();
        impl_->with_ctl([](ProjectorController* c) {
            if (c) c->pump_stream();
        });

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

    impl_->with_ctl([](ProjectorController* c) {
        if (c) (void)c->stop_path_stream();
    });

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(impl_->window);
    impl_->window = nullptr;
    glfwTerminate();
    return 0;
}
