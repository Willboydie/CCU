#include "ui/ProjectorCalibrationApplet.hpp"

#include "utils/projector_calibration.hpp"
#include "utils/projector_protocol.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace {

constexpr int kDutyMin = 15;
constexpr int kDutyMax = 99;

void glfw_error(int, const char* desc) {
    std::fprintf(stderr, "GLFW: %s\n", desc ? desc : "");
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

std::string calib_output_path() {
    const auto tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    char name[80];
    std::snprintf(name, sizeof(name),
                  "galvo_slope_%04d%02d%02d_%02d%02d%02d.bin",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                  tm.tm_hour, tm.tm_min, tm.tm_sec);
    const auto dir = executable_dir() / "calibration";
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    return (dir / name).string();
}

}  // namespace

struct ProjectorCalibrationApplet::Impl {
    Host& host;
    GLFWwindow* window = nullptr;

    std::vector<std::string> ports;
    int port_idx = 0;
    bool tried_initial = false;
    std::string banner;
    std::string last_error;
    std::string status = "Enable, move to a start pose, then Begin measure.";
    int increment = 256;
    int duty_percent = 50;

    float range_m = 1.0f;
    int axis_idx = 0;  // 0 = X, 1 = Y

    bool measuring = false;
    uint8_t measure_axis = kCalibAxisX;
    uint16_t measure_start = kProjDacCentre;
    float measure_range_m = 0.0f;

    std::vector<ProjectorCalibDatum> logged;

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

    uint16_t axis_value(uint8_t axis, uint16_t x, uint16_t y) const {
        return axis == kCalibAxisY ? y : x;
    }

    void begin_measure(uint16_t x, uint16_t y) {
        if (!(range_m > 0.0f) || !std::isfinite(range_m)) {
            last_error = "Enter a positive rangefinder length";
            status = last_error;
            return;
        }
        measure_axis = (axis_idx == 1) ? kCalibAxisY : kCalibAxisX;
        measure_start = axis_value(measure_axis, x, y);
        measure_range_m = range_m;
        measuring = true;
        last_error.clear();
        char buf[128];
        std::snprintf(buf, sizeof(buf),
                      "Measuring %s from %u  range %.4f m. Traverse 1 cm, then Log derivative.",
                      calib_axis_name(measure_axis),
                      static_cast<unsigned>(measure_start),
                      measure_range_m);
        status = buf;
    }

    void log_derivative(uint16_t x, uint16_t y) {
        if (!measuring) {
            last_error = "Click Begin measure first";
            status = last_error;
            return;
        }
        const uint16_t now = axis_value(measure_axis, x, y);
        const int delta = static_cast<int>(now) - static_cast<int>(measure_start);
        float deriv = 0.0f;
        if (!calib_derivative(measure_range_m, delta, deriv)) {
            last_error = "DAC increment is zero; move 1 cm before logging";
            status = last_error;
            return;
        }
        ProjectorCalibDatum d{};
        d.axis = measure_axis;
        d.start = measure_start;
        d.derivative = deriv;
        logged.push_back(d);
        measuring = false;
        last_error.clear();
        char buf[160];
        std::snprintf(buf, sizeof(buf),
                      "Logged %s: %u  (ΔDAC %+d, f' %.6e rad/count)",
                      calib_axis_name(d.axis),
                      static_cast<unsigned>(d.start),
                      delta, deriv);
        status = buf;
    }

    void save_data() {
        if (logged.empty()) {
            last_error = "No derivatives logged";
            status = last_error;
            return;
        }
        const std::string path = calib_output_path();
        std::ofstream out(path, std::ios::binary);
        if (!out) {
            last_error = "Failed to write " + path;
            status = last_error;
            return;
        }
        for (const auto& d : logged) {
            out.write(reinterpret_cast<const char*>(&d), sizeof(d));
            if (!out) {
                last_error = "Write failed: " + path;
                status = last_error;
                return;
            }
        }
        out.close();
        last_error.clear();
        char buf[256];
        std::snprintf(buf, sizeof(buf), "Saved %d samples to %s",
                      static_cast<int>(logged.size()), path.c_str());
        status = buf;
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
                measuring = false;
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
        ImGui::BeginChild("status", ImVec2(0, 72), ImGuiChildFlags_Border);
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
        ImGui::Text("duty %u%%   fault %s",
                    static_cast<unsigned>(s.laser_duty_cycle),
                    s.has_fault ? "YES" : "no");
        ImGui::EndChild();
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
        ImGui::BeginChild("point", ImVec2(0, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Point mode");
        ImGui::Separator();

        increment = std::clamp(increment, 1, static_cast<int>(kProjDacMax));
        duty_percent = std::clamp(duty_percent, kDutyMin, kDutyMax);
        const bool can_arm = on && have_status;
        const bool can_nudge = on && pointing;
        const bool can_duty = on;

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

        ImGui::Spacing();
        if (!can_duty) ImGui::BeginDisabled();
        ImGui::SetNextItemWidth(180);
        ImGui::SliderInt("Duty %", &duty_percent, kDutyMin, kDutyMax);
        ImGui::SameLine();
        if (ImGui::Button("Set", ImVec2(72, 0))) {
            bool ok = false;
            with_ctl([&](ProjectorController* c) {
                if (c) ok = c->set_laser_duty_cycle(static_cast<uint16_t>(duty_percent));
            });
            note_cmd("Duty", ok);
        }
        if (!can_duty) ImGui::EndDisabled();
        ImGui::EndChild();
    }

    void draw_calibration(bool pointing, uint16_t x, uint16_t y) {
        ImGui::BeginChild("calib", ImVec2(0, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Slope calibration");
        ImGui::Separator();
        ImGui::TextWrapped(
            "At a start pose, align the target so the beam is normal and centred. "
            "Enter rangefinder length, pick the axis you will sweep, Begin measure, "
            "nudge until the dot travels 1 cm, then Log derivative.");

        ImGui::Spacing();
        ImGui::SetNextItemWidth(140);
        ImGui::InputFloat("Range (m)", &range_m, 0.01f, 0.1f, "%.4f");
        ImGui::SetItemTooltip(
            "Distance from galvo mirror to target surface, metres. "
            "f' = (0.01 m / range) / ΔDAC.");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(80);
        const char* axes[] = {"X", "Y"};
        ImGui::Combo("Axis", &axis_idx, axes, 2);

        const bool can_begin = pointing;
        const bool can_log = pointing && measuring;
        const bool can_save = !logged.empty();

        if (!can_begin) ImGui::BeginDisabled();
        if (ImGui::Button("Begin measure", ImVec2(140, 28)))
            begin_measure(x, y);
        if (!can_begin) ImGui::EndDisabled();
        ImGui::SameLine();
        if (!can_log) ImGui::BeginDisabled();
        if (ImGui::Button("Log derivative", ImVec2(140, 28)))
            log_derivative(x, y);
        if (!can_log) ImGui::EndDisabled();
        ImGui::SameLine();
        if (!can_save) ImGui::BeginDisabled();
        if (ImGui::Button("Save data", ImVec2(120, 28)))
            save_data();
        if (!can_save) ImGui::EndDisabled();

        if (measuring) {
            const uint16_t now = axis_value(measure_axis, x, y);
            const int delta = static_cast<int>(now) - static_cast<int>(measure_start);
            ImGui::Text("Armed  %s: %u   now %u   ΔDAC %+d   range %.4f m",
                        calib_axis_name(measure_axis),
                        static_cast<unsigned>(measure_start),
                        static_cast<unsigned>(now),
                        delta, measure_range_m);
        } else {
            ImGui::TextDisabled("Not measuring");
        }

        ImGui::Spacing();
        ImGui::TextUnformatted("Logged derivatives");
        ImGui::BeginChild("logged", ImVec2(0, 160), ImGuiChildFlags_Border);
        if (logged.empty()) {
            ImGui::TextDisabled("(none)");
        } else {
            for (const auto& d : logged)
                ImGui::Text("%s: %u", calib_axis_name(d.axis),
                            static_cast<unsigned>(d.start));
        }
        ImGui::EndChild();

        ImGui::Spacing();
        ImGui::TextWrapped("%s", status.c_str());
        ImGui::EndChild();
    }

    void draw() {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("Projector Calibration", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings);

        draw_connection();

        bool on = false;
        bool have_status = false;
        bool pointing = false;
        uint16_t x = kProjDacCentre;
        uint16_t y = kProjDacCentre;
        ProjectorStatus dev{};
        with_ctl([&](ProjectorController* c) {
            on = c != nullptr;
            if (!c) return;
            have_status = c->has_status();
            pointing = c->pointing();
            x = c->x();
            y = c->y();
            if (have_status) dev = c->status();
        });

        draw_status(have_status, dev);

        const ImVec2 rest = ImGui::GetContentRegionAvail();
        ImGui::BeginChild("lower", ImVec2(0, rest.y));
        ImGui::BeginChild("left", ImVec2(rest.x * 0.46f, 0));
        draw_point_controls(on, have_status, pointing, x, y);
        ImGui::EndChild();
        ImGui::SameLine();
        draw_calibration(pointing, x, y);
        ImGui::EndChild();

        ImGui::End();
    }
};

ProjectorCalibrationApplet::ProjectorCalibrationApplet(Host host)
    : host_(std::move(host))
    , impl_(std::make_unique<Impl>(host_)) {}

ProjectorCalibrationApplet::~ProjectorCalibrationApplet() = default;

int ProjectorCalibrationApplet::run(std::atomic<bool>* keep_running) {
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

    impl_->window = glfwCreateWindow(960, 620, "Projector Calibration", nullptr, nullptr);
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
