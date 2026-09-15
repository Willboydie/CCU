#include "ui/FrontendApplet.hpp"

#include "utils/frontend_protocol.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace {

constexpr int kMonitorMaxLines = 2000;

void glfw_error(int, const char* desc) {
    std::fprintf(stderr, "GLFW: %s\n", desc ? desc : "");
}

int index_of_port(const std::vector<std::string>& ports, const std::string& name) {
    for (int i = 0; i < static_cast<int>(ports.size()); ++i) {
        if (ports[static_cast<std::size_t>(i)] == name)
            return i;
    }
    return 0;
}

}  // namespace

struct FrontendApplet::Impl {
    Host& host;
    GLFWwindow* window = nullptr;

    std::vector<std::string> ports;
    int port_idx = 0;
    bool tried_initial = false;
    std::string banner;
    std::string last_error;

    FrontendTelemetry live = make_default_telemetry();
    FrontendTelemetry draft = make_default_telemetry();
    char firmware_buf[48] = "1.0.0";
    bool apply_rx = false;

    FrontendSurveyPoint survey{};
    std::vector<FrontendNode> proj_nodes;

    std::deque<std::string> monitor;
    uint64_t last_tel_n = 0;
    double last_hz_t = 0.0;
    float tel_hz = 0.0f;

    explicit Impl(Host& h) : host(h) {
        survey.nodeId = 1;
        survey.success = 1;
        proj_nodes.push_back(FrontendNode{1, 0.0f, 0.0f, 1.570796f, 0.0f});
    }

    template <typename Fn>
    void with_ctl(Fn&& fn) {
        if (!host.with_controller) {
            fn(static_cast<FrontendController*>(nullptr));
            return;
        }
        host.with_controller([&](FrontendController* c) { fn(c); });
    }

    bool on() const {
        return host.connected && host.connected();
    }

    void refresh_ports() {
        const std::string sel = ports.empty()
            ? host.initial_port
            : ports[static_cast<std::size_t>(port_idx)];
        ports = host.list_ports ? host.list_ports() : std::vector<std::string>{};
        port_idx = index_of_port(ports, sel);
        if (ports.empty())
            port_idx = 0;
    }

    void sync_live() {
        clamp_telemetry(live);
        with_ctl([&](FrontendController* c) {
            if (c)
                c->set_telemetry(live);
        });
    }

    void pull_echoed_live() {
        if (!apply_rx)
            return;
        with_ctl([&](FrontendController* c) {
            if (c)
                live = c->telemetry();
        });
    }

    void drain_log() {
        with_ctl([&](FrontendController* c) {
            if (!c)
                return;
            for (auto& line : c->take_log())
                monitor.push_back(std::move(line));
        });
        while (static_cast<int>(monitor.size()) > kMonitorMaxLines)
            monitor.pop_front();
    }

    void update_hz() {
        const double now = glfwGetTime();
        uint64_t n = 0;
        with_ctl([&](FrontendController* c) {
            if (c)
                n = c->telemetry_count();
        });
        if (last_hz_t <= 0.0) {
            last_hz_t = now;
            last_tel_n = n;
            return;
        }
        if (now - last_hz_t >= 0.5) {
            tel_hz = static_cast<float>(n - last_tel_n) / static_cast<float>(now - last_hz_t);
            last_tel_n = n;
            last_hz_t = now;
        }
    }

    void begin_on_connect() {
        with_ctl([&](FrontendController* c) {
            if (!c)
                return;
            c->set_apply_rx(apply_rx);
            c->set_telemetry(live);
            if (!c->begin_session(firmware_buf))
                last_error = "Failed to send FIRMWARE_VERSION";
        });
    }

    void try_initial_connect() {
        if (tried_initial)
            return;
        tried_initial = true;
        if (host.initial_port.empty() || on())
            return;
        std::string err;
        if (host.connect && host.connect(host.initial_port, err)) {
            last_error.clear();
            banner = host.initial_port;
            begin_on_connect();
        } else {
            last_error = err.empty() ? "Connect failed" : err;
        }
    }

    void draw_port_combo() {
        const char* preview = ports.empty()
            ? "(none)"
            : ports[static_cast<std::size_t>(port_idx)].c_str();
        if (ImGui::BeginCombo("##port", preview)) {
            for (int i = 0; i < static_cast<int>(ports.size()); ++i) {
                const bool sel = (i == port_idx);
                if (ImGui::Selectable(ports[static_cast<std::size_t>(i)].c_str(), sel))
                    port_idx = i;
                if (sel)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }

    void set_button(bool pending, const std::function<void()>& fn) {
        if (pending)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.82f, 0.52f, 0.12f, 1));
        if (ImGui::Button("Set", ImVec2(-FLT_MIN, 0)) && fn)
            fn();
        if (pending)
            ImGui::PopStyleColor();
    }

    void bool_row(const char* label, uint8_t FrontendTelemetry::* field) {
        ImGui::PushID(label);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(label);
        ImGui::TableNextColumn();
        ImGui::TextUnformatted((live.*field) ? "true" : "false");
        ImGui::TableNextColumn();
        bool v = draft.*field != 0;
        ImGui::Checkbox("##d", &v);
        draft.*field = v ? 1 : 0;
        ImGui::TableNextColumn();
        set_button((live.*field != 0) != v, [&] {
            live.*field = v ? 1 : 0;
            sync_live();
        });
        ImGui::PopID();
    }

    void u8_row(const char* label, uint8_t FrontendTelemetry::* field, int lo, int hi,
                bool combo, const char* const* items, int n_items) {
        ImGui::PushID(label);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(label);
        ImGui::TableNextColumn();
        if (combo && live.*field < static_cast<uint8_t>(n_items))
            ImGui::Text("%s", items[live.*field]);
        else
            ImGui::Text("%u", static_cast<unsigned>(live.*field));
        ImGui::TableNextColumn();
        int v = draft.*field;
        ImGui::SetNextItemWidth(-FLT_MIN);
        if (combo)
            ImGui::Combo("##d", &v, items, n_items);
        else
            ImGui::InputInt("##d", &v, 1, 10);
        v = std::clamp(v, lo, hi);
        draft.*field = static_cast<uint8_t>(v);
        ImGui::TableNextColumn();
        set_button(live.*field != draft.*field, [&] {
            live.*field = draft.*field;
            sync_live();
        });
        ImGui::PopID();
    }

    void float_row(const char* label, float& live_v, float& draft_v, bool angle) {
        ImGui::PushID(label);
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::AlignTextToFramePadding();
        ImGui::TextUnformatted(label);
        ImGui::TableNextColumn();
        ImGui::Text("%.4f", static_cast<double>(live_v));
        ImGui::TableNextColumn();
        ImGui::SetNextItemWidth(-FLT_MIN);
        ImGui::InputFloat("##d", &draft_v, 0.0f, 0.0f, "%.4f");
        ImGui::TableNextColumn();
        set_button(live_v != draft_v, [&] {
            live_v = angle ? wrap_2pi(draft_v) : draft_v;
            draft_v = live_v;
            sync_live();
        });
        ImGui::PopID();
    }

    void draw_connection() {
        ImGui::BeginChild("connection", ImVec2(0, 72), ImGuiChildFlags_Border);

        ImGui::SetNextItemWidth(140);
        draw_port_combo();
        ImGui::SameLine();
        if (ImGui::Button("Refresh"))
            refresh_ports();
        ImGui::SameLine();
        if (on()) {
            if (ImGui::Button("Disconnect") && host.disconnect) {
                host.disconnect();
                banner = "disconnected";
                tel_hz = 0.0f;
            }
        } else if (ImGui::Button("Connect")) {
            if (ports.empty()) {
                last_error = "No serial ports";
            } else {
                std::string err;
                const std::string& port = ports[static_cast<std::size_t>(port_idx)];
                if (host.connect && host.connect(port, err)) {
                    last_error.clear();
                    banner = port;
                    begin_on_connect();
                } else {
                    last_error = err.empty() ? "Connect failed" : err;
                }
            }
        }

        ImGui::SameLine();
        ImGui::Text("Link: %s", on()
            ? (host.port_name ? host.port_name().c_str() : "up")
            : "down");
        ImGui::SameLine();
        ImGui::TextDisabled("921600 8N1  DTR/RTS off");

        if (!last_error.empty()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "%s", last_error.c_str());
        }

        bool streaming = false;
        uint64_t tel_n = 0;
        uint64_t rx_n = 0;
        with_ctl([&](FrontendController* c) {
            if (!c)
                return;
            streaming = c->streaming();
            tel_n = c->telemetry_count();
            rx_n = c->rx_count();
        });

        if (!on())
            ImGui::BeginDisabled();
        if (ImGui::Checkbox("Telemetry 10 Hz", &streaming)) {
            with_ctl([&](FrontendController* c) {
                if (c)
                    c->set_streaming(streaming);
            });
        }
        ImGui::SameLine();
        if (ImGui::Button("Send telemetry once")) {
            bool ok = false;
            with_ctl([&](FrontendController* c) {
                if (!c)
                    return;
                c->set_telemetry(live);
                ok = c->send_telemetry_once();
            });
            if (!ok)
                last_error = "Telemetry send failed";
        }
        if (!on())
            ImGui::EndDisabled();

        ImGui::SameLine();
        ImGui::Text("TX %.1f Hz  n=%llu   RX %llu",
                    static_cast<double>(tel_hz),
                    static_cast<unsigned long long>(tel_n),
                    static_cast<unsigned long long>(rx_n));

        ImGui::EndChild();
    }

    void draw_telemetry() {
        ImGui::BeginChild("telemetry", ImVec2(0, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Telemetry (device → tablet)");
        ImGui::SameLine();
        ImGui::TextDisabled("live packet at 10 Hz");
        ImGui::SameLine(ImGui::GetWindowWidth() - 220);
        if (ImGui::SmallButton("Set all")) {
            live = draft;
            sync_live();
            draft = live;
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Copy live → draft"))
            draft = live;

        if (ImGui::BeginTable("tel", 4,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV |
                                  ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Field", ImGuiTableColumnFlags_WidthFixed, 150);
            ImGui::TableSetupColumn("Live", ImGuiTableColumnFlags_WidthFixed, 110);
            ImGui::TableSetupColumn("Draft");
            ImGui::TableSetupColumn("##set", ImGuiTableColumnFlags_WidthFixed, 48);
            ImGui::TableHeadersRow();

            bool_row("healthOK", &FrontendTelemetry::healthOK);
            bool_row("moved", &FrontendTelemetry::moved);
            bool_row("originIsSet", &FrontendTelemetry::originIsSet);
            bool_row("isSurveying", &FrontendTelemetry::isSurveying);
            bool_row("surveyReady", &FrontendTelemetry::surveyReady);
            bool_row("isProjecting", &FrontendTelemetry::isProjecting);
            bool_row("laserOn", &FrontendTelemetry::laserOn);
            bool_row("poseValid", &FrontendTelemetry::poseValid);

            static const char* kConn[] = {"Linked (0)", "Searching (1)", "Lost (2)"};
            static const char* kMode[] = {"Off (0)", "Survey (1)", "Project (2)"};
            u8_row("connectionStatus", &FrontendTelemetry::connectionStatus, 0, 2, true, kConn, 3);
            u8_row("projectionMode", &FrontendTelemetry::projectionMode, 0, 2, true, kMode, 3);
            u8_row("laserPower", &FrontendTelemetry::laserPower, 0, 100, false, nullptr, 0);
            u8_row("reserved", &FrontendTelemetry::reserved, 0, 255, false, nullptr, 0);

            float_row("effectiveYaw", live.effectiveYaw, draft.effectiveYaw, true);
            float_row("pose.east_mm", live.lastKnownPose.east_mm, draft.lastKnownPose.east_mm, false);
            float_row("pose.north_mm", live.lastKnownPose.north_mm, draft.lastKnownPose.north_mm, false);
            float_row("pose.up_mm", live.lastKnownPose.up_mm, draft.lastKnownPose.up_mm, false);
            float_row("pose.roll_rad", live.lastKnownPose.roll_rad, draft.lastKnownPose.roll_rad, true);
            float_row("pose.pitch_rad", live.lastKnownPose.pitch_rad, draft.lastKnownPose.pitch_rad, true);
            float_row("pose.yaw_rad", live.lastKnownPose.yaw_rad, draft.lastKnownPose.yaw_rad, true);

            ImGui::EndTable();
        }

        ImGui::Separator();
        ImGui::TextDisabled("Set copies the draft into the packet currently being streamed.");
        ImGui::EndChild();
    }

    void draw_oneshot() {
        ImGui::BeginChild("oneshot", ImVec2(0, 268), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("One-shot device frames");
        ImGui::Separator();

        const bool can = on();
        if (!can)
            ImGui::BeginDisabled();

        ImGui::SetNextItemWidth(160);
        ImGui::InputText("firmwareVersion", firmware_buf, sizeof(firmware_buf));
        ImGui::SameLine();
        if (ImGui::Button("Send FIRMWARE_VERSION")) {
            bool ok = false;
            with_ctl([&](FrontendController* c) {
                if (c)
                    ok = c->send_firmware_version(firmware_buf);
            });
            if (!ok)
                last_error = "FIRMWARE_VERSION send failed";
        }

        ImGui::Spacing();
        ImGui::TextUnformatted("SURVEY_POINT");
        int sid = survey.nodeId;
        ImGui::SetNextItemWidth(80);
        if (ImGui::InputInt("nodeId", &sid))
            survey.nodeId = static_cast<uint16_t>(std::clamp(sid, 0, 65535));
        ImGui::SameLine();
        bool ok = survey.success != 0;
        if (ImGui::Checkbox("success", &ok))
            survey.success = ok ? 1 : 0;
        ImGui::SetNextItemWidth(70);
        ImGui::InputFloat("east", &survey.pose.east_mm, 0, 0, "%.1f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        ImGui::InputFloat("north", &survey.pose.north_mm, 0, 0, "%.1f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        ImGui::InputFloat("up", &survey.pose.up_mm, 0, 0, "%.1f");
        ImGui::SetNextItemWidth(70);
        ImGui::InputFloat("roll", &survey.pose.roll_rad, 0, 0, "%.3f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        ImGui::InputFloat("pitch", &survey.pose.pitch_rad, 0, 0, "%.3f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        ImGui::InputFloat("yaw", &survey.pose.yaw_rad, 0, 0, "%.3f");
        ImGui::SameLine();
        if (ImGui::Button("Send SURVEY_POINT")) {
            bool sent = false;
            with_ctl([&](FrontendController* c) {
                if (c)
                    sent = c->send_survey_point(survey);
            });
            if (!sent)
                last_error = "SURVEY_POINT send failed";
        }

        ImGui::Spacing();
        ImGui::Text("PROJECTION_START  (%d nodes)", static_cast<int>(proj_nodes.size()));
        ImGui::SameLine();
        if (ImGui::SmallButton("Add node")) {
            FrontendNode n{};
            n.id = static_cast<uint16_t>(proj_nodes.size() + 1);
            n.corner_inner_angle_rad = 1.570796f;
            proj_nodes.push_back(n);
        }
        ImGui::SameLine();
        if (ImGui::SmallButton("Remove last") && !proj_nodes.empty())
            proj_nodes.pop_back();

        if (ImGui::BeginTable("proj_nodes", 6,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                                  ImGuiTableFlags_SizingStretchProp,
                              ImVec2(0, 88))) {
            ImGui::TableSetupColumn("id", ImGuiTableColumnFlags_WidthFixed, 50);
            ImGui::TableSetupColumn("east");
            ImGui::TableSetupColumn("north");
            ImGui::TableSetupColumn("inner");
            ImGui::TableSetupColumn("orient");
            ImGui::TableSetupColumn("##x", ImGuiTableColumnFlags_WidthFixed, 22);
            ImGui::TableHeadersRow();
            int remove = -1;
            for (int i = 0; i < static_cast<int>(proj_nodes.size()); ++i) {
                FrontendNode& n = proj_nodes[static_cast<std::size_t>(i)];
                ImGui::PushID(i);
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                int id = n.id;
                ImGui::SetNextItemWidth(-FLT_MIN);
                if (ImGui::InputInt("##id", &id, 0, 0))
                    n.id = static_cast<uint16_t>(std::clamp(id, 0, 65535));
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::InputFloat("##e", &n.east_mm, 0, 0, "%.1f");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::InputFloat("##n", &n.north_mm, 0, 0, "%.1f");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::InputFloat("##i", &n.corner_inner_angle_rad, 0, 0, "%.3f");
                ImGui::TableNextColumn();
                ImGui::SetNextItemWidth(-FLT_MIN);
                ImGui::InputFloat("##o", &n.corner_orientation_rad, 0, 0, "%.3f");
                ImGui::TableNextColumn();
                if (ImGui::SmallButton("x"))
                    remove = i;
                ImGui::PopID();
            }
            ImGui::EndTable();
            if (remove >= 0)
                proj_nodes.erase(proj_nodes.begin() + remove);
        }
        if (ImGui::Button("Send PROJECTION_START")) {
            bool sent = false;
            with_ctl([&](FrontendController* c) {
                if (c)
                    sent = c->send_projection_start(proj_nodes);
            });
            if (!sent)
                last_error = "PROJECTION_START send failed";
        }

        if (!can)
            ImGui::EndDisabled();
        ImGui::EndChild();
    }

    void draw_monitor() {
        ImGui::BeginChild("monitor", ImVec2(0, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Tablet commands");
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear"))
            monitor.clear();
        ImGui::SameLine();
        if (ImGui::Checkbox("Apply RX to telemetry", &apply_rx)) {
            with_ctl([&](FrontendController* c) {
                if (c)
                    c->set_apply_rx(apply_rx);
            });
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%d lines", static_cast<int>(monitor.size()));
        ImGui::Separator();
        ImGui::BeginChild("monitor_body");
        if (!on() && monitor.empty())
            ImGui::TextDisabled("Connect the PC dongle. Commands from the tablet appear here.");
        for (const auto& line : monitor) {
            if (line.starts_with("TX "))
                ImGui::TextColored(ImVec4(0.55f, 0.82f, 1.0f, 1), "%s", line.c_str());
            else if (line.find("bad ") != std::string::npos)
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1), "%s", line.c_str());
            else if (line.starts_with("RX "))
                ImGui::TextColored(ImVec4(0.45f, 0.92f, 0.55f, 1), "%s", line.c_str());
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
        ImGui::Begin("Marker ↔ tablet", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings);

        draw_connection();

        const ImVec2 rest = ImGui::GetContentRegionAvail();
        ImGui::BeginChild("body", ImVec2(0, 0));
        ImGui::BeginChild("left", ImVec2(rest.x * 0.48f, 0));
        draw_telemetry();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("right");
        draw_oneshot();
        draw_monitor();
        ImGui::EndChild();
        ImGui::EndChild();

        ImGui::End();
    }
};

FrontendApplet::FrontendApplet(Host host)
    : host_(std::move(host))
    , impl_(std::make_unique<Impl>(host_)) {}

FrontendApplet::~FrontendApplet() = default;

int FrontendApplet::run(std::atomic<bool>* keep_running) {
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

    impl_->window = glfwCreateWindow(1320, 860, "Marker ↔ Tablet (ESP-NOW dongle)",
                                     nullptr, nullptr);
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
        impl_->drain_log();
        impl_->pull_echoed_live();
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
