#include "ui/PromoProjectApplet.hpp"

#include "bearing.hpp"
#include "device_config.hpp"
#include "path_engine.hpp"
#include "utils/projector_protocol.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr int kDutyMin = 15;
constexpr int kDutyMax = 99;
constexpr int kRateMinKhz = 1;
constexpr int kRateMaxKhz = 100;

void glfw_error(int, const char* desc) {
    std::fprintf(stderr, "GLFW: %s\n", desc ? desc : "");
}

float yaw_from_deg(float degrees) {
    return degrees * (kPi / 180.f);
}

std::string trim_copy(std::string text) {
    while (!text.empty() && (text.back() == '\r' || text.back() == ' ' || text.back() == '\t'))
        text.pop_back();
    std::size_t i = 0;
    if (text.size() >= 3 &&
        static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB &&
        static_cast<unsigned char>(text[2]) == 0xBF) {
        i = 3;
    }
    while (i < text.size() && (text[i] == ' ' || text[i] == '\t')) ++i;
    return text.substr(i);
}

std::string strip_comment(const std::string& text) {
    const auto hash = text.find('#');
    if (hash == std::string::npos) return text;
    return trim_copy(text.substr(0, hash));
}

}  // namespace

struct PromoProjectApplet::Impl {
    enum class Run { Idle, Projecting, Paused };

    struct Node {
        int id = 0;
        float east = 0.f;
        float north = 0.f;
        float up = 0.f;
        float inner = 0.f;
        float orient = 0.f;
        bool use = true;
    };

    struct Seg {
        float e1 = 0.f;
        float n1 = 0.f;
        float e2 = 0.f;
        float n2 = 0.f;
    };

    Host& host;
    GLFWwindow* window = nullptr;
    std::vector<std::string> ports;
    int port_idx = 0;
    bool tried_initial = false;
    char path_buf[520] = {};
    std::string banner;
    std::string last_error;
    std::string file_note;
    std::vector<Node> nodes;
    std::vector<Seg> lines;
    std::vector<ProjPathPoint> stream_pts;
    Run run = Run::Idle;
    float yaw_deg = 0.f;
    float frozen_yaw = 0.f;
    int duty_percent = 30;
    int rate_khz = 20;
    int dropped_fov = 0;
    int laser_on_count = 0;

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

    bool connected() const { return host.connected && host.connected(); }

    float live_yaw() const { return yaw_from_deg(yaw_deg); }

    float shown_yaw() const { return run == Run::Idle ? live_yaw() : frozen_yaw; }

    void note_cmd(const char* name, bool ok) {
        if (ok) last_error.clear();
        else last_error = std::string(name) + " failed";
    }

    bool load_file(const std::string& path) {
        std::ifstream in(path);
        if (!in) {
            last_error = "Cannot read " + path;
            return false;
        }
        std::vector<Node> loaded;
        std::vector<Seg> segs;
        bool lines_section = false;
        int line_no = 0;
        std::string raw;
        while (std::getline(in, raw)) {
            ++line_no;
            const std::string text = trim_copy(raw);
            if (text.empty()) continue;
            if (text == "# lines") {
                lines_section = true;
                continue;
            }
            if (text[0] == '#') continue;
            const std::string body = strip_comment(text);
            if (body.empty()) continue;
            std::istringstream row(body);
            if (!lines_section) {
                Node node;
                if (!(row >> node.id >> node.east >> node.north >> node.up >> node.inner >> node.orient)) {
                    last_error = "Bad node line " + std::to_string(line_no);
                    return false;
                }
                node.use = std::fabs(node.inner) > 1e-4f;
                loaded.push_back(node);
            } else {
                Seg seg;
                if (!(row >> seg.e1 >> seg.n1 >> seg.e2 >> seg.n2)) {
                    last_error = "Bad line segment " + std::to_string(line_no);
                    return false;
                }
                segs.push_back(seg);
            }
        }
        if (loaded.empty()) {
            last_error = "No nodes in " + path;
            return false;
        }
        nodes = std::move(loaded);
        lines = std::move(segs);
        std::snprintf(path_buf, sizeof(path_buf), "%s", path.c_str());
        file_note = std::to_string(nodes.size()) + " nodes, " +
                    std::to_string(lines.size()) + " lines, up " +
                    std::to_string(nodes.front().up) + " m";
        last_error.clear();
        banner = "Loaded " + path;
        dropped_fov = 0;
        laser_on_count = 0;
        return true;
    }

    void try_initial() {
        if (tried_initial) return;
        tried_initial = true;
        if (!host.initial_nodes.empty()) load_file(host.initial_nodes);
        if (host.initial_port.empty() || connected()) return;
        std::string err;
        if (host.connect && host.connect(host.initial_port, err))
            banner = "Connected " + host.initial_port;
        else
            last_error = err.empty() ? "Initial connect failed" : err;
    }

    void stop_device() {
        with_ctl([](ProjectorController* c) {
            if (c) (void)c->stop_path_stream();
        });
        stream_pts.clear();
        run = Run::Idle;
    }

    void draw_connection() {
        ImGui::BeginChild("connection", ImVec2(0, 52), ImGuiChildFlags_Border);
        if (ImGui::BeginCombo("Port",
                              ports.empty() ? "(none)"
                                            : ports[static_cast<std::size_t>(port_idx)].c_str())) {
            for (int i = 0; i < static_cast<int>(ports.size()); ++i) {
                const bool sel = i == port_idx;
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
            if (ImGui::Button("Disconnect")) {
                stop_device();
                if (host.disconnect) host.disconnect();
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
        ImGui::SameLine();
        ImGui::Text("Link: %s", connected()
                                    ? (host.port_name ? host.port_name().c_str() : "up")
                                    : "down");
        ImGui::EndChild();
    }

    void draw_status_line(bool have, const ProjectorStatus& status) {
        if (!have) {
            ImGui::TextDisabled("Projector status not in yet");
            return;
        }
        ImGui::Text("power %s   %s   duty %u%%   stream %s   %u kHz   fault %s",
                    proj_power_name(status.power_state),
                    status.projecting ? "projecting" : "stopped",
                    static_cast<unsigned>(status.laser_duty_cycle),
                    proj_stream_name(status.stream_state),
                    static_cast<unsigned>(status.points_frequency),
                    status.has_fault ? "YES" : "no");
    }

    int count_in_wedge(float yaw) const {
        int n = 0;
        for (const Node& node : nodes) {
            if (node.use && ccu::in_fov(node.east, node.north, yaw)) ++n;
        }
        return n;
    }

    void select_wedge() {
        const float yaw = live_yaw();
        for (Node& node : nodes)
            node.use = ccu::in_fov(node.east, node.north, yaw) && std::fabs(node.inner) > 1e-4f;
    }

    void aim_yaw() {
        double east = 0.0;
        double north = 0.0;
        int n = 0;
        for (const Node& node : nodes) {
            if (!node.use) continue;
            east += node.east;
            north += node.north;
            ++n;
        }
        if (n == 0) return;
        yaw_deg = ccu::bearing_from_north(static_cast<float>(east), static_cast<float>(north)) *
                  (180.f / kPi);
    }

    void begin_project(bool have_status) {
        if (!connected() || !have_status) {
            last_error = "Connect and wait for projector status";
            return;
        }
        const float yaw = live_yaw();
        std::vector<ccu::PathNode> draw;
        dropped_fov = 0;
        for (const Node& node : nodes) {
            if (!node.use) continue;
            if (!ccu::in_fov(node.east, node.north, yaw)) {
                ++dropped_fov;
                continue;
            }
            draw.push_back({node.east, node.north, node.up, node.inner, node.orient});
        }
        if (draw.empty()) {
            last_error = "No selected nodes inside the wedge";
            return;
        }
        const std::vector<ccu::DacPoint> dac = ccu::path_engine(draw, yaw);
        std::vector<ProjPathPoint> pts;
        pts.reserve(dac.size());
        laser_on_count = 0;
        for (const ccu::DacPoint& point : dac) {
            pts.push_back(ProjPathPoint{point.x, point.y, point.laser_on});
            if (point.laser_on) ++laser_on_count;
        }
        bool ok = false;
        with_ctl([&](ProjectorController* c) {
            if (!c) return;
            ok = c->start_path_stream(pts, static_cast<uint16_t>(rate_khz),
                                      static_cast<uint16_t>(duty_percent));
        });
        if (!ok) {
            last_error = "Could not start the stream";
            return;
        }
        stream_pts = std::move(pts);
        frozen_yaw = yaw;
        run = Run::Projecting;
        last_error.clear();
        banner = "Projecting " + std::to_string(draw.size()) + " nodes, " +
                 std::to_string(stream_pts.size()) + " points";
    }

    void pause_project() {
        bool ok = false;
        with_ctl([&](ProjectorController* c) {
            if (c) ok = c->stop();
        });
        note_cmd("Pause", ok);
        if (ok) run = Run::Paused;
    }

    void play_project() {
        if (stream_pts.empty()) return;
        bool ok = false;
        with_ctl([&](ProjectorController* c) {
            if (!c) return;
            ok = c->start_path_stream(stream_pts, static_cast<uint16_t>(rate_khz),
                                      static_cast<uint16_t>(duty_percent));
        });
        note_cmd("Play", ok);
        if (ok) run = Run::Projecting;
    }

    void draw_controls(bool have_status) {
        const bool busy = run != Run::Idle;
        if (busy) ImGui::BeginDisabled();
        ImGui::SetNextItemWidth(-70.f);
        ImGui::InputText("##path", path_buf, sizeof(path_buf));
        ImGui::SameLine();
        if (ImGui::Button("Browse") && host.browse_nodes) {
            std::string picked = path_buf;
            if (host.browse_nodes(picked))
                std::snprintf(path_buf, sizeof(path_buf), "%s", picked.c_str());
        }
        if (ImGui::Button("Load") && path_buf[0] != '\0') load_file(path_buf);
        ImGui::SameLine();
        ImGui::TextDisabled("%s", file_note.empty() ? "no file" : file_note.c_str());
        if (busy) ImGui::EndDisabled();

        ImGui::Text("Height %.3f m   wedge ±%.2f rad (%.1f°)   range %.3g m   corner %.2f m",
                    static_cast<double>(ccu::device_height_m()),
                    static_cast<double>(ccu::angle_max_rad()),
                    static_cast<double>(ccu::angle_max_rad() * 180.f / kPi),
                    static_cast<double>(ccu::d_max_m()),
                    static_cast<double>(ccu::corner_len_m()));
        ImGui::TextDisabled("Heading is clockwise from north. This link has no stage sensor.");

        if (busy) ImGui::BeginDisabled();
        ImGui::SetNextItemWidth(220.f);
        ImGui::SliderFloat("Yaw deg", &yaw_deg, 0.f, 360.f, "%.1f");
        ImGui::SameLine();
        if (ImGui::Button("Aim at selected")) aim_yaw();
        if (busy) ImGui::EndDisabled();
        else
            ImGui::Text("Yaw %.3f rad", static_cast<double>(live_yaw()));
        if (busy)
            ImGui::Text("Frozen yaw %.1f deg", static_cast<double>(frozen_yaw * 180.f / kPi));

        ImGui::SetNextItemWidth(220.f);
        ImGui::SliderInt("Duty %", &duty_percent, kDutyMin, kDutyMax);
        if (ImGui::IsItemDeactivatedAfterEdit() && connected()) {
            bool ok = false;
            with_ctl([&](ProjectorController* c) {
                if (c) ok = c->set_laser_duty_cycle(static_cast<uint16_t>(duty_percent));
            });
            note_cmd("Duty", ok);
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120.f);
        ImGui::SliderInt("kHz", &rate_khz, kRateMinKhz, kRateMaxKhz);
        duty_percent = std::clamp(duty_percent, kDutyMin, kDutyMax);
        rate_khz = std::clamp(rate_khz, kRateMinKhz, kRateMaxKhz);

        const int in_wedge = count_in_wedge(shown_yaw());
        if (run == Run::Idle) {
            if (ImGui::Button("Project", ImVec2(100, 32))) begin_project(have_status);
        } else if (run == Run::Projecting) {
            if (ImGui::Button("Pause", ImVec2(100, 32))) pause_project();
        } else if (ImGui::Button("Play", ImVec2(100, 32))) {
            play_project();
        }
        ImGui::SameLine();
        if (run == Run::Idle) ImGui::BeginDisabled();
        if (ImGui::Button("End", ImVec2(100, 32))) {
            stop_device();
            banner = "Projection ended";
        }
        if (run == Run::Idle) ImGui::EndDisabled();
        ImGui::SameLine();
        ImGui::Text("%d selected in wedge", in_wedge);
        if (dropped_fov > 0) {
            ImGui::SameLine();
            ImGui::TextDisabled("(%d out of FOV)", dropped_fov);
        }

        if (busy) ImGui::BeginDisabled();
        if (ImGui::Button("All corners")) {
            for (Node& node : nodes) node.use = std::fabs(node.inner) > 1e-4f;
        }
        ImGui::SameLine();
        if (ImGui::Button("In wedge")) select_wedge();
        ImGui::SameLine();
        if (ImGui::Button("None")) {
            for (Node& node : nodes) node.use = false;
        }
        if (busy) ImGui::EndDisabled();

        if (!banner.empty()) ImGui::TextWrapped("%s", banner.c_str());
        if (!last_error.empty())
            ImGui::TextColored(ImVec4(1.f, 0.45f, 0.35f, 1.f), "%s", last_error.c_str());
        if (run != Run::Idle && !stream_pts.empty()) {
            ImGui::Text("%zu path points, %d laser on", stream_pts.size(), laser_on_count);
        }
    }

    void draw_table() {
        const float yaw = shown_yaw();
        const float table_h = ImGui::GetContentRegionAvail().y;
        const ImGuiTableFlags flags = ImGuiTableFlags_RowBg | ImGuiTableFlags_ScrollY |
                                      ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable;
        if (!ImGui::BeginTable("nodes", 6, flags, ImVec2(0, table_h))) return;
        ImGui::TableSetupScrollFreeze(0, 1);
        ImGui::TableSetupColumn("Use", ImGuiTableColumnFlags_WidthFixed, 40.f);
        ImGui::TableSetupColumn("Id", ImGuiTableColumnFlags_WidthFixed, 52.f);
        ImGui::TableSetupColumn("East", ImGuiTableColumnFlags_WidthFixed, 72.f);
        ImGui::TableSetupColumn("North", ImGuiTableColumnFlags_WidthFixed, 72.f);
        ImGui::TableSetupColumn("Corner", ImGuiTableColumnFlags_WidthFixed, 64.f);
        ImGui::TableSetupColumn("Wedge", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableHeadersRow();
        const bool busy = run != Run::Idle;
        for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
            Node& node = nodes[static_cast<std::size_t>(i)];
            const bool inside = ccu::in_fov(node.east, node.north, yaw);
            ImGui::TableNextRow();
            ImGui::PushID(i);
            ImGui::TableNextColumn();
            if (busy) ImGui::BeginDisabled();
            ImGui::Checkbox("##use", &node.use);
            if (busy) ImGui::EndDisabled();
            ImGui::TableNextColumn();
            ImGui::Text("%d", node.id);
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", static_cast<double>(node.east));
            ImGui::TableNextColumn();
            ImGui::Text("%.3f", static_cast<double>(node.north));
            ImGui::TableNextColumn();
            ImGui::Text("%.0f°", static_cast<double>(node.inner * 180.f / kPi));
            ImGui::TableNextColumn();
            if (inside) ImGui::TextUnformatted("in");
            else ImGui::TextDisabled("out");
            ImGui::PopID();
        }
        ImGui::EndTable();
    }

    void draw_plan() {
        const ImVec2 avail = ImGui::GetContentRegionAvail();
        ImVec2 size = avail;
        if (size.x < 20.f) size.x = 20.f;
        if (size.y < 20.f) size.y = 20.f;
        ImGui::InvisibleButton("plan_canvas", size);
        const ImVec2 p0 = ImGui::GetItemRectMin();
        const ImVec2 p1 = ImGui::GetItemRectMax();
        ImDrawList* dl = ImGui::GetWindowDrawList();
        dl->AddRectFilled(p0, p1, IM_COL32(16, 18, 22, 255));
        dl->PushClipRect(p0, p1, true);

        float emin = 0.f, emax = 0.f, nmin = 0.f, nmax = 0.f;
        auto include = [&](float east, float north) {
            emin = std::min(emin, east);
            emax = std::max(emax, east);
            nmin = std::min(nmin, north);
            nmax = std::max(nmax, north);
        };
        for (const Node& node : nodes) include(node.east, node.north);
        for (const Seg& seg : lines) {
            include(seg.e1, seg.n1);
            include(seg.e2, seg.n2);
        }
        const float yaw = shown_yaw();
        const float half = ccu::angle_max_rad();
        const float reach = std::min(ccu::d_max_m(), std::max(2.f, std::max(emax - emin, nmax - nmin)));
        include(std::sin(yaw - half) * reach, std::cos(yaw - half) * reach);
        include(std::sin(yaw + half) * reach, std::cos(yaw + half) * reach);
        float span_e = std::max(0.5f, emax - emin);
        float span_n = std::max(0.5f, nmax - nmin);
        emin -= span_e * 0.08f;
        emax += span_e * 0.08f;
        nmin -= span_n * 0.08f;
        nmax += span_n * 0.08f;
        span_e = emax - emin;
        span_n = nmax - nmin;
        const float pad = 16.f;
        const float scale = std::min((size.x - 2.f * pad) / span_e, (size.y - 2.f * pad) / span_n);
        const float ox = p0.x + (size.x - span_e * scale) * 0.5f - emin * scale;
        const float oy = p0.y + (size.y - span_n * scale) * 0.5f + nmax * scale;
        auto screen = [&](float east, float north) {
            return ImVec2(ox + east * scale, oy - north * scale);
        };

        const ImVec2 origin = screen(0.f, 0.f);
        dl->PathLineTo(origin);
        for (int i = 0; i <= 28; ++i) {
            const float t = yaw - half + (2.f * half) * static_cast<float>(i) / 28.f;
            dl->PathLineTo(screen(std::sin(t) * ccu::d_max_m(), std::cos(t) * ccu::d_max_m()));
        }
        dl->PathFillConvex(IM_COL32(70, 130, 190, 48));
        dl->AddLine(origin, screen(std::sin(yaw - half) * reach, std::cos(yaw - half) * reach),
                    IM_COL32(120, 170, 210, 180), 1.5f);
        dl->AddLine(origin, screen(std::sin(yaw + half) * reach, std::cos(yaw + half) * reach),
                    IM_COL32(120, 170, 210, 180), 1.5f);

        for (const Seg& seg : lines) {
            dl->AddLine(screen(seg.e1, seg.n1), screen(seg.e2, seg.n2),
                        IM_COL32(90, 98, 110, 255), 1.f);
        }

        for (const Node& node : nodes) {
            if (!node.use || !ccu::in_fov(node.east, node.north, yaw)) continue;
            ccu::PathNode sample{node.east, node.north, node.up, node.inner, node.orient};
            const std::vector<ccu::CartPoint> arm = ccu::path_corner_points(sample);
            for (std::size_t i = 1; i < arm.size(); ++i) {
                dl->AddLine(screen(arm[i - 1].east_m, arm[i - 1].north_m),
                            screen(arm[i].east_m, arm[i].north_m),
                            IM_COL32(245, 200, 70, 255), 2.f);
            }
        }

        dl->AddLine(screen(-0.15f, 0.f), screen(0.15f, 0.f), IM_COL32(220, 220, 220, 255), 1.5f);
        dl->AddLine(screen(0.f, -0.15f), screen(0.f, 0.15f), IM_COL32(220, 220, 220, 255), 1.5f);

        for (const Node& node : nodes) {
            const ImVec2 at = screen(node.east, node.north);
            const bool inside = ccu::in_fov(node.east, node.north, yaw);
            ImU32 color = IM_COL32(120, 120, 120, 255);
            if (node.use && inside) color = IM_COL32(120, 220, 140, 255);
            else if (node.use) color = IM_COL32(220, 90, 80, 255);
            dl->AddCircleFilled(at, 4.f, color);
            char label[16];
            std::snprintf(label, sizeof(label), "%d", node.id);
            dl->AddText(ImVec2(at.x + 6.f, at.y - 7.f), IM_COL32(210, 210, 210, 255), label);
        }

        dl->PopClipRect();
        dl->AddText(ImVec2(p0.x + 8.f, p0.y + 8.f), IM_COL32(180, 180, 180, 255),
                    "North up. Yellow arms are laser on. Grey lines are the DXF.");
    }

    void draw(bool have_status, const ProjectorStatus& status) {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("Promo projection", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings);
        draw_connection();
        draw_status_line(have_status, status);
        ImGui::Separator();

        ImGui::BeginChild("controls", ImVec2(520, 0), ImGuiChildFlags_Border);
        draw_controls(have_status);
        ImGui::Separator();
        draw_table();
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("plan", ImVec2(0, 0), ImGuiChildFlags_Border);
        draw_plan();
        ImGui::EndChild();
        ImGui::End();
    }
};

PromoProjectApplet::PromoProjectApplet(Host host)
    : host_(std::move(host)), impl_(std::make_unique<Impl>(host_)) {}

PromoProjectApplet::~PromoProjectApplet() = default;

int PromoProjectApplet::run(std::atomic<bool>* keep_running) {
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

    impl_->window = glfwCreateWindow(1180, 760, "Promo projection", nullptr, nullptr);
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
    style.WindowRounding = 0.f;
    style.FrameRounding = 3.f;

    ImGui_ImplGlfw_InitForOpenGL(impl_->window, true);
    ImGui_ImplOpenGL3_Init(glsl);
    impl_->refresh_ports();
    impl_->try_initial();

    while (!glfwWindowShouldClose(impl_->window)) {
        if (keep_running && !keep_running->load())
            glfwSetWindowShouldClose(impl_->window, GLFW_TRUE);
        glfwPollEvents();
        impl_->with_ctl([](ProjectorController* c) {
            if (!c) return;
            (void)c->take_text();
            (void)c->request_status();
        });
        if (impl_->run == Impl::Run::Projecting) {
            impl_->with_ctl([](ProjectorController* c) {
                if (c) c->pump_stream();
            });
        }

        bool have = false;
        ProjectorStatus status{};
        impl_->with_ctl([&](ProjectorController* c) {
            if (!c) return;
            have = c->has_status();
            if (have) status = c->status();
        });

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        impl_->draw(have, status);
        ImGui::Render();

        int w = 0, h = 0;
        glfwGetFramebufferSize(impl_->window, &w, &h);
        glViewport(0, 0, w, h);
        glClearColor(0.08f, 0.08f, 0.10f, 1.f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(impl_->window);
    }

    impl_->stop_device();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(impl_->window);
    impl_->window = nullptr;
    glfwTerminate();
    return 0;
}
