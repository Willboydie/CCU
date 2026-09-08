#include "ui/ProjectorEspApplet.hpp"

#include "utils/esp_now_protocol.hpp"
#include "utils/projector_protocol.hpp"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdio>
#include <deque>
#include <functional>
#include <string>
#include <vector>

namespace {

constexpr int kDutyMin = 15;
constexpr int kDutyMax = 99;
constexpr int kEspBauds[] = {
    9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600
};
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

int index_of_baud(int baud) {
    for (int i = 0; i < static_cast<int>(std::size(kEspBauds)); ++i) {
        if (kEspBauds[i] == baud)
            return i;
    }
    return 7;  // 921600
}

void format_age(double age, char* buf, std::size_t n) {
    if (age < 0.0) age = 0.0;
    if (age < 10.0)
        std::snprintf(buf, n, "%.1f s ago", age);
    else
        std::snprintf(buf, n, "%.0f s ago", age);
}

ImVec4 age_color(double age) {
    if (age < 2.0) return ImVec4(0.35f, 0.92f, 0.45f, 1);
    if (age < 10.0) return ImVec4(1.0f, 0.82f, 0.28f, 1);
    return ImVec4(0.65f, 0.65f, 0.68f, 1);
}

}  // namespace

struct ProjectorEspApplet::Impl {
    struct TagRecord {
        int slave = 0;
        std::string mac;
        int intensity = 0;
        int battery = 0;
        int rssi = 0;
        int sample = 0;
        std::string quality;
        std::string state;
        bool have_sample = false;
        bool have_rssi = false;
        bool timed_out = false;
        double sample_t = 0.0;
        double rssi_t = 0.0;
    };

    Host& host;
    GLFWwindow* window = nullptr;

    std::vector<std::string> ports;
    int proj_port_idx = 0;
    int esp_port_idx = 0;
    int esp_baud_idx = 7;
    bool tried_initial = false;
    std::string banner;
    std::string last_error;
    std::string esp_error;
    int increment = 256;
    int duty_percent = 50;

    std::deque<std::string> monitor;
    std::string partial;
    uint64_t esp_bytes = 0;
    bool last_was_cr = false;

    char target_buf[48] = "1";
    int rssi_slave = 1;
    bool expect_list = false;
    int ready_count = 0;
    int active_slave = 0;
    int view_slave = 0;
    std::string last_target;
    std::vector<TagRecord> tags;

    explicit Impl(Host& h) : host(h) {
        esp_baud_idx = index_of_baud(host.initial_esp_baud);
    }

    void refresh_ports() {
        const std::string proj_sel =
            ports.empty() ? host.initial_port
                          : ports[static_cast<std::size_t>(proj_port_idx)];
        const std::string esp_sel =
            ports.empty() ? host.initial_esp_port
                          : ports[static_cast<std::size_t>(esp_port_idx)];
        ports = host.list_ports ? host.list_ports() : std::vector<std::string>{};
        proj_port_idx = index_of_port(ports, proj_sel);
        esp_port_idx = index_of_port(ports, esp_sel);
        if (ports.empty()) {
            proj_port_idx = 0;
            esp_port_idx = 0;
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

    bool projector_on() const {
        return host.connected && host.connected();
    }

    bool esp_on() const {
        return host.esp_connected && host.esp_connected();
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

    void reset_network() {
        tags.clear();
        expect_list = false;
        ready_count = 0;
        active_slave = 0;
        view_slave = 0;
        last_target.clear();
    }

    TagRecord* find_tag(int slave) {
        for (auto& t : tags) {
            if (t.slave == slave) return &t;
        }
        return nullptr;
    }

    TagRecord& upsert_tag(int slave) {
        if (TagRecord* t = find_tag(slave)) return *t;
        TagRecord t;
        t.slave = slave;
        tags.push_back(std::move(t));
        std::sort(tags.begin(), tags.end(),
                  [](const TagRecord& a, const TagRecord& b) { return a.slave < b.slave; });
        return *find_tag(slave);
    }

    void set_all_listen_except(int selected) {
        for (auto& t : tags)
            t.state = (t.slave == selected && selected != 0) ? "SELECTED" : "LISTEN";
    }

    void apply_measurement(TagRecord& t, const EspLine& msg, double now) {
        if (msg.have_intensity) {
            t.intensity = msg.intensity;
            t.battery = msg.battery;
            t.sample = msg.sample;
            t.have_sample = true;
            t.timed_out = false;
            t.sample_t = now;
        }
        if (msg.have_rssi) {
            t.rssi = msg.rssi;
            t.have_rssi = true;
            t.rssi_t = now;
        }
        if (!msg.target.empty()) last_target = msg.target;
    }

    void apply_esp_line(const EspLine& msg) {
        const double now = glfwGetTime();
        switch (msg.kind) {
        case EspLineKind::Enrolled: {
            TagRecord& t = upsert_tag(msg.slave);
            if (!msg.mac.empty()) t.mac = msg.mac;
            if (t.state.empty()) t.state = "LISTEN";
            ready_count = static_cast<int>(tags.size());
            break;
        }
        case EspLineKind::Ready:
            ready_count = msg.count;
            if (expect_list) {
                tags.clear();
                expect_list = false;
            }
            break;
        case EspLineKind::Slave: {
            TagRecord& t = upsert_tag(msg.slave);
            if (!msg.mac.empty()) t.mac = msg.mac;
            apply_measurement(t, msg, now);
            if (!msg.quality.empty()) t.quality = msg.quality;
            if (!msg.state.empty()) t.state = msg.state;
            if (ieq(t.state, "SELECTED")) {
                active_slave = t.slave;
                view_slave = t.slave;
                rssi_slave = t.slave;
            }
            break;
        }
        case EspLineKind::Reading: {
            TagRecord& t = upsert_tag(msg.slave);
            apply_measurement(t, msg, now);
            break;
        }
        case EspLineKind::Active: {
            TagRecord& t = upsert_tag(msg.slave);
            apply_measurement(t, msg, now);
            t.state = "SELECTED";
            active_slave = msg.slave;
            view_slave = msg.slave;
            rssi_slave = msg.slave;
            set_all_listen_except(msg.slave);
            break;
        }
        case EspLineKind::SampleResult: {
            TagRecord& t = upsert_tag(msg.slave);
            apply_measurement(t, msg, now);
            active_slave = msg.slave;
            view_slave = msg.slave;
            t.state = "SELECTED";
            set_all_listen_except(msg.slave);
            break;
        }
        case EspLineKind::Rssi: {
            TagRecord& t = upsert_tag(msg.slave);
            if (msg.rssi_no_reply) {
                t.have_rssi = false;
                t.quality = msg.quality.empty() ? "POOR" : msg.quality;
                t.rssi_t = now;
            } else if (msg.have_rssi) {
                t.rssi = msg.rssi;
                t.have_rssi = true;
                t.rssi_t = now;
                if (!msg.quality.empty()) t.quality = msg.quality;
            }
            break;
        }
        case EspLineKind::Timeout: {
            TagRecord& t = upsert_tag(msg.slave);
            t.timed_out = true;
            t.sample = msg.sample;
            t.sample_t = now;
            if (!msg.target.empty()) last_target = msg.target;
            esp_error = "TIMEOUT slave " + std::to_string(msg.slave);
            break;
        }
        case EspLineKind::Deselected:
            active_slave = 0;
            set_all_listen_except(0);
            break;
        case EspLineKind::Error:
            esp_error = msg.error.empty() ? "ESP error" : msg.error;
            break;
        case EspLineKind::RssiDone:
        case EspLineKind::Unknown:
            break;
        }
    }

    void finish_line() {
        monitor.push_back(std::move(partial));
        apply_esp_line(parse_esp_line(monitor.back()));
        partial.clear();
        while (static_cast<int>(monitor.size()) > kMonitorMaxLines)
            monitor.pop_front();
    }

    void append_monitor(const std::string& chunk) {
        if (chunk.empty()) return;
        esp_bytes += chunk.size();
        for (unsigned char uc : chunk) {
            const char c = static_cast<char>(uc);
            if (c == '\0') continue;
            if (c == '\n') {
                if (last_was_cr) {
                    last_was_cr = false;
                    continue;
                }
                finish_line();
                continue;
            }
            if (c == '\r') {
                last_was_cr = true;
                finish_line();
                continue;
            }
            last_was_cr = false;
            partial.push_back(c);
        }
    }

    void drain_esp() {
        if (!host.take_esp_text) return;
        append_monitor(host.take_esp_text());
    }

    bool send_line(const std::string& line) {
        if (!host.send_esp) {
            esp_error = "ESP send not available";
            return false;
        }
        std::string wire = line;
        if (wire.empty() || wire.back() != '\n') wire.push_back('\n');
        if (!host.send_esp(wire)) {
            esp_error = "ESP send failed";
            return false;
        }
        esp_error.clear();
        monitor.push_back("> " + line);
        while (static_cast<int>(monitor.size()) > kMonitorMaxLines)
            monitor.pop_front();
        return true;
    }

    std::string current_target() const {
        return std::string(trim_sv(target_buf));
    }

    bool send_targeted(const char* cmd) {
        const std::string target = current_target();
        if (target.empty()) {
            esp_error = std::string(cmd) + " needs a target";
            return false;
        }
        return send_line(std::string(cmd) + "," + target);
    }

    void try_initial_connect() {
        if (tried_initial) return;
        tried_initial = true;
        if (!host.initial_port.empty() && !projector_on()) {
            std::string err;
            if (host.connect && host.connect(host.initial_port, err)) {
                banner = "Projector " + host.initial_port;
            } else {
                last_error = err.empty() ? "Projector connect failed" : err;
            }
        }
        if (!host.initial_esp_port.empty() && !esp_on()) {
            std::string err;
            const int baud = kEspBauds[static_cast<std::size_t>(esp_baud_idx)];
            if (host.connect_esp && host.connect_esp(host.initial_esp_port, baud, err)) {
                banner = banner.empty()
                    ? "ESP " + host.initial_esp_port
                    : banner + "  ESP " + host.initial_esp_port;
            } else {
                esp_error = err.empty() ? "ESP connect failed" : err;
            }
        }
    }

    void draw_port_combo(const char* id, int& idx) {
        const char* preview = ports.empty()
            ? "(none)"
            : ports[static_cast<std::size_t>(idx)].c_str();
        if (ImGui::BeginCombo(id, preview)) {
            for (int i = 0; i < static_cast<int>(ports.size()); ++i) {
                const bool sel = (i == idx);
                if (ImGui::Selectable(ports[static_cast<std::size_t>(i)].c_str(), sel))
                    idx = i;
                if (sel) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
    }

    void draw_connection() {
        ImGui::BeginChild("connection", ImVec2(0, 96), ImGuiChildFlags_Border);

        ImGui::SetNextItemWidth(140);
        draw_port_combo("##proj_port", proj_port_idx);
        ImGui::SameLine();
        ImGui::TextUnformatted("Projector");
        ImGui::SameLine();
        if (ImGui::Button("Refresh")) refresh_ports();
        ImGui::SameLine();
        if (projector_on()) {
            if (ImGui::Button("Disconnect##proj") && host.disconnect) {
                host.disconnect();
                banner = "Projector disconnected";
            }
        } else if (ImGui::Button("Connect##proj")) {
            if (ports.empty()) {
                last_error = "No serial ports";
            } else {
                std::string err;
                const std::string& port = ports[static_cast<std::size_t>(proj_port_idx)];
                if (host.connect && host.connect(port, err)) {
                    last_error.clear();
                    banner = "Projector " + port;
                } else {
                    last_error = err.empty() ? "Projector connect failed" : err;
                }
            }
        }
        ImGui::SameLine(420);
        ImGui::Text("Link: %s", projector_on()
            ? (host.port_name ? host.port_name().c_str() : "up")
            : "down");
        if (!last_error.empty()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "%s", last_error.c_str());
        }

        ImGui::SetNextItemWidth(140);
        draw_port_combo("##esp_port", esp_port_idx);
        ImGui::SameLine();
        ImGui::TextUnformatted("ESP");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(90);
        const bool baud_locked = esp_on();
        if (baud_locked) ImGui::BeginDisabled();
        char baud_buf[16];
        std::snprintf(baud_buf, sizeof(baud_buf), "%d",
                      kEspBauds[static_cast<std::size_t>(esp_baud_idx)]);
        if (ImGui::BeginCombo("##esp_baud", baud_buf)) {
            for (int i = 0; i < static_cast<int>(std::size(kEspBauds)); ++i) {
                char item[16];
                std::snprintf(item, sizeof(item), "%d", kEspBauds[i]);
                const bool sel = (i == esp_baud_idx);
                if (ImGui::Selectable(item, sel))
                    esp_baud_idx = i;
                if (sel) ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        if (baud_locked) ImGui::EndDisabled();
        ImGui::SameLine();
        if (esp_on()) {
            if (ImGui::Button("Disconnect##esp") && host.disconnect_esp) {
                host.disconnect_esp();
                reset_network();
                banner = "ESP disconnected";
            }
        } else if (ImGui::Button("Connect##esp")) {
            if (ports.empty()) {
                esp_error = "No serial ports";
            } else {
                std::string err;
                const std::string& port = ports[static_cast<std::size_t>(esp_port_idx)];
                const int baud = kEspBauds[static_cast<std::size_t>(esp_baud_idx)];
                if (host.connect_esp && host.connect_esp(port, baud, err)) {
                    reset_network();
                    esp_error.clear();
                    banner = "ESP " + port;
                } else {
                    esp_error = err.empty() ? "ESP connect failed" : err;
                }
            }
        }
        ImGui::SameLine(420);
        ImGui::Text("Link: %s", esp_on()
            ? (host.esp_port_name ? host.esp_port_name().c_str() : "up")
            : "down");
        if (!esp_error.empty()) {
            ImGui::SameLine();
            ImGui::TextColored(ImVec4(1, 0.4f, 0.35f, 1), "%s", esp_error.c_str());
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

    void draw_controls(bool on, bool have_status, bool pointing,
                       uint16_t x, uint16_t y) {
        ImGui::BeginChild("controls", ImVec2(0, 0), ImGuiChildFlags_Border);
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

        ImGui::Spacing();
        if (!can_duty) ImGui::BeginDisabled();
        ImGui::SetNextItemWidth(220);
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

    bool cmd_btn(const char* label, bool enabled, const std::function<void()>& fn) {
        if (!enabled) ImGui::BeginDisabled();
        const bool hit = ImGui::Button(label, ImVec2(0, 28));
        if (!enabled) ImGui::EndDisabled();
        if (hit) fn();
        return hit;
    }

    void draw_commands() {
        ImGui::BeginChild("esp_cmds", ImVec2(0, 86), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("Master commands");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(120);
        ImGui::InputText("Target", target_buf, sizeof(target_buf));
        ImGui::SetItemTooltip("Argument for SCAN, NEXT and SAMPLE  (SCAN,<target>)");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(70);
        ImGui::InputInt("Slave", &rssi_slave);
        if (rssi_slave < 1) rssi_slave = 1;

        const bool on = esp_on();
        cmd_btn("HELP", on, [&] { send_line("HELP"); });
        ImGui::SameLine();
        cmd_btn("LIST", on, [&] {
            expect_list = true;
            send_line("LIST");
        });
        ImGui::SameLine();
        cmd_btn("SCAN", on, [&] { send_targeted("SCAN"); });
        ImGui::SameLine();
        cmd_btn("NEXT", on, [&] { send_targeted("NEXT"); });
        ImGui::SameLine();
        cmd_btn("SAMPLE", on, [&] { send_targeted("SAMPLE"); });
        ImGui::SameLine();
        cmd_btn("RSSI", on, [&] { send_line("RSSI"); });
        ImGui::SameLine();
        {
            char rssi_n[24];
            std::snprintf(rssi_n, sizeof(rssi_n), "RSSI %d", rssi_slave);
            cmd_btn(rssi_n, on, [&] {
                send_line(std::string("RSSI ") + std::to_string(rssi_slave));
            });
        }
        ImGui::SameLine();
        cmd_btn("DESELECT", on, [&] { send_line("DESELECT"); });
        ImGui::EndChild();
    }

    void draw_tags() {
        ImGui::BeginChild("tags", ImVec2(0, 168), ImGuiChildFlags_Border);
        ImGui::Text("Enrolled tags  (%d ready)", ready_count);
        ImGui::Separator();
        const double now = glfwGetTime();
        if (ImGui::BeginTable("tag_table", 8,
                              ImGuiTableFlags_RowBg | ImGuiTableFlags_Borders |
                                  ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp,
                              ImVec2(0, 0))) {
            ImGui::TableSetupColumn("#", ImGuiTableColumnFlags_WidthFixed, 28);
            ImGui::TableSetupColumn("MAC");
            ImGui::TableSetupColumn("I");
            ImGui::TableSetupColumn("Batt");
            ImGui::TableSetupColumn("RSSI");
            ImGui::TableSetupColumn("Qual");
            ImGui::TableSetupColumn("State");
            ImGui::TableSetupColumn("Age");
            ImGui::TableHeadersRow();
            for (const TagRecord& t : tags) {
                ImGui::TableNextRow();
                ImGui::TableNextColumn();
                const bool sel = (t.slave == view_slave);
                char id[16];
                std::snprintf(id, sizeof(id), "%d", t.slave);
                if (ImGui::Selectable(id, sel, ImGuiSelectableFlags_SpanAllColumns)) {
                    view_slave = t.slave;
                    rssi_slave = t.slave;
                }
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(t.mac.empty() ? "—" : t.mac.c_str());
                ImGui::TableNextColumn();
                if (t.have_sample) ImGui::Text("%d", t.intensity);
                else ImGui::TextDisabled("—");
                ImGui::TableNextColumn();
                if (t.have_sample) ImGui::Text("%d", t.battery);
                else ImGui::TextDisabled("—");
                ImGui::TableNextColumn();
                if (t.have_rssi) ImGui::Text("%d", t.rssi);
                else ImGui::TextDisabled("—");
                ImGui::TableNextColumn();
                ImGui::TextUnformatted(t.quality.empty() ? "—" : t.quality.c_str());
                ImGui::TableNextColumn();
                if (t.slave == active_slave)
                    ImGui::TextColored(ImVec4(0.35f, 0.92f, 0.45f, 1), "SELECTED");
                else
                    ImGui::TextUnformatted(t.state.empty() ? "LISTEN" : t.state.c_str());
                ImGui::TableNextColumn();
                const double stamp = t.have_sample ? t.sample_t
                    : (t.have_rssi ? t.rssi_t : 0.0);
                if (stamp > 0.0) {
                    char age[24];
                    format_age(now - stamp, age, sizeof(age));
                    ImGui::TextColored(age_color(now - stamp), "%s", age);
                } else {
                    ImGui::TextDisabled("—");
                }
            }
            ImGui::EndTable();
        }
        if (tags.empty())
            ImGui::TextDisabled("No enrolled tags yet. Power tags and click LIST.");
        ImGui::EndChild();
    }

    void draw_selected() {
        ImGui::BeginChild("selected", ImVec2(0, 118), ImGuiChildFlags_Border);
        const TagRecord* t = find_tag(view_slave != 0 ? view_slave : active_slave);
        ImGui::TextUnformatted("Selected tag");
        ImGui::Separator();
        if (!t) {
            ImGui::TextDisabled("None. Run SCAN or click a row in the table.");
            ImGui::EndChild();
            return;
        }
        const double now = glfwGetTime();
        const bool master_sel = (t->slave == active_slave);
        ImGui::Text("Tag %d   %s   %s",
                    t->slave,
                    t->mac.empty() ? "—" : t->mac.c_str(),
                    master_sel ? "ACTIVE" : "viewing");
        if (!last_target.empty()) {
            ImGui::SameLine();
            ImGui::TextDisabled("target %s", last_target.c_str());
        }

        if (t->have_sample) {
            char age[24];
            format_age(now - t->sample_t, age, sizeof(age));
            ImGui::Text("Intensity  %d", t->intensity);
            ImGui::SameLine(220);
            ImGui::TextColored(age_color(now - t->sample_t), "%s", age);
            ImGui::SameLine(340);
            ImGui::TextDisabled("battery %d   sample %d", t->battery, t->sample);
        } else if (t->timed_out) {
            ImGui::TextColored(ImVec4(1, 0.45f, 0.3f, 1), "Intensity  TIMEOUT");
        } else {
            ImGui::TextDisabled("Intensity  —");
        }

        if (t->have_rssi) {
            char age[24];
            format_age(now - t->rssi_t, age, sizeof(age));
            ImGui::Text("RSSI       %d dBm", t->rssi);
            ImGui::SameLine(220);
            ImGui::TextColored(age_color(now - t->rssi_t), "%s", age);
            ImGui::SameLine(340);
            ImGui::TextUnformatted(t->quality.empty() ? "" : t->quality.c_str());
        } else {
            ImGui::TextDisabled("RSSI       —");
            if (!t->quality.empty()) {
                ImGui::SameLine(220);
                ImGui::TextUnformatted(t->quality.c_str());
            }
        }
        ImGui::EndChild();
    }

    void draw_monitor() {
        ImGui::BeginChild("monitor", ImVec2(0, 0), ImGuiChildFlags_Border);
        ImGui::TextUnformatted("ESP monitor");
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear")) {
            monitor.clear();
            partial.clear();
            esp_bytes = 0;
            last_was_cr = false;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("%llu bytes  %d lines",
                            static_cast<unsigned long long>(esp_bytes),
                            static_cast<int>(monitor.size()) + (partial.empty() ? 0 : 1));
        ImGui::Separator();
        ImGui::BeginChild("monitor_body");
        if (!esp_on() && monitor.empty() && partial.empty()) {
            ImGui::TextDisabled("Connect an ESP serial port to see incoming data.");
        }
        for (const auto& line : monitor) {
            if (line.starts_with("> "))
                ImGui::TextColored(ImVec4(0.55f, 0.82f, 1.0f, 1), "%s", line.c_str());
            else if (line.starts_with("ERROR") || line.starts_with("TIMEOUT"))
                ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.35f, 1), "%s", line.c_str());
            else if (line.starts_with("ACTIVE") || line.starts_with("ENROLLED") ||
                     line.starts_with("SAMPLE_RESULT"))
                ImGui::TextColored(ImVec4(0.45f, 0.92f, 0.55f, 1), "%s", line.c_str());
            else
                ImGui::TextUnformatted(line.c_str());
        }
        if (!partial.empty())
            ImGui::TextColored(ImVec4(0.75f, 0.75f, 0.78f, 1), "%s", partial.c_str());
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY() - 8)
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
        ImGui::EndChild();
    }

    void draw() {
        const ImGuiViewport* vp = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(vp->WorkPos);
        ImGui::SetNextWindowSize(vp->WorkSize);
        ImGui::Begin("Projector + ESP", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                         ImGuiWindowFlags_NoSavedSettings);

        draw_connection();

        bool on = false;
        bool have_status = false;
        bool pointing = false;
        uint16_t x = kProjDacCentre;
        uint16_t y = kProjDacCentre;
        ProjectorStatus status{};
        with_ctl([&](ProjectorController* c) {
            on = c != nullptr;
            if (!c) return;
            have_status = c->has_status();
            pointing = c->pointing();
            x = c->x();
            y = c->y();
            if (have_status) status = c->status();
        });

        const ImVec2 rest = ImGui::GetContentRegionAvail();
        ImGui::BeginChild("body", ImVec2(0, 0));
        ImGui::BeginChild("left", ImVec2(rest.x * 0.38f, 0));
        draw_status(have_status, status);
        draw_controls(on, have_status, pointing, x, y);
        ImGui::EndChild();
        ImGui::SameLine();
        ImGui::BeginChild("right");
        draw_commands();
        draw_tags();
        draw_selected();
        draw_monitor();
        ImGui::EndChild();
        ImGui::EndChild();

        ImGui::End();
    }
};

ProjectorEspApplet::ProjectorEspApplet(Host host)
    : host_(std::move(host))
    , impl_(std::make_unique<Impl>(host_)) {}

ProjectorEspApplet::~ProjectorEspApplet() = default;

int ProjectorEspApplet::run(std::atomic<bool>* keep_running) {
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

    impl_->window = glfwCreateWindow(1280, 800, "Projector + ESP Monitor", nullptr, nullptr);
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
        impl_->drain_esp();
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
