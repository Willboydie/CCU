#include "rx/serial/SerialDevice.hpp"
#include "rx/MessageConsumer.hpp"
#include "rx/msg_queue/MessageQueue.hpp"
#include "rx/parser/ProjectorParser.hpp"
#include "tx/ProjectorController.hpp"
#include "ui/PromoProjectApplet.hpp"
#include "device_config.hpp"
#include "utils/projector_protocol.hpp"

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#endif

namespace {

std::atomic<bool> g_run{true};

#ifdef _WIN32
BOOL WINAPI on_console_ctrl(DWORD) {
    g_run = false;
    return TRUE;
}
#endif

std::vector<std::string> list_com_ports() {
    std::vector<std::string> ports;
#ifdef _WIN32
    char target[256];
    for (int i = 1; i <= 256; ++i) {
        const std::string name = "COM" + std::to_string(i);
        if (QueryDosDeviceA(name.c_str(), target, static_cast<DWORD>(sizeof(target))) == 0)
            continue;
        if (i == 1) continue;
        ports.push_back(name);
    }
#else
    for (int i = 0; i < 32; ++i) {
        ports.push_back("/dev/ttyACM" + std::to_string(i));
        ports.push_back("/dev/ttyUSB" + std::to_string(i));
    }
#endif
    return ports;
}

bool browse_nodes(std::string& path) {
#ifdef _WIN32
    char buf[MAX_PATH] = {};
    if (!path.empty()) {
        const std::size_t n = std::min(path.size(), static_cast<std::size_t>(MAX_PATH - 1));
        std::memcpy(buf, path.data(), n);
    }
    OPENFILENAMEA ofn{};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "Node files\0*.txt;*.nodes\0All files\0*.*\0";
    ofn.lpstrFile = buf;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
    ofn.lpstrTitle = "Surveyed nodes";
    if (!GetOpenFileNameA(&ofn)) return false;
    path = buf;
    return true;
#else
    (void)path;
    return false;
#endif
}

std::string exe_directory() {
#ifdef _WIN32
    char buf[MAX_PATH];
    const DWORD n = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    if (n == 0 || n >= MAX_PATH) return {};
    std::string path(buf, n);
    const auto slash = path.find_last_of("\\/");
    if (slash == std::string::npos) return {};
    return path.substr(0, slash);
#else
    return {};
#endif
}

void load_survey_config() {
    ccu::DeviceConfigStatus status = ccu::device_config_load("ccu_survey.cfg");
    if (!status.opened) {
        const std::string dir = exe_directory();
        if (!dir.empty()) {
#ifdef _WIN32
            const std::string path = dir + "\\ccu_survey.cfg";
#else
            const std::string path = dir + "/ccu_survey.cfg";
#endif
            status = ccu::device_config_load(path.c_str());
            if (status.opened)
                std::printf("config %s\n", path.c_str());
        }
    } else {
        std::printf("config ccu_survey.cfg\n");
    }
    if (!status.opened)
        std::printf("ccu_survey.cfg missing; defaults in use\n");
    std::printf("DEVICE_HEIGHT %.3f m D_MAX %.3g ANGLE_MAX %.2f\n",
                static_cast<double>(ccu::device_height_m()),
                static_cast<double>(ccu::d_max_m()),
                static_cast<double>(ccu::angle_max_rad()));
}

struct Session {
    std::mutex mu;
    MessageQueue queue;
    MessageConsumer consumer;
    ProjectorParser parser;
    std::unique_ptr<SerialDevice> device;
    std::unique_ptr<ProjectorController> controller;
    std::string port;

    Session() : consumer(queue) {}
};

Session g_session;

void register_handlers() {
    g_session.consumer.set_handler(kProjStatusHeader, [](uint8_t, std::span<const uint8_t> payload) {
        ProjectorStatus pkt{};
        if (!unpack_projector_status(payload, pkt)) return;
        std::lock_guard<std::mutex> lock(g_session.mu);
        if (g_session.controller) g_session.controller->on_status(pkt);
    });
    g_session.consumer.set_handler(kProjDebugHeader, [](uint8_t, std::span<const uint8_t> payload) {
        std::lock_guard<std::mutex> lock(g_session.mu);
        if (g_session.controller) g_session.controller->on_text(kProjDebugHeader, payload);
    });
    auto on_notify = [](uint8_t cmd, std::span<const uint8_t>) {
        std::lock_guard<std::mutex> lock(g_session.mu);
        if (g_session.controller) g_session.controller->on_notify(cmd);
    };
    g_session.consumer.set_handler(kProjNotifyPauseStream, on_notify);
    g_session.consumer.set_handler(kProjNotifyResumeStream, on_notify);
    g_session.consumer.set_handler(kProjNotifyResetStream, on_notify);
}

void close_device() {
    std::lock_guard<std::mutex> lock(g_session.mu);
    g_session.controller.reset();
    if (g_session.device) g_session.device->close();
    g_session.device.reset();
    g_session.port.clear();
}

bool open_device(const std::string& port, std::string& err) {
    close_device();
    g_session.parser.reset();
    auto device = std::make_unique<SerialDevice>(port, kProjBaud, g_session.queue, &g_session.parser);
    if (!device->open()) {
        err = "Failed to open " + port;
        return false;
    }
    SerialDevice* raw = nullptr;
    {
        std::lock_guard<std::mutex> lock(g_session.mu);
        g_session.device = std::move(device);
        g_session.controller = std::make_unique<ProjectorController>(*g_session.device);
        g_session.port = port;
        raw = g_session.device.get();
    }
    raw->start();
    return true;
}

bool file_exists(const std::string& path) {
    std::ifstream in(path);
    return static_cast<bool>(in);
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleCtrlHandler(on_console_ctrl, TRUE);
#endif
    load_survey_config();
    register_handlers();
    g_session.consumer.start();

    std::string port;
    std::string nodes;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (file_exists(arg)) nodes = arg;
        else port = arg;
    }

    PromoProjectApplet::Host host;
    host.initial_port = port;
    host.initial_nodes = nodes;
    host.list_ports = list_com_ports;
    host.connect = open_device;
    host.disconnect = close_device;
    host.connected = [] {
        std::lock_guard<std::mutex> lock(g_session.mu);
        return static_cast<bool>(g_session.controller);
    };
    host.port_name = [] {
        std::lock_guard<std::mutex> lock(g_session.mu);
        return g_session.port;
    };
    host.with_controller = [](const std::function<void(ProjectorController*)>& fn) {
        std::lock_guard<std::mutex> lock(g_session.mu);
        fn(g_session.controller.get());
    };
    host.browse_nodes = browse_nodes;

    PromoProjectApplet applet(std::move(host));
    const int rc = applet.run(&g_run);
    close_device();
    g_session.consumer.stop();
    return rc;
}
