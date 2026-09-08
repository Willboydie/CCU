#include "rx/serial/SerialDevice.hpp"
#include "rx/serial/SerialTextPort.hpp"
#include "rx/MessageConsumer.hpp"
#include "rx/msg_queue/MessageQueue.hpp"
#include "rx/parser/ProjectorParser.hpp"
#include "tx/ProjectorController.hpp"
#include "ui/ProjectorEspApplet.hpp"
#include "utils/projector_protocol.hpp"

#include <atomic>
#include <cstdint>
#include <cstdlib>
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
        if (i == 1)
            continue;
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

struct ProjSession {
    std::mutex mu;
    MessageQueue queue;
    MessageConsumer consumer;
    ProjectorParser parser;
    std::unique_ptr<SerialDevice> device;
    std::unique_ptr<ProjectorController> controller;
    std::string port;

    ProjSession() : consumer(queue) {}
};

struct EspSession {
    std::mutex mu;
    std::unique_ptr<SerialTextPort> device;
    std::string port;
};

ProjSession g_proj;
EspSession g_esp;

void register_handlers() {
    g_proj.consumer.set_handler(kProjStatusHeader, [](uint8_t, std::span<const uint8_t> payload) {
        ProjectorStatus pkt{};
        if (!unpack_projector_status(payload, pkt)) return;
        std::lock_guard<std::mutex> lock(g_proj.mu);
        if (g_proj.controller)
            g_proj.controller->on_status(pkt);
    });

    g_proj.consumer.set_handler(kProjDebugHeader, [](uint8_t, std::span<const uint8_t> payload) {
        std::lock_guard<std::mutex> lock(g_proj.mu);
        if (g_proj.controller)
            g_proj.controller->on_text(kProjDebugHeader, payload);
    });

    auto on_notify = [](uint8_t cmd, std::span<const uint8_t>) {
        std::lock_guard<std::mutex> lock(g_proj.mu);
        if (g_proj.controller)
            g_proj.controller->on_notify(cmd);
    };
    g_proj.consumer.set_handler(kProjNotifyPauseStream, on_notify);
    g_proj.consumer.set_handler(kProjNotifyResumeStream, on_notify);
    g_proj.consumer.set_handler(kProjNotifyResetStream, on_notify);
}

void close_projector() {
    std::lock_guard<std::mutex> lock(g_proj.mu);
    g_proj.controller.reset();
    if (g_proj.device)
        g_proj.device->close();
    g_proj.device.reset();
    g_proj.port.clear();
}

void close_esp() {
    std::lock_guard<std::mutex> lock(g_esp.mu);
    if (g_esp.device)
        g_esp.device->close();
    g_esp.device.reset();
    g_esp.port.clear();
}

bool projector_port_in_use(const std::string& port) {
    std::lock_guard<std::mutex> lock(g_proj.mu);
    return g_proj.controller && g_proj.port == port;
}

bool esp_port_in_use(const std::string& port) {
    std::lock_guard<std::mutex> lock(g_esp.mu);
    return static_cast<bool>(g_esp.device) && g_esp.port == port;
}

bool open_projector(const std::string& port, std::string& err) {
    if (esp_port_in_use(port)) {
        err = "Port in use by ESP";
        return false;
    }
    close_projector();

    g_proj.parser.reset();
    auto device = std::make_unique<SerialDevice>(
        port, kProjBaud, g_proj.queue, &g_proj.parser);
    if (!device->open()) {
        err = "Failed to open " + port;
        return false;
    }

    SerialDevice* raw = nullptr;
    {
        std::lock_guard<std::mutex> lock(g_proj.mu);
        g_proj.device = std::move(device);
        g_proj.controller = std::make_unique<ProjectorController>(*g_proj.device);
        g_proj.port = port;
        raw = g_proj.device.get();
    }
    raw->start();
    return true;
}

bool open_esp(const std::string& port, int baud, std::string& err) {
    if (projector_port_in_use(port)) {
        err = "Port in use by projector";
        return false;
    }
    close_esp();

    auto device = std::make_unique<SerialTextPort>();
    if (!device->open(port, baud)) {
        err = "Failed to open " + port;
        return false;
    }

    std::lock_guard<std::mutex> lock(g_esp.mu);
    g_esp.device = std::move(device);
    g_esp.port = port;
    return true;
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleCtrlHandler(on_console_ctrl, TRUE);
#endif

    register_handlers();
    g_proj.consumer.start();

    ProjectorEspApplet::Host host;
    host.initial_port = (argc > 1) ? argv[1] : "";
    host.initial_esp_port = (argc > 2) ? argv[2] : "";
    if (argc > 3)
        host.initial_esp_baud = std::atoi(argv[3]);
    host.list_ports = list_com_ports;
    host.connect = open_projector;
    host.disconnect = close_projector;
    host.connected = [] {
        std::lock_guard<std::mutex> lock(g_proj.mu);
        return static_cast<bool>(g_proj.controller);
    };
    host.port_name = [] {
        std::lock_guard<std::mutex> lock(g_proj.mu);
        return g_proj.port;
    };
    host.with_controller = [](const std::function<void(ProjectorController*)>& fn) {
        std::lock_guard<std::mutex> lock(g_proj.mu);
        fn(g_proj.controller.get());
    };
    host.connect_esp = open_esp;
    host.disconnect_esp = close_esp;
    host.esp_connected = [] {
        std::lock_guard<std::mutex> lock(g_esp.mu);
        return static_cast<bool>(g_esp.device) && g_esp.device->is_open();
    };
    host.esp_port_name = [] {
        std::lock_guard<std::mutex> lock(g_esp.mu);
        return g_esp.port;
    };
    host.take_esp_text = [] {
        std::lock_guard<std::mutex> lock(g_esp.mu);
        if (!g_esp.device) return std::string{};
        return g_esp.device->take();
    };
    host.send_esp = [](const std::string& text) {
        std::lock_guard<std::mutex> lock(g_esp.mu);
        if (!g_esp.device) return false;
        return g_esp.device->write(text);
    };

    ProjectorEspApplet applet(std::move(host));
    const int rc = applet.run(&g_run);

    close_projector();
    close_esp();
    g_proj.consumer.stop();
    return rc;
}
