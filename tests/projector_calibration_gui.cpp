#include "rx/serial/SerialDevice.hpp"
#include "rx/MessageConsumer.hpp"
#include "rx/msg_queue/MessageQueue.hpp"
#include "rx/parser/ProjectorParser.hpp"
#include "tx/ProjectorController.hpp"
#include "ui/ProjectorCalibrationApplet.hpp"
#include "utils/projector_protocol.hpp"

#include <atomic>
#include <cstdint>
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
        if (g_session.controller)
            g_session.controller->on_status(pkt);
    });

    g_session.consumer.set_handler(kProjDebugHeader, [](uint8_t, std::span<const uint8_t> payload) {
        std::lock_guard<std::mutex> lock(g_session.mu);
        if (g_session.controller)
            g_session.controller->on_text(kProjDebugHeader, payload);
    });

    auto on_notify = [](uint8_t cmd, std::span<const uint8_t>) {
        std::lock_guard<std::mutex> lock(g_session.mu);
        if (g_session.controller)
            g_session.controller->on_notify(cmd);
    };
    g_session.consumer.set_handler(kProjNotifyPauseStream, on_notify);
    g_session.consumer.set_handler(kProjNotifyResumeStream, on_notify);
    g_session.consumer.set_handler(kProjNotifyResetStream, on_notify);
}

void close_device() {
    std::lock_guard<std::mutex> lock(g_session.mu);
    g_session.controller.reset();
    if (g_session.device)
        g_session.device->close();
    g_session.device.reset();
    g_session.port.clear();
}

bool open_device(const std::string& port, std::string& err) {
    close_device();

    g_session.parser.reset();
    auto device = std::make_unique<SerialDevice>(
        port, kProjBaud, g_session.queue, &g_session.parser);
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

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleCtrlHandler(on_console_ctrl, TRUE);
#endif

    register_handlers();
    g_session.consumer.start();

    ProjectorCalibrationApplet::Host host;
    host.initial_port = (argc > 1) ? argv[1] : "";
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

    ProjectorCalibrationApplet applet(std::move(host));
    const int rc = applet.run(&g_run);

    close_device();
    g_session.consumer.stop();
    return rc;
}
