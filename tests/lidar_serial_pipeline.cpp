#include "rx/serial/SerialDevice.hpp"
#include "rx/MessageConsumer.hpp"
#include "rx/msg_queue/MessageQueue.hpp"
#include "tx/LidarController.hpp"
#include "utils/protocol.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <span>
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

std::atomic<bool> g_run{true};
std::atomic<bool> g_got_packet{false};
std::atomic<LidarController*> g_controller{nullptr};
std::mutex g_print_mu;

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
    // USB CDC typically lands on ttyACM*; USB-UART on ttyUSB*.
    for (int i = 0; i < 32; ++i) {
        ports.push_back("/dev/ttyACM" + std::to_string(i));
        ports.push_back("/dev/ttyUSB" + std::to_string(i));
    }
#endif
    return ports;
}

void print_control(uint64_t n, const ControlPayload& c) {
    std::lock_guard<std::mutex> lock(g_print_mu);
    std::cout << std::fixed << std::setprecision(3)
              << "[control #" << n << "] dist=" << c.distance << " cm"
              << "  assoc=(" << c.associated_angle_x << ", " << c.associated_angle_y << ")"
              << "  mech=(" << c.mechanical_angle_x << ", " << c.mechanical_angle_y << ")"
              << "  tgt=(" << c.target_angle_x << ", " << c.target_angle_y << ")\n";
}

void print_status(const StatusPayload& s) {
    std::lock_guard<std::mutex> lock(g_print_mu);
    std::cout << std::fixed << std::setprecision(3)
              << "[status] enabled=" << static_cast<int>(s.enabled)
              << "  scan=" << scan_mode_name(s.scan_mode)
              << "  routine=" << dynamic_routine_name(s.dynamic_routine)
              << "  debug=" << static_cast<int>(s.debug_mode)
              << "  fault=" << static_cast<int>(s.has_fault)
              << "  centre=(" << s.coarse_centre_x << ", " << s.coarse_centre_y << ")"
              << "  p2p=(" << s.p2p_target_x << ", " << s.p2p_target_y << ")\n";
}

void print_text(const char* kind, std::span<const uint8_t> payload) {
    std::lock_guard<std::mutex> lock(g_print_mu);
    std::cout << '[' << kind << "] ";
    std::cout.write(reinterpret_cast<const char*>(payload.data()),
                    static_cast<std::streamsize>(payload.size()));
    std::cout << '\n';
}

void register_handlers(MessageConsumer& consumer) {
    consumer.set_handler(kControlHeader, [](uint8_t, std::span<const uint8_t> payload) {
        ControlPayload pkt{};
        if (!unpack_control(payload, pkt)) return;
        g_got_packet = true;
        if (auto* c = g_controller.load(std::memory_order_acquire))
            c->on_control(pkt);
        static std::atomic<uint64_t> n{0};
        const uint64_t count = ++n;
        // 1 kHz stream: print ~10 Hz so the console keeps up.
        if (count == 1 || count % 100 == 0)
            print_control(count, pkt);
    });

    consumer.set_handler(kStatusHeader, [](uint8_t, std::span<const uint8_t> payload) {
        StatusPayload pkt{};
        if (!unpack_status(payload, pkt)) return;
        g_got_packet = true;
        if (auto* c = g_controller.load(std::memory_order_acquire))
            c->on_status(pkt);
        print_status(pkt);
    });

    consumer.set_handler(kDebugHeader, [](uint8_t, std::span<const uint8_t> payload) {
        if (auto* c = g_controller.load(std::memory_order_acquire))
            c->on_text(kDebugHeader, payload);
        print_text("debug", payload);
    });
}

bool try_port(
    const std::string& port,
    MessageQueue& queue,
    std::unique_ptr<SerialDevice>& device,
    std::chrono::milliseconds probe_timeout) {
    auto candidate = std::make_unique<SerialDevice>(port, 460800, queue);
    if (!candidate->open()) return false;

    std::cout << "Opened " << port << ", waiting for telemetry...\n" << std::flush;
    g_got_packet = false;
    candidate->start();

    const auto deadline = std::chrono::steady_clock::now() + probe_timeout;
    while (g_run && !g_got_packet && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!g_got_packet) {
        candidate->close();
        return false;
    }

    device = std::move(candidate);
    return true;
}

}  // namespace

int main(int argc, char** argv) {
#ifdef _WIN32
    SetConsoleCtrlHandler(on_console_ctrl, TRUE);
#endif

    const std::string forced_port = (argc > 1) ? argv[1] : "";

    MessageQueue queue;
    MessageConsumer consumer(queue);
    register_handlers(consumer);
    consumer.start();

    std::unique_ptr<SerialDevice> device;
    std::unique_ptr<LidarController> controller;
    std::cout << "Waiting for LiDAR device";
    if (!forced_port.empty())
        std::cout << " on " << forced_port;
    std::cout << " (Ctrl+C to quit)...\n" << std::flush;

    while (g_run && !device) {
        const std::vector<std::string> ports =
            forced_port.empty() ? list_com_ports() : std::vector<std::string>{forced_port};

        for (const auto& port : ports) {
            if (!g_run) break;
            if (try_port(port, queue, device, std::chrono::milliseconds(8000))) {
                std::cout << "Listening on " << port << '\n' << std::flush;
                controller = std::make_unique<LidarController>(*device);
                g_controller.store(controller.get(), std::memory_order_release);
                std::cout << "Command path ready\n" << std::flush;
                break;
            }
        }

        if (!device && g_run)
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    while (g_run)
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

    g_controller.store(nullptr, std::memory_order_release);
    consumer.stop();
    controller.reset();
    if (device)
        device->close();
    return 0;
}
