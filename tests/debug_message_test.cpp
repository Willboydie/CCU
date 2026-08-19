// Hardware integration test: receive 0xFF debug frames (HOST_PROTOCOL.md §7).
//
// Prerequisite: StatusPacket.debug_mode = 1 (default on; toggle with SET_DEBUG_MODE).
// Optional: pass COM port as argv[1] (e.g. debug_message_test COM5).

#include "rx/MessageConsumer.hpp"
#include "rx/msg_queue/MessageQueue.hpp"
#include "rx/serial/SerialDevice.hpp"
#include "tx/LidarController.hpp"
#include "utils/protocol.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
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

int g_fails = 0;

void check(bool cond, const char* what) {
    if (cond) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_fails;
}

struct DebugCapture {
    std::mutex mu;
    std::atomic<uint32_t> frame_count{0};
    std::string last_payload;
};

std::atomic<bool> g_got_status{false};
LidarController* g_controller = nullptr;
DebugCapture* g_capture = nullptr;

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

bool is_debug_ascii(uint8_t b) {
    return b <= 127;
}

bool payload_is_debug_ascii(std::span<const uint8_t> payload) {
    for (uint8_t b : payload) {
        if (!is_debug_ascii(b))
            return false;
    }
    return true;
}

void on_debug_frame(uint8_t cmd, std::span<const uint8_t> payload) {
    check(cmd == kDebugHeader, "debug handler cmd is 0xFF");
    check(payload.size() > 0, "debug payload is non-empty");
    check(payload.size() <= kDebugMaxLen, "debug payload within kDebugMaxLen");
    check(payload_is_debug_ascii(payload), "debug payload is ASCII 0-127");

    if (g_capture) {
        std::lock_guard<std::mutex> lock(g_capture->mu);
        g_capture->last_payload.assign(reinterpret_cast<const char*>(payload.data()),
                                       payload.size());
        g_capture->frame_count.fetch_add(1, std::memory_order_relaxed);
    }
    if (g_controller)
        g_controller->on_text(kDebugHeader, payload);
}

void register_handlers(MessageConsumer& consumer) {
    consumer.set_handler(kStatusHeader, [](uint8_t, std::span<const uint8_t> payload) {
        StatusPayload status{};
        if (!unpack_status(payload, status))
            return;
        g_got_status = true;
        if (g_controller)
            g_controller->on_status(status);
    });

    consumer.set_handler(kDebugHeader, [](uint8_t cmd, std::span<const uint8_t> payload) {
        on_debug_frame(cmd, payload);
    });
}

bool try_port(
    const std::string& port,
    MessageQueue& queue,
    std::unique_ptr<SerialDevice>& device,
    std::chrono::milliseconds probe_timeout) {
    auto candidate = std::make_unique<SerialDevice>(port, 460800, queue);
    if (!candidate->open())
        return false;

    std::printf("Opened %s, waiting for telemetry...\n", port.c_str());
    g_got_status = false;
    candidate->start();

    const auto deadline = std::chrono::steady_clock::now() + probe_timeout;
    while (!g_got_status && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (!g_got_status) {
        candidate->close();
        return false;
    }

    device = std::move(candidate);
    return true;
}

bool wait_for_debug(LidarController& controller, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        const std::vector<std::string> lines = controller.take_text();
        for (const std::string& line : lines) {
            if (line.starts_with("[debug] "))
                return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return false;
}

void run_test(const std::string& forced_port) {
    DebugCapture capture;
    g_capture = &capture;

    MessageQueue queue;
    MessageConsumer consumer(queue);
    register_handlers(consumer);
    consumer.start();

    std::unique_ptr<SerialDevice> device;
    const std::vector<std::string> ports =
        forced_port.empty() ? list_com_ports() : std::vector<std::string>{forced_port};

    std::string connected_port;
    for (const auto& port : ports) {
        if (try_port(port, queue, device, std::chrono::seconds(8))) {
            connected_port = port;
            break;
        }
    }

    check(static_cast<bool>(device), "LiDAR serial device found");
    if (!device) {
        consumer.stop();
        g_capture = nullptr;
        return;
    }

    std::printf("Connected on %s\n", connected_port.c_str());

    LidarController controller(*device);
    g_controller = &controller;

    check(controller.wait_for_status([](const StatusPayload&) { return true; },
                                     std::chrono::seconds(2)),
          "received status packet");

    const StatusPayload status = controller.status();
    check(status.debug_mode != 0,
          "firmware debug_mode=1 (SET_DEBUG_MODE / default on per HOST_PROTOCOL §3.1)");

    controller.take_text();  // discard boot-time debug lines

    // HOST_PROTOCOL.md §6: out-of-range P2P targets are rejected (not applied)
    // and emit a debug message when debug mode is on.
    check(controller.set_point_to_point_target(0.0f, 0.0f),
          "SET_POINT2POINT_TARGET sent for out-of-bounds target");

    check(wait_for_debug(controller, std::chrono::seconds(5)),
          "debug message received through serial pipeline");

    check(capture.frame_count.load(std::memory_order_relaxed) >= 1,
          "MessageConsumer saw at least one 0xFF frame");

    std::string payload;
    {
        std::lock_guard<std::mutex> lock(capture.mu);
        payload = capture.last_payload;
    }
    check(!payload.empty(), "captured debug payload is non-empty");
    check(payload.find("outwith bounds") != std::string::npos,
          "debug text matches expected rejection (Point2Point target outwith bounds.)");

    std::printf("debug payload: \"%s\"\n", payload.c_str());

    g_controller = nullptr;
    g_capture = nullptr;
    device->close();
    consumer.stop();
}

}  // namespace

int main(int argc, char** argv) {
    const std::string forced_port = (argc > 1) ? argv[1] : "";
    run_test(forced_port);

    if (g_fails) {
        std::fprintf(stderr, "%d check(s) failed\n", g_fails);
        return 1;
    }
    std::printf("debug message test passed\n");
    return 0;
}
