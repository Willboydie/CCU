// H2.2. Injected dongle bytes, no serial port.
// "ready" clears paired. "paired" sets it. A tablet frame before paired is
// dropped. DEVICE_ID is withheld until paired, then the bytes match pack_device_id.

#include "dongle_link.hpp"
#include "protocol.hpp"
#include "session.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <vector>

namespace {

int g_fails = 0;

void check(bool cond, const char* what) {
    if (cond) return;
    std::fprintf(stderr, "FAIL: %s\n", what);
    ++g_fails;
}

struct Capture {
    std::vector<uint8_t> tx;
};

int capture_write(void* ctx, const uint8_t* data, int n) {
    auto* cap = static_cast<Capture*>(ctx);
    cap->tx.insert(cap->tx.end(), data, data + n);
    return n;
}

std::vector<uint8_t> status_frame(const char* text) {
    const auto n = static_cast<uint8_t>(std::strlen(text));
    std::vector<uint8_t> frame{0xE6, n, 0x00};
    frame.insert(frame.end(), text, text + n);
    return frame;
}

bool same(const std::vector<uint8_t>& got, const std::vector<uint8_t>& want) {
    return got.size() == want.size() &&
           (got.empty() || std::memcmp(got.data(), want.data(), got.size()) == 0);
}

}  // namespace

int main() {
    ccu::session_reset();

    Capture cap;
    ccu::dongle_link_attach(capture_write, &cap);

    const std::vector<uint8_t> ready = status_frame("ready");
    const std::vector<uint8_t> paired = status_frame("paired");
    const uint8_t begin[] = {ccu::kCmdBeginNewSession, 0x00, 0x00};

    ccu::dongle_link_feed(std::span<const uint8_t>(ready.data(), ready.size()));
    check(!ccu::dongle_link_paired(), "ready leaves unpaired");

    ccu::dongle_link_feed(std::span<const uint8_t>(begin, 3));
    check(!ccu::session_running(), "tablet frame before paired is dropped");

    ccu::dongle_send_device_id_if_due();
    check(cap.tx.empty(), "DEVICE_ID withheld before paired");

    ccu::dongle_link_feed(std::span<const uint8_t>(paired.data(), paired.size()));
    check(ccu::dongle_link_paired(), "paired sets the link");

    ccu::dongle_send_device_id_if_due();
    ccu::DeviceId idle;
    idle.device_id = "sm-ccu";
    idle.display_name = "Smart Marker";
    idle.session_active = false;
    const auto expect = ccu::pack_device_id(idle);
    check(expect.has_value(), "pack_device_id");
    check(expect && same(cap.tx, *expect), "DEVICE_ID matches pack_device_id");
    check(!ccu::session_running(), "session still idle");

    cap.tx.clear();
    ccu::dongle_link_feed(std::span<const uint8_t>(ready.data(), ready.size()));
    check(!ccu::dongle_link_paired(), "ready clears paired");
    ccu::dongle_send_device_id_if_due();
    check(cap.tx.empty(), "DEVICE_ID withheld after ready");

    if (g_fails) {
        std::printf("H2.2 FAIL\n");
        std::fflush(stdout);
        return 1;
    }
    std::printf("H2.2 PASS\n");
    std::fflush(stdout);
    return 0;
}
