#include "session.hpp"

#include "angle.hpp"
#include "dongle.hpp"
#include "dongle_link.hpp"
#include "project.hpp"
#include "projector.hpp"
#include "survey.hpp"
#include "telemetry.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

namespace ccu {
namespace {

std::chrono::steady_clock::time_point last_telemetry_{};
int telemetry_count_ = 0;
bool announced_down_ = false;

void send_pairing_if_due() {
    if (!session_pairing_unsent()) return;
    const auto bytes = pack_pairing_accepted("sm-ccu");
    if (!bytes) return;
    if (!dongle_link_write(bytes->data(), static_cast<int>(bytes->size()))) return;
    session_mark_pairing_sent();
    std::printf("pairing_accepted\n");
    std::fflush(stdout);
}

void send_telemetry_if_due() {
    const auto now = std::chrono::steady_clock::now();
    if (last_telemetry_.time_since_epoch().count() != 0 &&
        now - last_telemetry_ < std::chrono::milliseconds(100)) {
        return;
    }
    const RegularTelemetry msg = build_regular_telemetry();
    const auto bytes = pack_regular_telemetry(msg);
    if (!bytes) return;
    if (!dongle_link_write(bytes->data(), static_cast<int>(bytes->size()))) return;
    last_telemetry_ = now;
    ++telemetry_count_;
    if (telemetry_count_ <= 2 || telemetry_count_ % 10 == 0) {
        std::printf(
            "telemetry n=%d origin=%d moved=%d pose=0,0,0,0,0,0 yaw=%.3f\n",
            telemetry_count_, msg.origin_is_set ? 1 : 0, msg.moved ? 1 : 0,
            static_cast<double>(msg.effective_yaw));
        std::fflush(stdout);
    }
}

}  // namespace

void session_poll() {
    dongle_poll();
    angle_poll();
    if (!session_running()) {
        dongle_send_device_id_if_due();
        last_telemetry_ = {};
        telemetry_count_ = 0;
        announced_down_ = false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        return;
    }

    if (!session_link_up()) {
        if (!announced_down_) {
            std::printf("link down origin=%d yaw=%.3f\n",
                        session_origin_is_set() ? 1 : 0,
                        static_cast<double>(session_effective_yaw()));
            std::fflush(stdout);
            announced_down_ = true;
            last_telemetry_ = {};
            telemetry_count_ = 0;
        }
        dongle_send_device_id_if_due();
    } else {
        announced_down_ = false;
        send_pairing_if_due();
        send_telemetry_if_due();
    }

    switch (session_work()) {
    case WorkMode::Surveying:
        survey_poll();
        break;
    case WorkMode::Projecting:
        project_poll();
        projector_poll();
        break;
    case WorkMode::Idle:
        break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

}  // namespace ccu
