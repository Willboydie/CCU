#include "telemetry.hpp"

#include "device_config.hpp"
#include "session.hpp"

namespace ccu {
namespace {

constexpr const char* kDeviceId = "sm-ccu";

}  // namespace

RegularTelemetry build_regular_telemetry() {
    RegularTelemetry msg;
    msg.device_id = kDeviceId;
    msg.session_id = session_id();
    msg.origin_is_set = session_origin_is_set();
    msg.pose.up_m = device_height_m();
    msg.effective_yaw = session_effective_yaw();
    msg.moved = false;
    msg.origin_set_status = OriginSetStatus::Idle;
    msg.tag_finding_status = TagFindingStatus::PoseKnown;
    msg.laser_on = false;
    msg.laser_power = kLaserDutyDefault;

    switch (session_work()) {
    case WorkMode::Surveying:
        msg.survey_status = SurveyStatus::Scanning;
        break;
    case WorkMode::Projecting:
        msg.projector_status = ProjectorStatus::Projecting;
        msg.projector_running = true;
        break;
    case WorkMode::Idle:
        break;
    }
    return msg;
}

}  // namespace ccu
