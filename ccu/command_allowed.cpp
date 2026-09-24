#include "command_allowed.hpp"

#include "protocol.hpp"

namespace ccu {

bool command_allowed(AcceptMode mode, uint8_t cmd) {
    const bool idle_session = !mode.running;
    const bool work_idle = mode.running && mode.work == WorkMode::Idle;
    const bool surveying = mode.running && mode.work == WorkMode::Surveying;
    const bool projecting = mode.running && mode.work == WorkMode::Projecting;

    switch (cmd) {
    case kCmdBeginNewSession:
    case kCmdJoinSession:
        return idle_session;
    case kCmdConnected:
    case kCmdEndSession:
    case kCmdSetLaserPower:
        return mode.running;
    case kCmdBeginOriginSet:
        return work_idle && !mode.origin_is_set;
    case kCmdPerformSurvey:
    case kCmdPerformProjection:
        return work_idle && mode.origin_is_set;
    case kCmdAbortSurvey:
        return surveying;
    case kCmdProjectorPause:
    case kCmdProjectorPlay:
    case kCmdProjectionEnd:
    case kCmdNudgeX:
    case kCmdNudgeY:
        return projecting;
    case kCmdBeginTagFinding:
    case kCmdTagAligned:
        return false;
    default:
        return false;
    }
}

}  // namespace ccu
