#pragma once

#include <cstdint>

namespace ccu {

enum class WorkMode : uint8_t {
    Idle = 0,
    Surveying = 1,
    Projecting = 2,
};

// Session running, work mode, and whether origin is set.
// A matching session id is checked by the session switch, not here.
struct AcceptMode {
    bool running = false;
    WorkMode work = WorkMode::Idle;
    bool origin_is_set = false;
};

bool command_allowed(AcceptMode mode, uint8_t cmd);

}  // namespace ccu
