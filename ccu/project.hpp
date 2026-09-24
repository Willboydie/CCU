#pragma once

#include "path_engine.hpp"
#include "protocol.hpp"

#include <vector>

namespace ccu {

ProjectionReady project_begin(const std::vector<PlanNode>& nodes);
const std::vector<PathPoint>& project_path();
void project_poll();
void project_pause();
void project_play();
void project_end();

}  // namespace ccu
