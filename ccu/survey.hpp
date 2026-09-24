#pragma once

#include "lidar.hpp"
#include "protocol.hpp"

#include <optional>
#include <vector>

namespace ccu {

using RangeRead = std::optional<float> (*)();

void survey_begin(const std::vector<PlanNode>& nodes, RangeRead read = lidar_read_range);
std::optional<SurveyResult> survey_poll();
std::vector<SurveyResult> survey_abort();
std::optional<SurveyResult> survey_result(uint16_t id);
void survey_reset();

}  // namespace ccu
