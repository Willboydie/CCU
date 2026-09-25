#pragma once

#include "lidar.hpp"
#include "protocol.hpp"

#include <optional>
#include <vector>

namespace ccu {

struct SurveyNode {
    uint16_t id = 0;
    float east_m = 0.f;
    float north_m = 0.f;
    float up_m = 0.f;
    float corner_inner_angle_rad = 0.f;
    float corner_orientation_rad = 0.f;
    float normal_east = 0.f;
    float normal_north = 0.f;
    float normal_up = 1.f;
};

void survey_begin(const std::vector<PlanNode>& nodes);
std::optional<SurveyResult> survey_poll();
std::vector<SurveyResult> survey_abort();
std::optional<SurveyResult> survey_result(uint16_t id);
std::optional<SurveyNode> survey_node(uint16_t id);
void survey_reset();

}  // namespace ccu
