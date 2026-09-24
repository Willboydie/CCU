#pragma once

#include "command_allowed.hpp"
#include "protocol.hpp"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace ccu {

void session_reset();
void session_poll();
void session_set_stage(float stage_rad);
void session_on_frame(std::span<const uint8_t> frame);
bool session_link_up();
void session_expire_link();

bool session_running();
bool session_pairing_unsent();
void session_mark_pairing_sent();
bool session_origin_is_set();
std::string session_id();
float session_stage();
float session_north_offset();
float session_effective_yaw();
WorkMode session_work();
AcceptMode session_mode();
std::vector<SurveyResult> session_take_survey_results();
std::optional<ProjectionReady> session_take_projection_ready();

}  // namespace ccu
