#pragma once

#include "utils/protocol.hpp"

#include <cstdint>
#include <vector>

namespace ccu {

using LidarRead = int (*)(uint8_t* dst, int cap);
using LidarWrite = int (*)(const uint8_t* src, int n);

// The Pi build binds the serial port. Tests leave it unbound and push payloads.
void lidar_bind_port(LidarRead read, LidarWrite write);
void lidar_poll();

void lidar_push_status(const StatusPayload& status);
void lidar_push_control(const ControlPayload& control);
void lidar_test_reset();

bool lidar_take_status(StatusPayload& out);
std::vector<ControlPayload> lidar_take_controls();

void lidar_cmd(uint8_t cmd);
void lidar_cmd_f32(uint8_t cmd, float value);
void lidar_cmd_f32_pair(uint8_t cmd, float x, float y);
std::vector<std::vector<uint8_t>> lidar_take_commands();

bool lidar_open(const char* path);
void lidar_close();

}  // namespace ccu
