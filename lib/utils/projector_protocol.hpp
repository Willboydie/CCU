#pragma once

#include <cstdint>
#include <cstring>
#include <span>

// PROJECTOR_HOST_PROTOCOL.md — Teensy 4.1 laser projector over USB CDC.

inline constexpr int kProjBaud = 115200;

inline constexpr uint16_t kProjDacMin = 0;
inline constexpr uint16_t kProjDacMax = 65535;
inline constexpr uint16_t kProjDacCentre = 32768;

inline constexpr uint8_t kProjModePath = 0;
inline constexpr uint8_t kProjModePoint = 1;

inline constexpr uint8_t kProjPowerSleeping = 0;
inline constexpr uint8_t kProjPowerAwake = 1;

inline constexpr uint8_t kProjStreamRunning = 0;
inline constexpr uint8_t kProjStreamPaused = 1;
inline constexpr uint8_t kProjStreamReset = 2;

// Host → device
inline constexpr uint8_t kProjCmdPlay = 0x07;
inline constexpr uint8_t kProjCmdStop = 0x08;
inline constexpr uint8_t kProjCmdPlayPause = 0x09;
inline constexpr uint8_t kProjCmdSetPlaybackSpeed = 0x0A;
inline constexpr uint8_t kProjCmdSendStatus = 0x0B;
inline constexpr uint8_t kProjCmdDebugMode = 0x1A;
inline constexpr uint8_t kProjCmdSetLaserDuty = 0x1B;
inline constexpr uint8_t kProjCmdToggleProjectMode = 0x3A;
inline constexpr uint8_t kProjCmdSetPoint = 0x3B;
inline constexpr uint8_t kProjCmdRestart = 0x8E;
inline constexpr uint8_t kProjCmdEnableDisable = 0x8F;
inline constexpr uint8_t kProjCmdSleep = 0xE0;
inline constexpr uint8_t kProjCmdWake = 0xE1;
inline constexpr uint8_t kProjCmdBootloader = 0xE2;
inline constexpr uint8_t kProjCmdResetStream = 0xF7;
inline constexpr uint8_t kProjCmdGetFaultMsg = 0xFC;

// Device → host
inline constexpr uint8_t kProjNotifyResetStream = 0xF7;
inline constexpr uint8_t kProjNotifyPauseStream = 0xF8;
inline constexpr uint8_t kProjNotifyResumeStream = 0xF9;
inline constexpr uint8_t kProjStatusHeader = 0xFA;
inline constexpr uint8_t kProjDebugHeader = 0xFF;
inline constexpr uint8_t kProjDebugPrefix0 = 0xFF;
inline constexpr uint8_t kProjDebugPrefix1 = 0x04;
inline constexpr uint8_t kProjDebugPrefix2 = 0x06;

inline constexpr uint16_t kProjStatusPayloadSize = 16;

#pragma pack(push, 1)
struct ProjectorStatus {
    uint8_t power_state;
    uint8_t mode;
    uint8_t enabled;
    uint8_t projecting;
    uint8_t laser_duty_cycle;
    uint8_t projector_delay;
    uint16_t points_frequency;
    uint32_t file_size;
    uint8_t debug_mode;
    uint8_t project_mode;
    uint8_t stream_state;
    uint8_t has_fault;
};
#pragma pack(pop)

static_assert(sizeof(ProjectorStatus) == kProjStatusPayloadSize);

inline bool unpack_projector_status(std::span<const uint8_t> payload,
                                    ProjectorStatus& out) {
    if (payload.size() < sizeof(ProjectorStatus))
        return false;
    std::memcpy(&out, payload.data(), sizeof(out));
    return true;
}

inline void pack_set_point(uint8_t out[5], uint16_t x, uint16_t y, bool laser) {
    out[0] = static_cast<uint8_t>(x & 0xFF);
    out[1] = static_cast<uint8_t>((x >> 8) & 0xFF);
    out[2] = static_cast<uint8_t>(y & 0xFF);
    out[3] = static_cast<uint8_t>((y >> 8) & 0xFF);
    out[4] = laser ? 1 : 0;
}

inline uint16_t saturate_dac(int value) {
    if (value < static_cast<int>(kProjDacMin))
        return kProjDacMin;
    if (value > static_cast<int>(kProjDacMax))
        return kProjDacMax;
    return static_cast<uint16_t>(value);
}

inline const char* proj_mode_name(uint8_t mode) {
    switch (mode) {
        case kProjModePath: return "PATH";
        case kProjModePoint: return "POINT";
        default: return "?";
    }
}

inline const char* proj_power_name(uint8_t power) {
    switch (power) {
        case kProjPowerSleeping: return "sleeping";
        case kProjPowerAwake: return "awake";
        default: return "?";
    }
}

inline const char* proj_stream_name(uint8_t state) {
    switch (state) {
        case kProjStreamRunning: return "RUNNING";
        case kProjStreamPaused: return "PAUSED";
        case kProjStreamReset: return "RESET";
        default: return "?";
    }
}
