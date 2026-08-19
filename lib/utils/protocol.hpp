#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>

inline constexpr uint8_t kControlHeader = 0x55;
inline constexpr uint8_t kStatusHeader = 0xAA;
inline constexpr uint8_t kDebugHeader = 0xFF;
inline constexpr uint16_t kDebugMaxLen = 256;  // HOST_PROTOCOL.md §7

// HOST_PROTOCOL.md §2 / §4: every frame is
//   [header : uint8] [len : uint16 LE] [payload : len bytes]
// DeviceParser does not interpret header. MessageConsumer routes on it.
// Device → host headers (§4):
//   0x55 control  len=26  total=29
//   0xAA status   len=21  total=24
//   0xFF debug    len=payload byte count, max 256, total=3+len

inline constexpr uint8_t kScanModeOff = 0;
inline constexpr uint8_t kScanModeCoarse = 1;
inline constexpr uint8_t kScanModePoint2Point = 2;
inline constexpr uint8_t kRoutineNone = 0;
inline constexpr uint8_t kRoutineWindingUpToCoarse = 1;
inline constexpr uint8_t kRoutineTaperingDownFromCoarse = 2;

// HOST_PROTOCOL.md §4.1 / §4.2 — packed on-wire layouts, little-endian, no padding.

inline constexpr uint16_t kControlPacketSize = 29;
inline constexpr uint16_t kStatusPacketSize = 24;

inline constexpr uint16_t kControlLenField = 26;  // sizeof(ControlPacket) - 3
inline constexpr uint16_t kStatusLenField = 21;   // sizeof(StatusPacket) - 3

inline constexpr uint16_t kControlPayloadSize = kControlPacketSize - 3;  // 26
inline constexpr uint16_t kStatusPayloadSize = kStatusPacketSize - 3;    // 21

#pragma pack(push, 1)
struct ControlPayload {
    float target_angle_x;
    float target_angle_y;
    uint16_t distance;
    float mechanical_angle_x;
    float mechanical_angle_y;
    float associated_angle_x;
    float associated_angle_y;
};

struct StatusPayload {
    uint8_t enabled;
    uint8_t scan_mode;
    uint8_t dynamic_routine;
    uint8_t debug_mode;
    uint8_t has_fault;
    float coarse_centre_x;
    float coarse_centre_y;
    float p2p_target_x;
    float p2p_target_y;
};

// Full firmware packets (Interface.hpp), including the 3-byte prefix.
struct ControlPacket {
    uint8_t header;
    uint16_t len;
    ControlPayload body;
};

struct StatusPacket {
    uint8_t header;
    uint16_t len;
    StatusPayload body;
};
#pragma pack(pop)

static_assert(sizeof(ControlPayload) == kControlPayloadSize);
static_assert(sizeof(StatusPayload) == kStatusPayloadSize);
static_assert(sizeof(ControlPacket) == kControlPacketSize);
static_assert(sizeof(StatusPacket) == kStatusPacketSize);
static_assert(offsetof(ControlPacket, body) == 3);
static_assert(offsetof(StatusPacket, body) == 3);
static_assert(offsetof(ControlPacket, body.target_angle_x) == 3);
static_assert(offsetof(ControlPacket, body.target_angle_y) == 7);
static_assert(offsetof(ControlPacket, body.distance) == 11);
static_assert(offsetof(ControlPacket, body.mechanical_angle_x) == 13);
static_assert(offsetof(ControlPacket, body.mechanical_angle_y) == 17);
static_assert(offsetof(ControlPacket, body.associated_angle_x) == 21);
static_assert(offsetof(ControlPacket, body.associated_angle_y) == 25);
static_assert(offsetof(StatusPacket, body.enabled) == 3);
static_assert(offsetof(StatusPacket, body.scan_mode) == 4);
static_assert(offsetof(StatusPacket, body.dynamic_routine) == 5);
static_assert(offsetof(StatusPacket, body.debug_mode) == 6);
static_assert(offsetof(StatusPacket, body.has_fault) == 7);
static_assert(offsetof(StatusPacket, body.coarse_centre_x) == 8);
static_assert(offsetof(StatusPacket, body.coarse_centre_y) == 12);
static_assert(offsetof(StatusPacket, body.p2p_target_x) == 16);
static_assert(offsetof(StatusPacket, body.p2p_target_y) == 20);
static_assert(kControlLenField == kControlPacketSize - 3);
static_assert(kStatusLenField == kStatusPacketSize - 3);

inline uint16_t telemetry_packet_size(uint8_t header) {
    if (header == kControlHeader) return kControlPacketSize;
    if (header == kStatusHeader) return kStatusPacketSize;
    return 0;
}

inline uint16_t telemetry_payload_size(uint8_t header) {
    if (header == kControlHeader) return kControlPayloadSize;
    if (header == kStatusHeader) return kStatusPayloadSize;
    return 0;
}

inline bool unpack_control(std::span<const uint8_t> payload, ControlPayload& out) {
    if (payload.size() < sizeof(ControlPayload)) return false;
    std::memcpy(&out, payload.data(), sizeof(out));
    return true;
}

inline bool unpack_status(std::span<const uint8_t> payload, StatusPayload& out) {
    if (payload.size() < sizeof(StatusPayload)) return false;
    std::memcpy(&out, payload.data(), sizeof(out));
    return true;
}

// HOST_PROTOCOL.md §7: payload bytes are ASCII code points 0–127; `len` is the
// character count. No NUL inside the payload. Equivalent to:
//   memcpy(text, payload, len); text[len] = '\0';
inline std::string decode_debug_ascii(std::span<const uint8_t> payload) {
    std::string text(payload.size(), '\0');
    if (!payload.empty())
        std::memcpy(text.data(), payload.data(), payload.size());
    return text;
}

inline const char* scan_mode_name(uint8_t mode) {
    switch (mode) {
        case 0: return "OFF";
        case 1: return "COARSE";
        case 2: return "POINT2POINT";
        default: return "?";
    }
}

inline const char* dynamic_routine_name(uint8_t routine) {
    switch (routine) {
        case 0: return "NONE";
        case 1: return "WINDING_UP_TO_COARSE";
        case 2: return "TAPERING_DOWN_FROM_COARSE";
        default: return "?";
    }
}
