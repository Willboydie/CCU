#pragma once

#include <cstdint>

// Host → device command IDs (cmd_defs.h). Frames are
// [cmd : uint8] [len : uint16 LE] [payload : len bytes], payload ≤ kCmdPayloadMax.

inline constexpr uint16_t kCmdPayloadMax = 16;

inline constexpr uint8_t kCmdDisableEnable = 0x01;
inline constexpr uint8_t kCmdSetDebugMode = 0x02;
inline constexpr uint8_t kCmdToggleControlMode = 0x03;

inline constexpr uint8_t kCmdVelocityPXSet = 0x08;
inline constexpr uint8_t kCmdVelocityIXSet = 0x09;
inline constexpr uint8_t kCmdVelocityDXSet = 0x0A;
inline constexpr uint8_t kCmdVelocityPYSet = 0x0B;
inline constexpr uint8_t kCmdVelocityIYSet = 0x0C;
inline constexpr uint8_t kCmdVelocityDYSet = 0x0D;

inline constexpr uint8_t kCmdAnglePXSet = 0x0E;
inline constexpr uint8_t kCmdAngleIXSet = 0x0F;
inline constexpr uint8_t kCmdAngleDXSet = 0x10;
inline constexpr uint8_t kCmdAnglePYSet = 0x11;
inline constexpr uint8_t kCmdAngleIYSet = 0x12;
inline constexpr uint8_t kCmdAngleDYSet = 0x13;

inline constexpr uint8_t kCmdVelocityLpfTauSet = 0x14;
inline constexpr uint8_t kCmdAngleLpfTauSet = 0x15;
inline constexpr uint8_t kCmdFocVoltageLpfTauSet = 0x16;
inline constexpr uint8_t kCmdVelocityFfSet = 0x17;
inline constexpr uint8_t kCmdAccelerationFfSet = 0x18;

inline constexpr uint8_t kCmdTargetXSet = 0x19;
inline constexpr uint8_t kCmdTargetYSet = 0x1A;

inline constexpr uint8_t kCmdSleep = 0x1B;
inline constexpr uint8_t kCmdWake = 0x1C;
inline constexpr uint8_t kCmdRestart = 0x1D;
inline constexpr uint8_t kCmdEnterBootloader = 0x1E;
inline constexpr uint8_t kCmdGetErrorMsg = 0x1F;
inline constexpr uint8_t kCmdReconfigureLdm = 0x20;

inline constexpr uint8_t kCmdBeginCoarseScanning = 0x23;
inline constexpr uint8_t kCmdXAmplitudeSet = 0x24;
inline constexpr uint8_t kCmdYAmplitudeSet = 0x25;
inline constexpr uint8_t kCmdXFrequencySet = 0x26;
inline constexpr uint8_t kCmdYFrequencySet = 0x27;
inline constexpr uint8_t kCmdXPhaseSet = 0x28;
inline constexpr uint8_t kCmdYPhaseSet = 0x29;
inline constexpr uint8_t kCmdStopCoarseScanning = 0x2A;
inline constexpr uint8_t kCmdScanEngineOn = 0x2B;
inline constexpr uint8_t kCmdScanEngineOff = 0x2C;
inline constexpr uint8_t kCmdSetPoint2PointTarget = 0x2D;
inline constexpr uint8_t kCmdUpdateCoarseScanCentre = 0x2E;
inline constexpr uint8_t kCmdSeekNeutral = 0x2F;
inline constexpr uint8_t kCmdSetVelocityMax = 0x30;
