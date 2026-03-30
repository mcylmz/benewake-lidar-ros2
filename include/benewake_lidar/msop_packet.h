#pragma once

#include <cstddef>
#include <cstdint>

namespace benewake_lidar {

// MSOP (Multi-block Socket Protocol) constants.
constexpr int kMsopBlockCount = 12;
constexpr int kMsopChannelCount = 16;
constexpr uint16_t kDataFlagValid = 0xEEFF;
constexpr std::size_t kMsopPacketSize = 1206;
constexpr float kDistanceScale = 0.001f;  // mm -> meters
constexpr float kMaxRange = 100.0f;       // meters
constexpr float kMinRange = 0.0f;         // meters
constexpr uint16_t kDefaultResolution = 25;  // 0.25 deg in 0.01-deg units
constexpr uint16_t kFullRotation = 36000;    // 360.00 deg in 0.01-deg units

// Packed structs matching the MSOP wire format exactly.
// All multi-byte fields are little-endian on the wire.
#pragma pack(push, 1)

struct MeasuringResult {
  uint16_t dist_1;   // First return distance (mm)
  uint8_t rssi_1;    // First return signal strength
  uint16_t dist_2;   // Second return distance (mm)
  uint8_t rssi_2;    // Second return signal strength
};

struct DataBlock {
  uint16_t data_flag;  // 0xEEFF if valid
  uint16_t azimuth;    // Rotation angle in 0.01-degree units
  MeasuringResult results[kMsopChannelCount];
};

struct MsopPacket {
  DataBlock blocks[kMsopBlockCount];
  uint32_t timestamp;
  uint16_t factory;
};

#pragma pack(pop)

static_assert(sizeof(MeasuringResult) == 6,
              "MeasuringResult size mismatch with wire protocol");
static_assert(sizeof(DataBlock) == 100,
              "DataBlock size mismatch with wire protocol");
static_assert(sizeof(MsopPacket) == kMsopPacketSize,
              "MsopPacket size mismatch with wire protocol");

// Processed scan point (not packed — used internally).
struct ScanPoint {
  uint16_t azimuth = 0;    // 0.01-degree units
  uint16_t distance = 0;   // mm
  uint8_t rssi = 0;
};

}  // namespace benewake_lidar
