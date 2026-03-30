#include "benewake_lidar/scan_assembler.h"

#include <cmath>
#include <cstring>

namespace benewake_lidar {

namespace {
constexpr float kPi = static_cast<float>(M_PI);
}  // namespace

ScanAssembler::ScanAssembler(const ScanConfig& config, ScanCallback callback,
                             rclcpp::Logger logger)
    : config_(config),
      callback_(std::move(callback)),
      logger_(logger) {
  scan_points_.reserve(1500);
}

void ScanAssembler::ProcessPacket(const uint8_t* data, std::size_t length) {
  if (length != kMsopPacketSize) {
    RCLCPP_WARN(logger_, "Invalid packet size: %zu", length);
    return;
  }

  const auto* packet = reinterpret_cast<const MsopPacket*>(data);

  // Update resolution from first two valid blocks.
  if (packet->blocks[0].data_flag == kDataFlagValid &&
      packet->blocks[1].data_flag == kDataFlagValid) {
    auto diff = static_cast<int>(packet->blocks[1].azimuth) -
                static_cast<int>(packet->blocks[0].azimuth);
    if (diff > 0) {
      resolution_ = static_cast<uint16_t>(diff / kMsopChannelCount);
    }
  }

  for (int i = 0; i < kMsopBlockCount; ++i) {
    const auto& block = packet->blocks[i];
    if (block.data_flag != kDataFlagValid) {
      continue;
    }

    for (int j = 0; j < kMsopChannelCount; ++j) {
      auto azimuth = static_cast<uint16_t>(
          (static_cast<uint32_t>(block.azimuth) + resolution_ * j) %
          kFullRotation);

      // Detect scan boundary: azimuth wraps from high value back near zero.
      if (!first_scan_ && !scan_points_.empty() &&
          azimuth < prev_azimuth_ && prev_azimuth_ > 27000) {
        FinalizeAndPublishScan();
      }

      // Record scan start time at the zero crossing.
      if (azimuth == 0 || (first_scan_ && scan_points_.empty())) {
        prev_scan_start_time_ = scan_start_time_;
        scan_start_time_ = rclcpp::Clock().now();
        first_scan_ = false;
      }

      prev_azimuth_ = azimuth;

      ScanPoint pt;
      pt.azimuth = azimuth;
      pt.distance = block.results[j].dist_1;
      pt.rssi = block.results[j].rssi_1;
      scan_points_.push_back(pt);
    }
  }
}

void ScanAssembler::FinalizeAndPublishScan() {
  if (scan_points_.empty()) {
    return;
  }

  auto duration = static_cast<float>(
      (scan_start_time_ - prev_scan_start_time_).seconds());

  if (duration <= 0.0f || duration > 1.0f) {
    RCLCPP_WARN(logger_, "Invalid scan duration: %.3f s, skipping", duration);
    scan_points_.clear();
    return;
  }

  auto scan = std::make_unique<sensor_msgs::msg::LaserScan>();
  const auto num_readings = static_cast<uint32_t>(scan_points_.size());
  const float angle_offset_rad =
      static_cast<float>(config_.angle_offset) * kPi / 180.0f;

  scan->header.stamp = scan_start_time_;
  scan->header.frame_id = config_.frame_id;
  scan->angle_min = -kPi + angle_offset_rad;
  scan->angle_max = kPi + angle_offset_rad;
  scan->angle_increment = 2.0f * kPi / static_cast<float>(num_readings);
  scan->scan_time = duration;
  scan->time_increment = duration / static_cast<float>(num_readings);
  scan->range_min = kMinRange;
  scan->range_max = kMaxRange;
  scan->ranges.resize(num_readings);
  scan->intensities.resize(num_readings);

  for (uint32_t i = 0; i < num_readings; ++i) {
    const std::size_t idx = config_.inverted ? (num_readings - 1 - i) : i;
    scan->ranges[idx] =
        static_cast<float>(scan_points_[i].distance) * kDistanceScale;
    scan->intensities[idx] = static_cast<float>(scan_points_[i].rssi);
  }

  RCLCPP_DEBUG(logger_, "Scan assembled: %u points, duration %.3f s",
               num_readings, duration);

  callback_(std::move(scan));
  scan_points_.clear();
}

}  // namespace benewake_lidar
