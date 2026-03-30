#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "benewake_lidar/msop_packet.h"

namespace benewake_lidar {

using ScanCallback =
    std::function<void(sensor_msgs::msg::LaserScan::UniquePtr scan)>;

struct ScanConfig {
  std::string frame_id = "laser";
  bool inverted = false;
  int angle_offset = 0;  // degrees
};

// Assembles complete 360-degree scans from individual MSOP packets.
//
// Receives raw packet buffers (from UdpReceiver), parses the MSOP protocol,
// accumulates scan points, and invokes a callback with a complete LaserScan
// message when a full revolution is detected (azimuth wraps past zero).
class ScanAssembler {
 public:
  ScanAssembler(const ScanConfig& config, ScanCallback callback,
                rclcpp::Logger logger);

  // Called from the UdpReceiver's IO thread for each validated packet.
  void ProcessPacket(const uint8_t* data, std::size_t length);

 private:
  void FinalizeAndPublishScan();

  ScanConfig config_;
  ScanCallback callback_;
  rclcpp::Logger logger_;

  std::vector<ScanPoint> scan_points_;
  uint16_t resolution_ = kDefaultResolution;
  uint16_t prev_azimuth_ = 0;

  rclcpp::Time scan_start_time_;
  rclcpp::Time prev_scan_start_time_;
  bool first_scan_ = true;
};

}  // namespace benewake_lidar
