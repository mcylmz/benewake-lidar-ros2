#pragma once

#include <string>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>

namespace benewake_lidar {

// HTTP REST client for configuring the LIDAR sensor and reading telemetry.
//
// Uses boost::beast for HTTP transport and nlohmann/json for response parsing.
// Each request creates a short-lived TCP connection with a 3-second timeout.
class SensorRestClient {
 public:
  SensorRestClient(const std::string& sensor_ip, uint16_t http_port,
                   rclcpp::Logger logger);
  ~SensorRestClient() = default;

  SensorRestClient(const SensorRestClient&) = delete;
  SensorRestClient& operator=(const SensorRestClient&) = delete;

  // Configuration methods (HTTP PUT). Return true on HTTP 200.
  bool SetScanFrequency(const std::string& value);
  bool SetLaserEnable(const std::string& value);
  bool SetScanRangeStart(const std::string& value);
  bool SetScanRangeStop(const std::string& value);
  bool SetFilter(const std::string& value);

  // Telemetry methods (HTTP GET). Return parsed JSON or empty object on error.
  nlohmann::json GetFirmwareInfo();
  nlohmann::json GetSystemMonitor();
  nlohmann::json GetSensorOverview();

  // Log all telemetry endpoints to the ROS logger.
  void LogTelemetry();

 private:
  bool HttpPut(const std::string& endpoint, const std::string& body);
  nlohmann::json HttpGet(const std::string& endpoint);

  std::string sensor_ip_;
  uint16_t http_port_;
  rclcpp::Logger logger_;
};

}  // namespace benewake_lidar
