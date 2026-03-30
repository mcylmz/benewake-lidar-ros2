#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "benewake_lidar/msop_packet.h"
#include "benewake_lidar/scan_assembler.h"
#include "benewake_lidar/sensor_rest_client.h"
#include "benewake_lidar/udp_receiver.h"

namespace benewake_lidar {

class BenewakeLidarNode : public rclcpp::Node {
 public:
  explicit BenewakeLidarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
      : Node("laser_scan_publisher", options) {
    DeclareAndGetParameters();

    scan_publisher_ = create_publisher<sensor_msgs::msg::LaserScan>(
        output_topic_, rclcpp::SensorDataQoS());

    LogConfiguration();

    if (configure_sensor_) {
      rest_client_ = std::make_unique<SensorRestClient>(
          sensor_ip_, sensor_http_port_, get_logger());
      ConfigureSensor();
    }

    ScanConfig config;
    config.frame_id = frame_id_;
    config.inverted = inverted_;
    config.angle_offset = angle_offset_;

    scan_assembler_ = std::make_unique<ScanAssembler>(
        config,
        [this](sensor_msgs::msg::LaserScan::UniquePtr scan) {
          OnScanReady(std::move(scan));
        },
        get_logger());

    udp_receiver_ = std::make_unique<UdpReceiver>(
        host_ip_, static_cast<uint16_t>(port_),
        [this](const uint8_t* data, std::size_t len) {
          scan_assembler_->ProcessPacket(data, len);
        },
        get_logger());

    udp_receiver_->Start();
    RCLCPP_INFO(get_logger(), "Benewake LIDAR node started successfully");
  }

  ~BenewakeLidarNode() override {
    RCLCPP_INFO(get_logger(), "Benewake LIDAR node shutting down");
    if (udp_receiver_) {
      udp_receiver_->Stop();
    }
  }

 private:
  void DeclareAndGetParameters() {
    frame_id_ = declare_parameter<std::string>("frame_id", "laser");
    host_ip_ = declare_parameter<std::string>("host_ip", "0.0.0.0");
    sensor_ip_ = declare_parameter<std::string>("sensor_ip", "192.168.198.2");
    port_ = declare_parameter<int>("port", 2368);
    sensor_http_port_ = declare_parameter<int>("sensor_http_port", 80);
    output_topic_ = declare_parameter<std::string>("output_topic", "scan");
    inverted_ = declare_parameter<bool>("inverted", false);
    angle_offset_ = declare_parameter<int>("angle_offset", 0);

    scan_freq_ = declare_parameter<std::string>("scan_freq", "30");
    filter_ = declare_parameter<std::string>("filter", "3");
    laser_enable_ = declare_parameter<std::string>("laser_enable", "true");
    scan_range_start_ = declare_parameter<std::string>("scan_range_start", "45");
    scan_range_stop_ = declare_parameter<std::string>("scan_range_stop", "315");
    configure_sensor_ = declare_parameter<bool>("configure_sensor", false);
  }

  void LogConfiguration() {
    RCLCPP_INFO(get_logger(), "Configuration:");
    RCLCPP_INFO(get_logger(), "  frame_id:     %s", frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "  host_ip:      %s", host_ip_.c_str());
    RCLCPP_INFO(get_logger(), "  sensor_ip:    %s", sensor_ip_.c_str());
    RCLCPP_INFO(get_logger(), "  port:         %d", port_);
    RCLCPP_INFO(get_logger(), "  topic:        %s", output_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  inverted:     %s", inverted_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  angle_offset: %d", angle_offset_);
  }

  void ConfigureSensor() {
    RCLCPP_INFO(get_logger(), "Configuring sensor at %s ...",
                sensor_ip_.c_str());
    rest_client_->SetScanFrequency(scan_freq_);
    rest_client_->SetLaserEnable(laser_enable_);
    rest_client_->SetScanRangeStart(scan_range_start_);
    rest_client_->SetScanRangeStop(scan_range_stop_);
    rest_client_->SetFilter(filter_);
    rest_client_->LogTelemetry();
  }

  void OnScanReady(sensor_msgs::msg::LaserScan::UniquePtr scan) {
    RCLCPP_DEBUG(get_logger(), "Publishing scan: %zu points",
                 scan->ranges.size());
    scan_publisher_->publish(std::move(scan));
  }

  // Parameters
  std::string frame_id_;
  std::string host_ip_;
  std::string sensor_ip_;
  int port_ = 2368;
  int sensor_http_port_ = 80;
  std::string output_topic_;
  bool inverted_ = false;
  int angle_offset_ = 0;
  std::string scan_freq_;
  std::string filter_;
  std::string laser_enable_;
  std::string scan_range_start_;
  std::string scan_range_stop_;
  bool configure_sensor_ = false;

  // ROS2 publisher
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;

  // Components
  std::unique_ptr<UdpReceiver> udp_receiver_;
  std::unique_ptr<ScanAssembler> scan_assembler_;
  std::unique_ptr<SensorRestClient> rest_client_;
};

}  // namespace benewake_lidar

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<benewake_lidar::BenewakeLidarNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
