#include "benewake_lidar/sensor_rest_client.h"

#include <chrono>
#include <string>

#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>

namespace benewake_lidar {

namespace beast = boost::beast;
namespace http = beast::http;
namespace net = boost::asio;
using tcp = net::ip::tcp;

SensorRestClient::SensorRestClient(const std::string& sensor_ip,
                                   uint16_t http_port,
                                   rclcpp::Logger logger)
    : sensor_ip_(sensor_ip), http_port_(http_port), logger_(logger) {}

bool SensorRestClient::HttpPut(const std::string& endpoint,
                               const std::string& body) {
  try {
    net::io_context ioc;
    tcp::resolver resolver(ioc);
    beast::tcp_stream stream(ioc);

    auto const results =
        resolver.resolve(sensor_ip_, std::to_string(http_port_));
    stream.expires_after(std::chrono::seconds(3));
    stream.connect(results);

    http::request<http::string_body> req(http::verb::put, endpoint, 11);
    req.set(http::field::host, sensor_ip_);
    req.set(http::field::content_type, "application/json");
    req.body() = body;
    req.prepare_payload();

    http::write(stream, req);

    beast::flat_buffer buffer;
    http::response<http::string_body> res;
    http::read(stream, buffer, res);

    beast::error_code ec;
    stream.socket().shutdown(tcp::socket::shutdown_both, ec);

    if (res.result_int() == 200) {
      RCLCPP_INFO(logger_, "Set %s = %s ... OK",
                  endpoint.c_str(), body.c_str());
      return true;
    }

    RCLCPP_WARN(logger_, "Set %s = %s ... HTTP %u",
                endpoint.c_str(), body.c_str(), res.result_int());
    return false;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "HTTP PUT %s failed: %s",
                 endpoint.c_str(), e.what());
    return false;
  }
}

nlohmann::json SensorRestClient::HttpGet(const std::string& endpoint) {
  try {
    net::io_context ioc;
    tcp::resolver resolver(ioc);
    beast::tcp_stream stream(ioc);

    auto const results =
        resolver.resolve(sensor_ip_, std::to_string(http_port_));
    stream.expires_after(std::chrono::seconds(3));
    stream.connect(results);

    http::request<http::empty_body> req(http::verb::get, endpoint, 11);
    req.set(http::field::host, sensor_ip_);

    http::write(stream, req);

    beast::flat_buffer buffer;
    http::response<http::string_body> res;
    http::read(stream, buffer, res);

    beast::error_code ec;
    stream.socket().shutdown(tcp::socket::shutdown_both, ec);

    return nlohmann::json::parse(res.body());

  } catch (const nlohmann::json::parse_error& e) {
    RCLCPP_ERROR(logger_, "JSON parse error for %s: %s",
                 endpoint.c_str(), e.what());
    return {};
  } catch (const std::exception& e) {
    RCLCPP_ERROR(logger_, "HTTP GET %s failed: %s",
                 endpoint.c_str(), e.what());
    return {};
  }
}

bool SensorRestClient::SetScanFrequency(const std::string& value) {
  return HttpPut("/api/v1/sensor/scanfreq", value);
}

bool SensorRestClient::SetLaserEnable(const std::string& value) {
  return HttpPut("/api/v1/sensor/laser_enable", value);
}

bool SensorRestClient::SetScanRangeStart(const std::string& value) {
  return HttpPut("/api/v1/sensor/scan_range/start", value);
}

bool SensorRestClient::SetScanRangeStop(const std::string& value) {
  return HttpPut("/api/v1/sensor/scan_range/stop", value);
}

bool SensorRestClient::SetFilter(const std::string& value) {
  return HttpPut("/api/v1/sensor/filter", value);
}

nlohmann::json SensorRestClient::GetFirmwareInfo() {
  return HttpGet("/api/v1/system/firmware");
}

nlohmann::json SensorRestClient::GetSystemMonitor() {
  return HttpGet("/api/v1/system/monitor");
}

nlohmann::json SensorRestClient::GetSensorOverview() {
  return HttpGet("/api/v1/sensor/overview");
}

void SensorRestClient::LogTelemetry() {
  RCLCPP_INFO(logger_, "--- Sensor Telemetry ---");

  auto fw = GetFirmwareInfo();
  if (!fw.empty()) {
    RCLCPP_INFO(logger_, "Model:    %s", fw.value("model", "?").c_str());
    RCLCPP_INFO(logger_, "SN:       %s", fw.value("sn", "?").c_str());
    RCLCPP_INFO(logger_, "HW:       %s", fw.value("hw", "?").c_str());
    RCLCPP_INFO(logger_, "FPGA:     %s", fw.value("fpga", "?").c_str());
    RCLCPP_INFO(logger_, "Core:     %s", fw.value("core", "?").c_str());
    RCLCPP_INFO(logger_, "Aux:      %s", fw.value("aux", "?").c_str());
  }

  auto mon = GetSystemMonitor();
  if (!mon.empty()) {
    RCLCPP_INFO(logger_, "Load avg: %.2f", mon.value("load_average", 0.0));
    RCLCPP_INFO(logger_, "Mem use:  %.2f", mon.value("mem_useage", 0.0));
    RCLCPP_INFO(logger_, "Uptime:   %.2f s", mon.value("uptime", 0.0));
  }

  auto overview = GetSensorOverview();
  if (!overview.empty()) {
    RCLCPP_INFO(logger_, "Scan freq:    %d Hz",
                overview.value("scanfreq", 0));
    RCLCPP_INFO(logger_, "Motor RPM:    %d",
                overview.value("motor_rpm", 0));
    RCLCPP_INFO(logger_, "Laser enable: %s",
                overview.value("laser_enable", false) ? "true" : "false");

    if (overview.contains("scan_range")) {
      auto& sr = overview["scan_range"];
      RCLCPP_INFO(logger_, "Scan range:   %d - %d deg",
                  sr.value("start", 0), sr.value("stop", 0));
    }

    if (overview.contains("filter")) {
      auto& flt = overview["filter"];
      RCLCPP_INFO(logger_, "Filter level: %d", flt.value("level", 0));
    }

    if (overview.contains("host")) {
      auto& host = overview["host"];
      RCLCPP_INFO(logger_, "Host:         %s:%d",
                  host.value("ip", "?").c_str(), host.value("port", 0));
    }
  }

  RCLCPP_INFO(logger_, "------------------------");
}

}  // namespace benewake_lidar
