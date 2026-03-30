#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <rclcpp/rclcpp.hpp>

#include "benewake_lidar/msop_packet.h"

namespace benewake_lidar {

using PacketCallback =
    std::function<void(const uint8_t* data, std::size_t length)>;

// Asynchronous UDP receiver using boost::asio.
//
// Owns an io_context and a worker thread. Delivers validated packets
// (correct size) to the registered callback on the worker thread.
class UdpReceiver {
 public:
  UdpReceiver(const std::string& host_ip, uint16_t port,
              PacketCallback callback, rclcpp::Logger logger);
  ~UdpReceiver();

  UdpReceiver(const UdpReceiver&) = delete;
  UdpReceiver& operator=(const UdpReceiver&) = delete;

  void Start();
  void Stop();

 private:
  void StartAsyncReceive();
  void HandleReceive(const boost::system::error_code& ec,
                     std::size_t bytes_received);

  std::string host_ip_;
  uint16_t port_;
  PacketCallback callback_;
  rclcpp::Logger logger_;

  boost::asio::io_context io_context_;
  std::unique_ptr<boost::asio::ip::udp::socket> socket_;
  boost::asio::ip::udp::endpoint sender_endpoint_;

  static constexpr std::size_t kBufferSize = 2048;
  std::array<uint8_t, kBufferSize> recv_buffer_{};

  std::thread worker_thread_;
  std::atomic<bool> running_{false};
};

}  // namespace benewake_lidar
