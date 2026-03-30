#include "benewake_lidar/udp_receiver.h"

namespace benewake_lidar {

namespace asio = boost::asio;
using udp = asio::ip::udp;

UdpReceiver::UdpReceiver(const std::string& host_ip, uint16_t port,
                         PacketCallback callback, rclcpp::Logger logger)
    : host_ip_(host_ip),
      port_(port),
      callback_(std::move(callback)),
      logger_(logger) {}

UdpReceiver::~UdpReceiver() {
  Stop();
}

void UdpReceiver::Start() {
  if (running_.load()) {
    return;
  }

  auto address = asio::ip::make_address(host_ip_);
  udp::endpoint endpoint(address, port_);

  socket_ = std::make_unique<udp::socket>(io_context_);
  socket_->open(udp::v4());
  socket_->set_option(asio::socket_base::reuse_address(true));
  socket_->bind(endpoint);

  running_.store(true);
  StartAsyncReceive();

  worker_thread_ = std::thread([this]() {
    RCLCPP_INFO(logger_, "UDP receiver thread started on %s:%d",
                host_ip_.c_str(), port_);
    io_context_.run();
    RCLCPP_INFO(logger_, "UDP receiver thread exiting");
  });
}

void UdpReceiver::Stop() {
  if (!running_.load()) {
    return;
  }

  running_.store(false);
  io_context_.stop();

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  if (socket_ && socket_->is_open()) {
    boost::system::error_code ec;
    socket_->close(ec);
  }
  socket_.reset();

  io_context_.restart();
  RCLCPP_INFO(logger_, "UDP receiver stopped");
}

void UdpReceiver::StartAsyncReceive() {
  if (!socket_ || !socket_->is_open()) {
    return;
  }

  socket_->async_receive_from(
      asio::buffer(recv_buffer_), sender_endpoint_,
      [this](const boost::system::error_code& ec,
             std::size_t bytes_received) {
        HandleReceive(ec, bytes_received);
      });
}

void UdpReceiver::HandleReceive(const boost::system::error_code& ec,
                                std::size_t bytes_received) {
  if (!running_.load()) {
    return;
  }

  if (ec) {
    if (ec == asio::error::operation_aborted) {
      return;
    }
    RCLCPP_WARN(logger_, "UDP receive error: %s", ec.message().c_str());
    StartAsyncReceive();
    return;
  }

  if (bytes_received != kMsopPacketSize) {
    RCLCPP_DEBUG(logger_,
                 "Unexpected packet size: %zu bytes (expected %zu)",
                 bytes_received, kMsopPacketSize);
    StartAsyncReceive();
    return;
  }

  callback_(recv_buffer_.data(), bytes_received);
  StartAsyncReceive();
}

}  // namespace benewake_lidar
