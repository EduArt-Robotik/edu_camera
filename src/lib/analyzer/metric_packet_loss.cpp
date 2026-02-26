#include "edu_camera/analyzer/metric_packet_loss.hpp"

#include <chrono>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/ip_icmp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>

#include <rclcpp/logging.hpp>

namespace eduart {
namespace camera {
namespace analyzer {

// Calculate ICMP checksum
static uint16_t calculateChecksum(uint16_t* buf, int len)
{
  uint32_t sum = 0;
  
  while (len > 1) {
    sum += *buf++;
    len -= 2;
  }
  
  if (len == 1) {
    sum += *(uint8_t*)buf;
  }
  
  sum = (sum >> 16) + (sum & 0xFFFF);
  sum += (sum >> 16);
  
  return ~sum;
}

MetricPacketLoss::MetricPacketLoss(
  const float scale, const std::string& target_host, const std::chrono::milliseconds& measurement_interval)
  : WifiMetric(scale)
  , _measurement_interval(measurement_interval)
  , _target_host(target_host)
{
  // Open ICMP socket once for reuse
  _sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
  if (_sock < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MetricPacketLoss"), "failed to open ICMP socket (requires root privileges)");
  }
  
  // Set receive timeout
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 500000; // 500ms timeout
  setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  
  // Start background measurement thread
  _measurement_thread = std::thread(&MetricPacketLoss::measurementLoop, this);
}

MetricPacketLoss::~MetricPacketLoss()
{
  // Stop the thread
  _running = false;

  if (_measurement_thread.joinable()) {
    _measurement_thread.join();
  }
  
  // Close socket
  if (_sock >= 0) {
    close(_sock);
  }
}

void MetricPacketLoss::measurementLoop()
{
  while (_running) {
    const auto measurement_start = std::chrono::steady_clock::now();
    
    // Send 10 pings with equal spacing throughout the measurement interval
    const std::chrono::milliseconds ping_interval(_measurement_interval.count() / _pings_per_measurement);
    
    for (std::size_t i = 0; i < _pings_per_measurement && _running; ++i) {
      // Send ping and record result
      const bool success = sendPing();
      
      {
        std::lock_guard<std::mutex> lock(_results_mutex);
        _ping_results.push_back(success);
        
        // Keep only the last 100 samples
        if (_ping_results.size() > _sample_size) {
          _ping_results.pop_front();
        }
      }
      
      // Wait for the next ping interval (except after the last ping)
      if (i < _pings_per_measurement - 1) {
        std::this_thread::sleep_for(ping_interval);
      }
    }
    
    // Calculate score after all pings are sent
    const float new_score = calculatePacketLossScore();

    RCLCPP_INFO(rclcpp::get_logger("MetricPacketLoss"), "Packet Loss Score: %.2f", new_score);
    update(new_score);
    
    // Sleep for remaining time to reach the full measurement interval
    const auto measurement_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - measurement_start);
    const auto sleep_duration = (_measurement_interval > measurement_elapsed) ? 
      (_measurement_interval - measurement_elapsed) : std::chrono::milliseconds(0);
    
    std::this_thread::sleep_for(sleep_duration);
  }
}

bool MetricPacketLoss::sendPing()
{
  if (_sock < 0) {
    return false;
  }

  // Prepare destination address
  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  
  if (inet_pton(AF_INET, _target_host.c_str(), &addr.sin_addr) <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MetricPacketLoss"), "invalid target host IP address '%s'", _target_host.c_str()
    );
    return false;
  }

  // Prepare ICMP packet
  char packet[64];
  std::memset(packet, 0, sizeof(packet));
  
  struct icmphdr* icmp = (struct icmphdr*)packet;
  icmp->type = ICMP_ECHO;
  icmp->code = 0;
  icmp->un.echo.id = getpid();
  icmp->un.echo.sequence = _sequence++;
  icmp->checksum = 0;
  icmp->checksum = calculateChecksum((uint16_t*)packet, sizeof(packet));

  // Send packet
  if (sendto(_sock, packet, sizeof(packet), 0, (struct sockaddr*)&addr, sizeof(addr)) <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MetricPacketLoss"), "failed to send ICMP packet to '%s'", _target_host.c_str()
    );
    return false;
  }

  // Receive response
  char recv_buf[1024];
  struct sockaddr_in recv_addr;
  socklen_t addr_len = sizeof(recv_addr);
  
  const int bytes = recvfrom(_sock, recv_buf, sizeof(recv_buf), 0, (struct sockaddr*)&recv_addr, &addr_len);
  
  return bytes > 0;
}

float MetricPacketLoss::calculatePacketLossScore()
{
  std::lock_guard<std::mutex> lock(_results_mutex);
  
  // Need at least 10 samples (one measurement cycle) before calculating
  if (_ping_results.size() < _pings_per_measurement) {
    return 100.0f; // Default to good until we have enough data
  }
  
  // Calculate packet loss percentage
  const size_t successful_pings = std::count(_ping_results.begin(), _ping_results.end(), true);
  const float loss_percentage = 100.0f * (1.0f - (static_cast<float>(successful_pings) / _ping_results.size()));
  
  // Normalize to 0-100 score
  // Packet loss: < 1% = excellent, 1-2% = acceptable, > 2% = poor
  float score = 0.0f;
  
  if (loss_percentage <= 1.0f) {
    score = 100.0f - (loss_percentage * 10.0f);
  } else if (loss_percentage <= 5.0f) {
    score = 90.0f - ((loss_percentage - 1.0f) / 4.0f) * 40.0f;
  } else {
    score = std::max(0.0f, 50.0f - ((loss_percentage - 5.0f) / 5.0f) * 50.0f);
  }
  
  return score;
}

} // namespace analyzer
} // namespace camera
} // namespace eduart
