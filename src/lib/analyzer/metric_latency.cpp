#include "edu_camera/analyzer/metric_latency.hpp"

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

MetricLatency::MetricLatency(
  const float scale, const std::string& target_host, const std::chrono::milliseconds& measurement_interval)
  : WifiMetric(scale)
  , _measurement_interval(measurement_interval)
  , _target_host(target_host)
{
  // Open ICMP socket once for reuse
  _sock = socket(AF_INET, SOCK_RAW, IPPROTO_ICMP);
  if (_sock < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MetricLatency"), "failed to open ICMP socket (requires root privileges)");
  }
  
  // Set receive timeout
  struct timeval tv;
  tv.tv_sec = 1;
  tv.tv_usec = 0;
  setsockopt(_sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  
  // Start background measurement thread
  _measurement_thread = std::thread(&MetricLatency::measurementLoop, this);
}

MetricLatency::~MetricLatency()
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

void MetricLatency::measurementLoop()
{
  auto stamp_last_measurement = std::chrono::steady_clock::now();

  while (_running) {
    // Calculate new score
    const float new_score = calculateLatencyScore();
    RCLCPP_INFO(rclcpp::get_logger("MetricLatency"), "Latency Score: %.2f", new_score);
    
    // Update the score (thread-safe via base class method)
    update(new_score);
    
    // Sleep for the remaining time until the next measurement
    const auto stamp_now = std::chrono::steady_clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(stamp_now - stamp_last_measurement);
    const auto sleep_duration = (_measurement_interval > elapsed) ? (_measurement_interval - elapsed) : std::chrono::milliseconds(0);
    
    std::this_thread::sleep_for(sleep_duration);
    stamp_last_measurement = std::chrono::steady_clock::now();
  }
}

float MetricLatency::sendPing()
{
  if (_sock < 0) {
    return -1.0f;
  }

  // Prepare destination address
  struct sockaddr_in addr;
  std::memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  
  if (inet_pton(AF_INET, _target_host.c_str(), &addr.sin_addr) <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MetricLatency"),
      "invalid target host IP address '%s'", _target_host.c_str()
    );
    return -1.0f;
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

  // Send packet and measure time
  const auto start = std::chrono::steady_clock::now();
  
  if (sendto(_sock, packet, sizeof(packet), 0, (struct sockaddr*)&addr, sizeof(addr)) <= 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("MetricLatency"), "failed to send ICMP packet to '%s'", _target_host.c_str()
    );
    return -1.0f;
  }

  // Receive response
  char recv_buf[1024];
  struct sockaddr_in recv_addr;
  socklen_t addr_len = sizeof(recv_addr);
  
  const int bytes = recvfrom(_sock, recv_buf, sizeof(recv_buf), 0, (struct sockaddr*)&recv_addr, &addr_len);
  const auto end = std::chrono::steady_clock::now();
  
  if (bytes < 0) {
    return -1.0f; // Timeout or error
  }

  // Calculate RTT in milliseconds
  const auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  return duration.count() / 1000.0f;
}

float MetricLatency::calculateLatencyScore()
{
  const float latency_ms = sendPing();
  
  if (latency_ms < 0) {
    return 0.0f; // Ping failed
  }

  // Normalize latency to 0-100 score
  // Latency ranges: < 20 ms = excellent, 20-50 ms = good, > 50 ms = poor
  float score = 0.0f;

  if (latency_ms <= 20.0f) {
    score = 100.0f;
  } else if (latency_ms <= 50.0f) {
    // Linear interpolation between 20 and 50 ms
    score = 100.0f - ((latency_ms - 20.0f) / 30.0f) * 50.0f;
  } else if (latency_ms <= 100.0f) {
    // Linear interpolation between 50 and 100 ms
    score = 50.0f - ((latency_ms - 50.0f) / 50.0f) * 50.0f;
  } else {
    score = 0.0f;
  }

  return std::max(0.0f, score);
}

} // namespace analyzer
} // namespace camera
} // namespace eduart
