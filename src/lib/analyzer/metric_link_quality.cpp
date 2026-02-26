#include "edu_camera/analyzer/metric_link_quality.hpp"

#include <chrono>
#include <cstring>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/wireless.h>
#include <unistd.h>
#include <cmath>

#include <rclcpp/logging.hpp>

namespace eduart {
namespace camera {
namespace analyzer {

MetricLinkQuality::MetricLinkQuality(
  const float scale, const std::string& interface, const std::chrono::milliseconds& measurement_interval)
  : WifiMetric(scale)
  , _measurement_interval(measurement_interval)
  , _interface(interface)
{
  // Open socket once for reuse
  _sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (_sock < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MetricLinkQuality"), "failed to open socket for Link Quality measurement");
  }
  
  // Start background measurement thread
  _measurement_thread = std::thread(&MetricLinkQuality::measurementLoop, this);
}

MetricLinkQuality::~MetricLinkQuality()
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

void MetricLinkQuality::measurementLoop()
{
  auto stamp_last_measurement = std::chrono::steady_clock::now();

  while (_running) {
    // Calculate new score
    const float new_score = calculateLinkQualityScore();
    
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

float MetricLinkQuality::calculateLinkQualityScore()
{
  if (_sock < 0) {
    return 0.0f; // Socket not available
  }

  struct iwreq req;
  std::memset(&req, 0, sizeof(req));
  std::strncpy(req.ifr_name, _interface.c_str(), IFNAMSIZ - 1);

  struct iw_statistics stats;
  req.u.data.pointer = &stats;
  req.u.data.length = sizeof(stats);

  if (ioctl(_sock, SIOCGIWSTATS, &req) < 0) {
     // Error getting wireless stats
    RCLCPP_ERROR(
      rclcpp::get_logger("MetricLinkQuality"), "failed to get wireless stats for interface '%s'",
      _interface.c_str()
    );
    return 0.0f;
  }

  // Extract link quality value
  // Link quality is typically a value between 0 and max (often 70 or 100)
  const uint8_t quality = stats.qual.qual;
  const uint8_t quality_max = 70; // Typical max value, may vary by driver

  // Normalize to 0-100 score
  const float score = (static_cast<float>(quality) / quality_max) * 100.0f;
  RCLCPP_INFO(rclcpp::get_logger("MetricLinkQuality"), "Link Quality: %d/%d -> Score: %.2f", static_cast<int>(quality), quality_max, score);
  return std::min(100.0f, score);
}

} // namespace analyzer
} // namespace camera
} // namespace eduart
