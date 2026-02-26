#include "edu_camera/analyzer/metric_rssi.hpp"

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

MetricRssi::MetricRssi(
  const float scale, const std::string& interface, const std::chrono::milliseconds& measurement_interval)
  : WifiMetric(scale)
  , _measurement_interval(measurement_interval)
  , _interface(interface)
{
  // Open socket once for reuse
  _sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (_sock < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MetricRssi"), "failed to open socket for RSSI measurement");
  }
  
  // Start background measurement thread
  _measurement_thread = std::thread(&MetricRssi::measurementLoop, this);
}

MetricRssi::~MetricRssi()
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

void MetricRssi::measurementLoop()
{
  auto stamp_last_measurement = std::chrono::steady_clock::now();

  while (_running) {
    // Calculate new score
    const float new_score = calculateRssiScore();
    RCLCPP_INFO(rclcpp::get_logger("MetricRssi"), "RSSI Score: %.2f", new_score);
    
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

float MetricRssi::calculateRssiScore()
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
    RCLCPP_ERROR(
      rclcpp::get_logger("MetricRssi"),
      "failed to get wireless stats for interface '%s'", _interface.c_str()
    );
    return 0.0f; // Error getting wireless stats
  }

  // Extract signal level (RSSI in dBm)
  // The signal level is typically stored in stats.qual.level
  // Note: The actual value format depends on the driver
  const int8_t rssi_dbm = stats.qual.level - 256; // Convert to signed dBm

  // Normalize RSSI to 0-100 score
  // RSSI ranges: > -50 dBm = excellent, -50 to -70 dBm = good, < -70 dBm = poor
  // \todo make it parameterizable
  float score = 0.0f;

  if (rssi_dbm >= -50) {
    score = 100.0f;
  } else if (rssi_dbm >= -70) {
    // Linear interpolation between -50 and -70
    score = 100.0f - (((-50.0f - rssi_dbm) / 20.0f) * 50.0f);
  } else {
    // Linear interpolation between -70 and -90
    score = std::max(0.0f, 50.0f - (((-70.0f - rssi_dbm) / 20.0f) * 50.0f));
  }

  return score;
}

} // namespace analyzer
} // namespace camera
} // namespace eduart
