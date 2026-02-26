#include "edu_camera/analyzer/metric_snr.hpp"

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

MetricSnr::MetricSnr(
  const float scale, const std::string& interface, const std::chrono::milliseconds& measurement_interval)
  : WifiMetric(scale)
  , _measurement_interval(measurement_interval)
  , _interface(interface)
{
  // Open socket once for reuse
  _sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (_sock < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MetricSnr"), "failed to open socket for SNR measurement");
  }
  
  // Start background measurement thread
  _measurement_thread = std::thread(&MetricSnr::measurementLoop, this);
}

MetricSnr::~MetricSnr()
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

void MetricSnr::measurementLoop()
{
  auto stamp_last_measurement = std::chrono::steady_clock::now();

  while (_running) {
    // Calculate new score
    const float new_score = calculateSnrScore();
    RCLCPP_INFO(rclcpp::get_logger("MetricSnr"), "SNR Score: %.2f", new_score);
    
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

float MetricSnr::calculateSnrScore()
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
      rclcpp::get_logger("MetricSnr"), "failed to get wireless stats for interface '%s'", _interface.c_str()
    );
    return 0.0f; // Error getting wireless stats
  }

  // Calculate SNR (Signal-to-Noise Ratio) in dB
  // SNR = Signal Level - Noise Level
  const int8_t signal_dbm = stats.qual.level - 256;
  const int8_t noise_dbm = stats.qual.noise - 256;
  const float snr_db = signal_dbm - noise_dbm;

  // Normalize SNR to 0-100 score
  // SNR ranges: > 40 dB = excellent, 25-40 dB = good, < 25 dB = poor
  float score = 0.0f;

  if (snr_db >= 40.0f) {
    score = 100.0f;
  } else if (snr_db >= 25.0f) {
    // Linear interpolation between 25 and 40 dB
    score = 50.0f + ((snr_db - 25.0f) / 15.0f) * 50.0f;
  } else if (snr_db >= 10.0f) {
    // Linear interpolation between 10 and 25 dB
    score = ((snr_db - 10.0f) / 15.0f) * 50.0f;
  } else {
    score = 0.0f;
  }

  return score;
}

} // namespace analyzer
} // namespace camera
} // namespace eduart
