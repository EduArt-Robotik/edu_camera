/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/analyzer/wifi_metric.hpp"

#include <thread>
#include <atomic>
#include <string>

namespace eduart {
namespace camera {
namespace analyzer {

class MetricLinkQuality : public WifiMetric
{
public:
  MetricLinkQuality(
    const float scale = 1.0f, const std::string& interface = "wlan0",
    const std::chrono::milliseconds& measurement_interval = std::chrono::milliseconds(1000));
  ~MetricLinkQuality();

private:
  void measurementLoop();
  float calculateLinkQualityScore();

  std::thread _measurement_thread;
  std::chrono::milliseconds _measurement_interval{1000};
  std::atomic<bool> _running{true};
  std::string _interface;
  int _sock{-1};
};

} // namespace analyzer
} // namespace camera
} // namespace eduart
