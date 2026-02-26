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
#include <deque>

namespace eduart {
namespace camera {
namespace analyzer {

class MetricPacketLoss : public WifiMetric
{
public:
  MetricPacketLoss(
    const float scale = 1.0f, const std::string& target_host = "8.8.8.8",
    const std::chrono::milliseconds& measurement_interval = std::chrono::milliseconds(1000));
  ~MetricPacketLoss();

private:
  void measurementLoop();
  float calculatePacketLossScore();
  bool sendPing();

  std::thread _measurement_thread;
  std::chrono::milliseconds _measurement_interval{1000};
  std::atomic<bool> _running{true};
  std::string _target_host;
  int _sock{-1};
  uint16_t _sequence{0};
  
  // Ping configuration
  static constexpr std::size_t _pings_per_measurement{10}; //> number of pings to send per measurement cycle
  static constexpr std::size_t _sample_size{100}; //> 100 needed for 1% resolution in packet loss score calculation
  
  // Rolling window for packet loss calculation
  std::deque<bool> _ping_results;
  std::mutex _results_mutex;
};

} // namespace analyzer
} // namespace camera
} // namespace eduart
