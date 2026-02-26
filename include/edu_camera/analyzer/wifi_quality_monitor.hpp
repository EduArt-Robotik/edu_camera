/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/analyzer/wifi_metric.hpp"

#include <vector>
#include <memory>

namespace eduart {
namespace camera {
namespace analyzer {

class WifiQualityMonitor
{
public:
  WifiQualityMonitor();
  
  void addMetric(const std::shared_ptr<WifiMetric>& metric);
  float overallScore() const;

private:
  std::vector<std::shared_ptr<WifiMetric>> _metrics;
};

} // namespace analyzer
} // namespace camera
} // namespace eduart
