#include "edu_camera/analyzer/wifi_quality_monitor.hpp"

namespace eduart {
namespace camera {
namespace analyzer {

WifiQualityMonitor::WifiQualityMonitor()
{

}

void WifiQualityMonitor::addMetric(const std::shared_ptr<WifiMetric>& metric)
{
  _metrics.push_back(metric);
}

float WifiQualityMonitor::overallScore() const
{
  if (_metrics.empty()) {
    return 0.0f;
  }

  // Average the scores of all metrics
  float normalizer = 0.0f;
  float total_score = 0.0f;

  for (const auto& metric : _metrics) {
    normalizer += metric->scale();
    total_score += metric->score();
  }
  
  // Normalize to 0-100 scale
  return total_score / normalizer;
}

} // namespace analyzer
} // namespace camera
} // namespace eduart
