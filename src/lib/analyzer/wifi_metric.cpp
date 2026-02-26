#include "edu_camera/analyzer/wifi_metric.hpp"

namespace eduart {
namespace camera {
namespace analyzer {

WifiMetric::WifiMetric(const float scale)
  : _scale(scale)
{

}

float WifiMetric::score() const {
  std::lock_guard<std::mutex> lock(_mutex);
  return _score;
}

void WifiMetric::update(const float score) {
  std::lock_guard<std::mutex> lock(_mutex);
  _score = score * _scale;
}

} // namespace analyzer
} // namespace camera
} // namespace eduart
