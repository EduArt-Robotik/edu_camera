/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <mutex>

namespace eduart {
namespace camera {
namespace analyzer {

class WifiMetric
{
public:
  WifiMetric(const float scale = 1.0f);
  virtual ~WifiMetric() = default;

  float score() const;
  inline float scale() const { return _scale; }

protected:
  void update(const float score);

private:
  mutable std::mutex _mutex; //> used by update to allow thread safe updates of the score
  float _score = 0.0f;
  float _scale = 1.0f;
};

} // namespace analyzer
} // namespace camera
} // namespace eduart
