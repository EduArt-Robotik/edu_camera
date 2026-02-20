/**
 * Copyright EduArt Robotik GmbH 2022
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

namespace eduart {
namespace camera {
namespace video_stream {

/**
 * @brief Network metrics for adaptive streaming
 */
struct NetworkMetrics {
  int latency_ms;     // RTT in milliseconds
  float packet_loss;  // 0.0 - 1.0
  int available_bandwidth; // kbps estimated
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
