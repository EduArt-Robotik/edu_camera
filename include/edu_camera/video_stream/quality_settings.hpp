/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

namespace eduart {
namespace camera {
namespace video_stream {

/**
 * @brief Quality settings for adaptive bitrate streaming
 */
struct QualitySettings {
  int bitrate = 5000;      // kbps
  int width = 640;
  int height = 480;
  int fps = 30;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
