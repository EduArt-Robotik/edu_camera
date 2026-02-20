/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/video_stream/quality_settings.hpp"

#include <opencv2/core/mat.hpp>

namespace eduart {
namespace camera {
namespace camera {

class VideoCamera
{
public:
  VideoCamera() = default;
  virtual ~VideoCamera() = default;

  virtual bool open(const video_stream::QualitySettings& settings) = 0;
  virtual void close() = 0;
  virtual cv::Mat captureFrame() = 0;
};

} // end namespace camera
} // end namespace camera
} // end namespace eduart