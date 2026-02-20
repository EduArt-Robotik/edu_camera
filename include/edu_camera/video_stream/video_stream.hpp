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
namespace video_stream {

class VideoStreamOutput
{
public:
  virtual ~VideoStreamOutput() = default;

  inline void setQualitySettings(const QualitySettings& settings) {
    _video_stream_quality = settings;
  }
  inline QualitySettings getQualitySettings() const {
    return _video_stream_quality;
  }
  virtual void encodeAndSendFrame(const cv::Mat& frame) = 0;

private:
  QualitySettings _video_stream_quality;
};


class VideoStreamInput
{
public:
  virtual ~VideoStreamInput() = default;

  virtual void receiveFrameAndDecode(cv::Mat& frame) = 0;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
