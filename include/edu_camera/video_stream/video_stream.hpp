/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/video_stream/quality_settings.hpp"
#include "edu_camera/video_stream/codec.hpp"

#include <edu_camera/camera/video_camera.hpp>

#include <opencv2/core/mat.hpp>

namespace eduart {
namespace camera {
namespace video_stream {

class VideoStreamOutput
{
public:
  /**
   * @brief Construct a new Video Stream Output object
   * @param camera_parameter Camera parameters used to specify the source video quality.
   */
  VideoStreamOutput(const camera::VideoCamera::Parameter& camera_parameter, const QualitySettings& quality_settings)
    : _camera_parameter(camera_parameter)
    , _video_stream_quality(quality_settings)
  { }
  virtual ~VideoStreamOutput() = default;

  /**
   * @brief Set the quality settings for the video stream output. This can be used for adaptive streaming based on
   *        network conditions.
   * @param settings The quality settings to apply.
   */
  inline void setQualitySettings(const QualitySettings& settings) {
    _video_stream_quality = settings;
  }
  inline QualitySettings getQualitySettings() const {
    return _video_stream_quality;
  }
  inline camera::VideoCamera::Parameter getCameraParameter() const {
    return _camera_parameter;
  }
  virtual void encodeAndSendFrame(const cv::Mat& frame, const Codec codec) = 0;

protected:
  virtual void updateQualitySettings(const QualitySettings& metrics) = 0;

private:
  const camera::VideoCamera::Parameter _camera_parameter;
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
