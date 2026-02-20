/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/camera/video_camera.hpp"

#include <opencv2/videoio.hpp>

namespace eduart {
namespace camera {
namespace camera {

class VideoCameraOpenCV : public camera::VideoCamera
{
public:
  VideoCameraOpenCV(int device_id = 0);
  ~VideoCameraOpenCV() override;

  bool open(const video_stream::QualitySettings& settings) override;
  void close() override;
  cv::Mat captureFrame() override;

private:
  int _device_id;
  cv::VideoCapture _camera_device;
};

} // end namespace camera
} // end namespace camera
} // end namespace eduart
