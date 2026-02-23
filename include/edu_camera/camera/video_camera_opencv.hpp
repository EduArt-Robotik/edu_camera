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
  struct Parameter : public camera::VideoCamera::Parameter {
    int device_id = 0;
  };

  VideoCameraOpenCV(const Parameter& parameter);
  ~VideoCameraOpenCV() override;

  bool open() override;
  void close() override;
  cv::Mat captureFrame() override;

  static Parameter get_parameter(const Parameter& default_parameter, rclcpp::Node& ros_node);

private:
  const Parameter _parameter;
  cv::VideoCapture _camera_device;
};

} // end namespace camera
} // end namespace camera
} // end namespace eduart
