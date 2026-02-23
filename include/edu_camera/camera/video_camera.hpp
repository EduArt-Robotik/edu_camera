/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/video_stream/quality_settings.hpp"

#include <edu_camera/video_stream/codec.hpp>

#include <opencv2/core/mat.hpp>

#include <rclcpp/node.hpp>

namespace eduart {
namespace camera {
namespace camera {

class VideoCamera
{
public:
  struct Parameter {
    cv::Size2i resolution{800, 600};
    float fps = 30.0f;
    video_stream::Codec codec = video_stream::Codec::Type::MJPEG;
  };

  VideoCamera() = default;
  virtual ~VideoCamera() = default;

  virtual bool open() = 0;
  virtual void close() = 0;
  virtual cv::Mat captureFrame() = 0;

  static Parameter get_parameter(const Parameter& default_parameter, rclcpp::Node& ros_node);
};

} // end namespace camera
} // end namespace camera
} // end namespace eduart