/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

struct _GstElement;
typedef struct _GstElement GstElement;

#include <unordered_map>
#include <string>

#include "edu_camera/camera/video_camera.hpp"

namespace eduart {
namespace camera {
namespace video_stream {

class GstreamPipeline
{
public:
  GstreamPipeline(const camera::VideoCamera::Parameter& camera_parameter);
  ~GstreamPipeline();
  
  GstreamPipeline& addVideoConvert(const std::string& name);
  GstreamPipeline& addVideoScale(const std::string& name);
  GstreamPipeline& addEncoderH264(const std::string& name);
  GstreamPipeline& addRtpPayloader(const std::string& name);
  GstreamPipeline& addUdpSink(const std::string& name);
  GstreamPipeline& build();

private:
  std::unordered_map<std::string, GstElement*> _elements;
  GstElement* _pipeline = nullptr;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
