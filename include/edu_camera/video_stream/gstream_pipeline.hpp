/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <gst/gst.h>

#include <vector>

namespace eduart {
namespace camera {
namespace video_stream {

class GstreamPipeline
{
public:
  GstreamPipeline();
  ~GstreamPipeline();

  

private:
  std::vector<GstElement*> _elements;
  GstElement* _pipeline = nullptr;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
