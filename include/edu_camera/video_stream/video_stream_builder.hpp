/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/camera/video_camera.hpp"

#include <memory>
#include <string>
#include <cstdint>

namespace eduart {
namespace camera {
namespace video_stream {

class VideoStreamOutput;
class VideoStreamInput;

// This class should help creating different types of stream outputs. It isn't the way I would like to have but it
// works at the moment.
class VideoStreamBuilder
{
public:
  virtual ~VideoStreamBuilder() = default;
  std::unique_ptr<VideoStreamOutput> buildOutput(
    const std::string& pipeline, camera::VideoCamera::Parameter& camera_parameter,
    const std::string& ip_address, const std::uint32_t port);
  std::unique_ptr<VideoStreamInput> buildInput(const std::string& pipeline, const std::uint32_t port);
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
