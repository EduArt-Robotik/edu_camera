/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/video_stream/video_stream.hpp"
#include "edu_camera/video_stream/gstream_pipeline.hpp"

#include <string>

namespace eduart {
namespace camera {
namespace video_stream {

class VideoGstreamOutput : public VideoStreamOutput
{
public:
  struct PipelineElement {
    std::string name;
    std::string type;
  };

  struct Parameter {
    std::string destination = "127.0.0.1";
    std::uint32_t port = 5000;
    std::vector<PipelineElement> pipeline_elements;
  };

  VideoGstreamOutput(
    const Parameter& parameter, const camera::VideoCamera::Parameter& camera_parameter, const QualitySettings& quality_settings);
  ~VideoGstreamOutput() override;

  void encodeAndSendFrame(const cv::Mat& frame, const Codec codec) override;

  static Parameter get_parameter(const Parameter& default_parameter, rclcpp::Node& node);

private:
  void initialize();
  void updateQualitySettings(const QualitySettings& metrics) override;

  std::unique_ptr<GstreamPipeline> _pipeline;
  
  const Parameter _parameter;
  bool _is_initialized = false;
};


class VideoGstreamInput : public VideoStreamInput
{
public:
  struct PipelineElement {
    std::string name;
    std::string type;
  };

  struct Parameter {
    std::uint32_t port = 5000;
    Codec codec{Codec::Type::BGR};              // codec must fit to sender pipeline
    std::vector<PipelineElement> pipeline_elements;  // pipeline elements must fit to sender pipeline and codec
  };

  VideoGstreamInput(const Parameter& parameter);
  ~VideoGstreamInput() override;

  void receiveFrameAndDecode(cv::Mat& frame, Codec& codec, const std::chrono::nanoseconds timeout) override;

private:
  void initialize();

  const Parameter _parameter;
  std::unique_ptr<GstreamPipeline> _pipeline;  
  bool _is_initialized = false;
};


} // namespace video_stream
} // namespace camera
} // namespace eduart
