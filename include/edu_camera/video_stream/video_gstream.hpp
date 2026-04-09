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
    Codec input_codec = Codec(Codec::Type::BGR);
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
  VideoGstreamInput(const std::uint32_t port = 5000);
  ~VideoGstreamInput() override;

  void receiveFrameAndDecode(cv::Mat& frame) override;

private:
  void initialize();

  std::unique_ptr<GstreamPipeline> _pipeline;
  
  std::uint32_t _port = 5000;
  bool _is_initialized = false;
};


class VideoGstreamBuilder : public VideoStreamBuilder
{
public:
  VideoGstreamBuilder(
    const VideoGstreamOutput::Parameter& parameter, const camera::VideoCamera::Parameter& camera_parameter,
    const QualitySettings& quality_settings);
  ~VideoGstreamBuilder() override = default;

  std::unique_ptr<VideoStreamOutput> buildOutput(const std::string& ip_address, const std::uint32_t port) override;
  std::unique_ptr<VideoStreamInput> buildInput(const std::uint32_t port) override;

private:
  VideoGstreamOutput::Parameter _parameter;
  camera::VideoCamera::Parameter _camera_parameter;
  QualitySettings _quality_settings;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
