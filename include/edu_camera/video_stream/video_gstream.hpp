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
    int port = 5000;
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
  VideoGstreamInput(int port = 5000);
  ~VideoGstreamInput() override;

  void receiveFrameAndDecode(cv::Mat& frame) override;

private:
  void initialize();

  std::unique_ptr<GstreamPipeline> _pipeline;
  
  int _port = 5000;
  bool _is_initialized = false;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
