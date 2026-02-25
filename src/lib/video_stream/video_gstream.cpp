#include "edu_camera/video_stream/video_gstream.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include <opencv2/imgproc.hpp>

#include <string>
#include <cstring>

#include <rclcpp/logging.hpp>

namespace eduart {
namespace camera {
namespace video_stream {

VideoGstreamOutput::Parameter VideoGstreamOutput::get_parameter(const Parameter &default_parameter, rclcpp::Node &node)
{
  Parameter parameter = default_parameter;

  node.declare_parameter<int>("pipeline.number_of_elements", 0);
  const int number_of_elements = node.get_parameter("pipeline.number_of_elements").as_int();

  if (number_of_elements <= 0) {
    // no custom pipeline will be created --> use default
    return parameter;
  }

  for (int i = 0; i < number_of_elements; ++i) {
    const std::string prefix = "pipeline.element_" + std::to_string(i);

    node.declare_parameter<std::string>(prefix + ".name", "none");
    node.declare_parameter<std::string>(prefix + ".type", "none");
    
    PipelineElement element;

    element.name = node.get_parameter(prefix + ".name").as_string();
    element.type = node.get_parameter(prefix + ".type").as_string();

    if (element.name == "none" || element.type == "none") {
      RCLCPP_FATAL(
        rclcpp::get_logger("VideoGstreamOutput"),
        "it is required to define 'name' and 'type' for each element! name = '%s', type = '%s'.",
        element.name.c_str(), element.type.c_str()
      );
    }

    // element parameter are fine (or at least defined)
    RCLCPP_INFO(
      rclcpp::get_logger("VideoGstreamOutput"),
      "add pipeline element to parameter. name = '%s', type = '%s', index = %i.",
      element.name.c_str(), element.type.c_str(), i
    );

    parameter.pipeline_elements.emplace_back(std::move(element));
  }

  return parameter;
}

VideoGstreamOutput::VideoGstreamOutput(
  const Parameter& parameter, const camera::VideoCamera::Parameter& camera_parameter, const QualitySettings& quality_settings)
  : VideoStreamOutput(camera_parameter, quality_settings)
  , _parameter(parameter)
{
  // Initialize GStreamer
  initialize();
}

VideoGstreamOutput::~VideoGstreamOutput()
{

}

void VideoGstreamOutput::initialize()
{
  if (_is_initialized) {
    return;
  }

  const auto& settings = getQualitySettings();
  const auto& camera_parameter = getCameraParameter();
  
  GstreamPipelineBuilder builder(camera_parameter, _parameter.input_codec);

  // _pipeline = builder.addVideoConvert("video_converter")
  //                    .addVideoScale("video_scaler")
  //                    .addEncoderH264("encoder", 5000)
  //                    .addRtpPayloader("rtp_payloader")
  //                    .addUdpSink("udp_sink", _parameter.destination, _parameter.port)
  //                    .build();

  for (const auto& element : _parameter.pipeline_elements) {
    if (element.type == "videoconvert") {
      builder.addVideoConvert(element.name);
    } else if (element.type == "videoscale") {
      builder.addVideoScale(element.name);
    } else if (element.type == "encoder_h264") {
      builder.addEncoderH264(element.name, settings.bitrate);
    } else if (element.type == "rtp_payloader") {
      builder.addRtpPayloader(element.name);
    } else if (element.type == "udp_sink") {
      builder.addUdpSink(element.name, _parameter.destination, _parameter.port);
    } else if (element.type == "decoder_mjpeg") {
      builder.addDecoderMJpeg(element.name);
    } else if (element.type == "cap_filter") {
      builder.addCapFilter(element.name, camera_parameter.resolution.width, camera_parameter.resolution.height);
    } else {
      RCLCPP_FATAL(
        rclcpp::get_logger("VideoGstreamOutput"), "unknown pipeline element type '%s' for element '%s'.",
        element.type.c_str(), element.name.c_str()
      );
    }
  }

  _pipeline = builder.build();
  _is_initialized = true;

  RCLCPP_INFO(
    rclcpp::get_logger("VideoGstreamOutput"),
    "GStreamer output pipeline initialized (UDP to %s:%d, bitrate: %d kbps)",
    _parameter.destination.c_str(), _parameter.port, settings.bitrate
  );
}

void VideoGstreamOutput::updateQualitySettings(const QualitySettings& metrics)
{
  if (!_is_initialized) {
    return;
  }
  if (_pipeline == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoGstreamOutput"), "cannot update quality settings, pipeline is not initialized");
    return;
  }

  std::cout << "updating quality settings based on metrics: " << std::endl;
  std::cout << "  bitrate: " << metrics.bitrate << " kbps" << std::endl;
  _pipeline->set("encoder", "bitrate", metrics.bitrate);
}

void VideoGstreamOutput::encodeAndSendFrame(const cv::Mat& frame, const Codec codec)
{
  if (frame.empty()) {
    return;
  }

  // Initialize pipeline on first frame (lazy initialization)
  if (!_is_initialized) {
    throw std::runtime_error("GStreamer pipeline is not initialized.");
  }

  _pipeline->sendFrame(frame, codec);
}


VideoGstreamInput::VideoGstreamInput(int port)
  : _pipeline(nullptr)
  , _port(port)
  , _is_initialized(false)
{
  // Initialize GStreamer
  gst_init(nullptr, nullptr);
}

VideoGstreamInput::~VideoGstreamInput()
{

}

void VideoGstreamInput::initialize()
{
  if (_is_initialized) {
    return;
  }

  _is_initialized = true;
  RCLCPP_INFO(rclcpp::get_logger("VideoGstreamInput"), "GStreamer input pipeline initialized (UDP port %d)", _port);
}

void VideoGstreamInput::receiveFrameAndDecode(cv::Mat& frame)
{
  // Initialize pipeline on first call
  if (!_is_initialized) {
    return;
  }

  
}

} // namespace video_stream
} // namespace camera
} // namespace eduart
