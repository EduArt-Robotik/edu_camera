#include "edu_camera/video_stream/video_stream_builder.hpp"

#include "edu_camera/video_stream/video_gstream.hpp"

namespace eduart {
namespace camera {
namespace video_stream {

std::unique_ptr<VideoStreamOutput> VideoStreamBuilder::buildOutput(
  const std::string& pipeline, camera::VideoCamera::Parameter& camera_parameter,
  const std::string& ip_address, const std::uint32_t port)
{
  if (pipeline == "gstream_udp") {
    // constructs a sender pipeline for sending video frames via RTP and H264 codec. 
    const std::vector<VideoGstreamOutput::PipelineElement> video_convert_element = {
      {"videoconvert", "videoconvert"},
      // {"videoscale"  , "videoscale"},
      // {"capfilter"   , "cap_filter"},
      {"encoder"     , "encoder_h264"},
      {"payloader"   , "rtp_payloader"},
      {"sink"        , "udp_sink"}
    };
    const VideoGstreamOutput::Parameter parameter = {
      .destination = ip_address,
      .port = port,
      .pipeline_elements = video_convert_element
    };

    return std::make_unique<VideoGstreamOutput>(parameter, camera_parameter, QualitySettings{});
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("VideoStreamBuilder"), "unknown pipeline type '%s'", pipeline.c_str());
    return nullptr;
  }
}

std::unique_ptr<VideoStreamInput> VideoStreamBuilder::buildInput(const std::string& pipeline, const std::uint32_t port)
{
  if (pipeline == "gstream_udp") {
    // construct a receiver pipeline that fits to the sender pipeline
    VideoGstreamInput::Parameter parameter {
      .port = port,
      .codec = Codec(Codec::Type::BGR),
      .pipeline_elements = {
        {"depayloader", "rtp_h264_depayloader"},
        {"decoder", "decoder_h264"},
        {"videoconvert", "videoconvert"},
        {"sink", "appsink"}
      }
    };
    return std::make_unique<VideoGstreamInput>(parameter);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("VideoStreamBuilder"), "unknown pipeline type '%s'", pipeline.c_str());
    return nullptr;
  }
}

} // namespace video_stream
} // namespace camera
} // namespace eduart
