#include "edu_camera/video_stream/video_stream_server.hpp"

#include <rclcpp/logging.hpp>

#include <cmath>

namespace eduart {
namespace camera {
namespace video_stream {

VideoStreamServer::VideoStreamServer(
  const camera::VideoCamera::Parameter& camera_parameter, std::unique_ptr<VideoStreamBuilder> builder, rclcpp::Node& node)
  : _camera_parameter(camera_parameter)
  , _stream_builder(std::move(builder))
{
  _service_subscribe_to_stream = node.create_service<edu_camera::srv::SubscribeToStream>(
    "subscribe_to_stream", 
    std::bind(&VideoStreamServer::addStreamClient, this, std::placeholders::_1, std::placeholders::_2)
  );
  _service_unsubscribe_from_stream = node.create_service<edu_camera::srv::UnsubscribeFromStream>(
    "unsubscribe_from_stream", 
    std::bind(&VideoStreamServer::removeStreamClient, this, std::placeholders::_1, std::placeholders::_2)
  );
}

VideoStreamServer::~VideoStreamServer()
{

}

bool VideoStreamServer::initialize()
{
  return true; // \todo implement actual initialization
}

void VideoStreamServer::shutdown()
{
  // \todo implement actual shutdown
}

bool VideoStreamServer::sendFrame(const cv::Mat& frame, const Codec codec)
{
  if (frame.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("VideoStreamServer"), "Empty frame, skipping");
    return false;
  }
  
  // Send frame to output
  for (auto& [name, output] : _stream_output) {
    output->encodeAndSendFrame(frame, codec);
  }
  
  return true;
}

void VideoStreamServer::updateNetworkMetrics(const NetworkMetrics& metrics)
{
  // Calculate new quality settings based on network metrics
  // \todo implement actual adaptation logic
  (void)metrics;

  // _stream_output->setQualitySettings({});
}

void VideoStreamServer::setQualityManual(int bitrate, int width, int height, int fps)
{
  QualitySettings settings;

  settings.bitrate = bitrate;
  settings.width = width;
  settings.height = height;
  settings.fps = fps;

  for (auto& [name, output] : _stream_output) {
    output->setQualitySettings(settings);
  }
}

// bool VideoStreamServer::isConnected() const
// {
//   return true; // \todo implement actual connection check
// }

QualitySettings calculate_quality_(const NetworkMetrics& metrics)
{
  QualitySettings qs;// = current_quality_;
  
  // Adaptation thresholds
  const int HIGH_LATENCY_MS = 100;
  const int MEDIUM_LATENCY_MS = 50;
  const float HIGH_PACKET_LOSS = 0.05f; // 5%
  const float MEDIUM_PACKET_LOSS = 0.02f; // 2%
  
  // Calculate stress factor (0.0 = optimal, 1.0+ = bad)
  float stress = 0.0f;
  
  if (metrics.latency_ms > HIGH_LATENCY_MS) {
    stress += (metrics.latency_ms - HIGH_LATENCY_MS) / 50.0f;
  } else if (metrics.latency_ms > MEDIUM_LATENCY_MS) {
    stress += 0.3f;
  }
  
  if (metrics.packet_loss > HIGH_PACKET_LOSS) {
    stress += (metrics.packet_loss - HIGH_PACKET_LOSS) / 0.05f;
  } else if (metrics.packet_loss > MEDIUM_PACKET_LOSS) {
    stress += 0.5f;
  }
  
  // Apply quality adjustments based on stress
  if (stress > 1.5f) {
    // Very bad conditions
    qs.bitrate = 800;
    qs.width = 480;
    qs.height = 360;
    qs.fps = 10;
  } else if (stress > 1.0f) {
    // Bad conditions
    qs.bitrate = 1200;
    qs.width = 640;
    qs.height = 480;
    qs.fps = 15;
  } else if (stress > 0.5f) {
    // Medium conditions
    qs.bitrate = 2000;
    qs.width = 854;
    qs.height = 480;
    qs.fps = 24;
  } else if (stress > 0.2f) {
    // Good conditions
    qs.bitrate = 3500;
    qs.width = 1280;
    qs.height = 720;
    qs.fps = 30;
  } else {
    // Excellent conditions
    qs.bitrate = 5000;
    qs.width = 1280;
    qs.height = 720;
    qs.fps = 30;
  }
  
  return qs;
}

void VideoStreamServer::addStreamClient(
  edu_camera::srv::SubscribeToStream::Request::SharedPtr request, 
  edu_camera::srv::SubscribeToStream::Response::SharedPtr response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("VideoStreamServer"), 
    "Received subscription request for ip = %s, port = %d, pipeline = %s",
    request->ip.c_str(), request->port, request->pipeline.c_str()
  );

  // first create hash for identifying stream output
  const std::string hash_string = request->pipeline + request->ip + std::to_string(request->port);
  const std::size_t stream_hash = std::hash<std::string>{}(hash_string);

  const auto search_stream = _stream_output.find(stream_hash);

  if (search_stream != _stream_output.end()) {
    // Client already subscribed to this stream output
    RCLCPP_ERROR(rclcpp::get_logger("VideoStreamServer"), "client already subscribed to stream: %zx", stream_hash);
    response->success = false;
    response->message = "Client already subscribed to stream: " + request->pipeline;
  } else {
    // Builder found and client not subscribed yet --> create new stream output
    auto output = _stream_builder->buildOutput(request->pipeline, _camera_parameter, request->ip, request->port);

    if (output) {
      // stream output successfully created
      _stream_output[stream_hash] = std::move(output);
      response->output_id = stream_hash;
      response->success = true;
      response->message = "Created stream output " + std::to_string(stream_hash) + ", for ip = " + request->ip + ", port = " 
        + std::to_string(request->port) + ", pipeline = " + request->pipeline;
    } else {
      response->success = false;
      response->message = "Failed to create stream output for: " + request->pipeline;
    }
  }
}

void VideoStreamServer::removeStreamClient(
  edu_camera::srv::UnsubscribeFromStream::Request::SharedPtr request, 
  edu_camera::srv::UnsubscribeFromStream::Response::SharedPtr response)
{
  RCLCPP_INFO(
    rclcpp::get_logger("VideoStreamServer"),
    "Received unsubscription request for output_id = %zx", request->output_id
  );

  // first create hash for identifying stream output
  const auto search_stream = _stream_output.find(request->output_id);

  if (search_stream != _stream_output.end()) {
    // Stream output found --> remove it
    _stream_output.erase(search_stream);
    response->success = true;
    response->message = "Removed stream output " + std::to_string(request->output_id) + ".";
  } else {
    // Stream output not found
    RCLCPP_ERROR(
      rclcpp::get_logger("VideoStreamServer"), "client not subscribed to stream: %zx", request->output_id
    );
    response->success = false;
    response->message = "Client not subscribed to stream: " + std::to_string(request->output_id);
  }
}

} // namespace video_stream
} // namespace camera
} // namespace eduart
