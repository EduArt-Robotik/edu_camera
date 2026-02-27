#include "edu_camera/video_stream/video_stream_server.hpp"

#include <rclcpp/logging.hpp>

#include <iostream>
#include <cmath>

namespace eduart {
namespace camera {
namespace video_stream {

VideoStreamServer::VideoStreamServer()
{

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
  // \todo implement actual client subscription logic
  (void)request;
  (void)response;

  const auto search = _stream_output.find(request->pipeline);

  if (search != _stream_output.end()) {
    RCLCPP_INFO(
      rclcpp::get_logger("VideoStreamServer"), "Client subscribed to stream: %s", request->pipeline.c_str()
    );
    response->success = true;
    response->message = "Subscribed to stream: " + request->pipeline;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("VideoStreamServer"),
      "Stream not found for client subscription: %s", request->pipeline.c_str()
    );
    response->success = false;
    response->message = "Stream not found: " + request->pipeline;
  }
}

} // namespace video_stream
} // namespace camera
} // namespace eduart
