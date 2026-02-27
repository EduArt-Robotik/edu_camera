/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/video_stream/network_metric.hpp"
#include "edu_camera/video_stream/quality_settings.hpp"
#include "edu_camera/video_stream/video_stream.hpp"
#include "edu_camera/srv/subscribe_to_stream.hpp"

#include <memory>

namespace eduart {
namespace camera {
namespace video_stream {

/**
 * @brief Defines a interface for a video streaming server. And it provides basic functionality like the
 *        adaptive metric handling.
 */
class VideoStreamServer
{
public:
  VideoStreamServer();
  virtual ~VideoStreamServer();
  
  /**
   * @brief Initialize the streamer
   * @return true if initialization successful
   */
  bool initialize();
  
  /**
   * @brief Shutdown the streamer
   */
  void shutdown();
  
  /**
   * @brief Send a frame to the stream
   * @param frame cv::Mat frame to send
   * @param codec Codec of the frame
   * @return true if frame was sent successfully
   */
  // \todo what is with a codec frame?
  bool sendFrame(const cv::Mat& frame, const Codec codec);

  /**
   * @brief Add a new stream output. This allows to send the same frame to multiple outputs (e.g. RTMP, WebRTC, etc.)
   * @param name Name of the stream output
   * @param output Unique pointer to the stream output
   */
  inline void addStreamOutput(const std::string& name, std::unique_ptr<VideoStreamOutput> output) {
    _stream_output[name] = std::move(output);
  }

  /**
   * @brief Update network metrics for adaptive adjustment
   * @param metrics Network metrics (latency, packet loss, bandwidth)
   */
  void updateNetworkMetrics(const NetworkMetrics& metrics);
  
  /**
   * @brief Get current quality settings
   * @return Current quality settings
   */
  inline QualitySettings getQualitySettings() const { return _quality_settings; }
  
  /**
   * @brief Set manual quality preset
   * @param bitrate Bitrate in kbps
   * @param width Frame width
   * @param height Frame height
   * @param fps Frames per second
   */
  void setQualityManual(int bitrate, int width, int height, int fps);
  
  // /**
  //  * @brief Check if streamer is connected
  //  * @return true if connected to RTMP server
  //  */
  // bool isConnected() const;
  
  /**
   * @brief Get current statistics
   * @return Number of frames sent
   */
  // uint64_t getFramesSent() const { return frames_sent_; }

private:
  void addStreamClient(
    edu_camera::srv::SubscribeToStream::Request::SharedPtr request,
    edu_camera::srv::SubscribeToStream::Response::SharedPtr response);

  std::unordered_map<std::string, std::unique_ptr<VideoStreamOutput>> _stream_output;
  QualitySettings _quality_settings;

  std::shared_ptr<rclcpp::Service<edu_camera::srv::SubscribeToStream>> _sub_scribe_service;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
