/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/video_stream/network_metric.hpp"
#include "edu_camera/video_stream/quality_settings.hpp"
#include "edu_camera/video_stream/video_stream.hpp"
#include "edu_camera/video_stream/video_stream_builder.hpp"

#include "edu_camera/srv/subscribe_to_stream.hpp"
#include "edu_camera/srv/unsubscribe_from_stream.hpp"

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
  VideoStreamServer(
    const camera::VideoCamera::Parameter& camera_parameter, std::unique_ptr<VideoStreamBuilder> builder,
    rclcpp::Node& node);
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

   * @brief Add a new stream builder. This allows to create different types of stream outputs (e.g. RTMP, WebRTC, etc.)
   *        and allows to parameterize them on runtime.
   * @param builder Unique pointer to the stream builder
   */
  void setBuilder(std::unique_ptr<VideoStreamBuilder> builder) {
    _stream_builder = std::move(builder);
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
  void removeStreamClient(
    edu_camera::srv::UnsubscribeFromStream::Request::SharedPtr request,
    edu_camera::srv::UnsubscribeFromStream::Response::SharedPtr response);

  std::unordered_map<std::size_t, std::unique_ptr<VideoStreamOutput>> _stream_output;
  camera::VideoCamera::Parameter _camera_parameter;
  std::unique_ptr<VideoStreamBuilder> _stream_builder;
  QualitySettings _quality_settings;

  std::shared_ptr<rclcpp::Service<edu_camera::srv::SubscribeToStream>> _service_subscribe_to_stream;
  std::shared_ptr<rclcpp::Service<edu_camera::srv::UnsubscribeFromStream>> _service_unsubscribe_from_stream;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
