/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/video_stream/video_stream.hpp"
#include "edu_camera/video_stream/video_stream_builder.hpp"

#include "edu_camera/srv/subscribe_to_stream.hpp"
#include "edu_camera/srv/unsubscribe_from_stream.hpp"

namespace eduart {
namespace camera {
namespace video_stream {

/**
 * @brief Network reception statistics
 */
struct ReceptionStats {
  uint64_t frames_received;
  uint64_t frames_dropped;
  float packet_loss;
  int current_buffered_frames;
};

/**
 * @brief VideoStreamClient - Receives video stream and outputs cv::Mat
 */
class VideoStreamClient {
public:
  /**
   * @brief Constructor
   */
  VideoStreamClient(std::unique_ptr<VideoStreamBuilder> builder, rclcpp::Node& node);
  
  /**
   * @brief Destructor
   */
  ~VideoStreamClient();
  
  /**
   * @brief Initialize the client and connect to stream
   * @return true if connection successful
   */
  bool initialize();
  
  /**
   * @brief Connect to the video stream. This will send a request to the server and create a stream input.
   */
  void connect(const std::string& ip_address = "127.0.0.1");
  rclcpp::Client<edu_camera::srv::UnsubscribeFromStream>::SharedFutureAndRequestId disconnect();
  void disconnectAll();
  inline bool isConnected() const { return _stream_input != nullptr; }

  /**
   * @brief Add a new stream builder. This allows to create different types of stream outputs (e.g. RTMP, WebRTC, etc.)
   *        and allows to parameterize them on runtime.
   * @param builder Unique pointer to the stream builder
   */
  void setBuilder(std::unique_ptr<VideoStreamBuilder> builder) {
    _stream_builder = std::move(builder);
  }

  /**
   * @brief Receive a frame from the stream (blocking call until frame is received or timeout is reached)
   * @param frame received frame will be stored here
   * @param codec code of received frame
   * @param timeout maximum time to wait for a frame, 0 means wait indefinitely
   * @return true if frame was received, false if timeout was reached or an error occurred
   */
  bool receiveFrame(
    cv::Mat& frame, Codec& codec, const std::chrono::nanoseconds timeout = std::chrono::nanoseconds(0));

  /**
   * @brief Shutdown the client and disconnect from stream
   */
  void shutdown();

private:
  rclcpp::Node& _node;
  bool _is_initialized = false;
  std::unique_ptr<VideoStreamBuilder> _stream_builder;
  std::shared_ptr<rclcpp::Client<edu_camera::srv::SubscribeToStream>> _client_subscribe_to_stream;
  std::shared_ptr<rclcpp::Client<edu_camera::srv::UnsubscribeFromStream>> _client_unsubscribe_from_stream;
  std::unique_ptr<VideoStreamInput> _stream_input;
  std::uint64_t _output_id = 0; // id of the stream output on the server
  std::mutex _mutex_creating_input;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
