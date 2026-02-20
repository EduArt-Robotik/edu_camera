/**
 * Copyright EduArt Robotik GmbH 2022
 * This class is copied from https://github.com/franc0r/libfrancor/blob/master/francor_base/include/francor_base/angle.h
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include <opencv2/opencv.hpp>

#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace eduart {
namespace camera {
namespace video_stream {

/**
 * @brief Quality settings for adaptive bitrate streaming
 */
struct QualitySettings {
  int bitrate;      // kbps
  int width;
  int height;
  int fps;
};

/**
 * @brief Network metrics for adaptive streaming
 */
struct NetworkMetrics {
  int latency_ms;     // RTT in milliseconds
  float packet_loss;  // 0.0 - 1.0
  int available_bandwidth; // kbps estimated
};

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
  virtual bool initialize() = 0;
  
  /**
   * @brief Shutdown the streamer
   */
  virtual void shutdown() = 0;
  
  /**
   * @brief Send a frame to the stream
   * @param frame cv::Mat frame to send
   * @return true if frame was sent successfully
   */
  // \todo what is with a codec frame?
  bool send_frame(const cv::Mat& frame);
  
  /**
   * @brief Update network metrics for adaptive adjustment
   * @param metrics Network metrics (latency, packet loss, bandwidth)
   */
  void update_network_metrics(const NetworkMetrics& metrics);
  
  /**
   * @brief Get current quality settings
   * @return Current quality settings
   */
  QualitySettings get_quality_settings() const;
  
  /**
   * @brief Set manual quality preset
   * @param bitrate Bitrate in kbps
   * @param width Frame width
   * @param height Frame height
   * @param fps Frames per second
   */
  void set_quality_manual(int bitrate, int width, int height, int fps);
  
  /**
   * @brief Check if streamer is connected
   * @return true if connected to RTMP server
   */
  bool is_connected() const;
  
  /**
   * @brief Get current statistics
   * @return Number of frames sent
   */
  uint64_t get_frames_sent() const { return frames_sent_; }

private:
  // Network monitoring and adaptation
  void network_monitor_worker_();
  QualitySettings calculate_quality_(const NetworkMetrics& metrics);
  void update_encoder_bitrate_(int bitrate);
  
  // Encoding
  void encoding_worker_();
  bool encode_frame_(const cv::Mat& frame, std::vector<uint8_t>& encoded_data);

  std::string rtmp_url_;
  bool is_initialized_;
  std::atomic<bool> should_shutdown_;
  
  // Quality management
  mutable std::mutex quality_mutex_;
  QualitySettings current_quality_;
  NetworkMetrics last_metrics_;
  
  // Frame queue
  std::queue<cv::Mat> frame_queue_;
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  const size_t max_queue_size_ = 5;
  
  // RTMP connection
  void* rtmp_handle_; // RTMP connection handle
  bool is_connected_;
  
  // Threads
  std::unique_ptr<std::thread> encoding_thread_;
  std::unique_ptr<std::thread> network_monitor_thread_;
  
  // Statistics
  uint64_t frames_sent_;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
