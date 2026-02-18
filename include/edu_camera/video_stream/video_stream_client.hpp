#ifndef EDU_CAMERA_VIDEO_STREAM_VIDEO_STREAM_CLIENT_HPP
#define EDU_CAMERA_VIDEO_STREAM_VIDEO_STREAM_CLIENT_HPP

#include <opencv2/opencv.hpp>
#include <memory>
#include <thread>
#include <atomic>
#include <queue>
#include <mutex>
#include <condition_variable>

namespace edu_camera {
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
   * @param stream_url RTMP/HTTP stream URL to connect to
   * @param buffer_size Maximum frames to buffer (default: 10)
   */
  VideoStreamClient(const std::string& stream_url, size_t buffer_size = 10);
  
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
   * @brief Shutdown the client and disconnect from stream
   */
  void shutdown();
  
  /**
   * @brief Get the latest available frame
   * @param frame Output cv::Mat frame (non-blocking, returns latest or empty)
   * @return true if a new frame was available
   */
  bool get_frame(cv::Mat& frame);
  
  /**
   * @brief Get frame with timeout (blocking)
   * @param frame Output cv::Mat frame
   * @param timeout_ms Timeout in milliseconds
   * @return true if frame was received within timeout
   */
  bool get_frame_timeout(cv::Mat& frame, int timeout_ms);
  
  /**
   * @brief Check if client is connected to stream
   * @return true if actively receiving stream
   */
  bool is_connected() const;
  
  /**
   * @brief Get reception statistics
   * @return Reception statistics
   */
  ReceptionStats get_stats() const;
  
  /**
   * @brief Clear the buffered frames (useful for seeking/sync)
   */
  void clear_buffer();
  
  /**
   * @brief Get current buffer fill level
   * @return Number of buffered frames
   */
  size_t get_buffer_size() const;

private:
  // Stream reception thread
  void reception_worker_();
  bool decode_frame_(const std::vector<uint8_t>& data, cv::Mat& frame);
  
  std::string stream_url_;
  bool is_initialized_;
  std::atomic<bool> should_shutdown_;
  
  // Frame buffer
  std::queue<cv::Mat> frame_buffer_;
  mutable std::mutex buffer_mutex_;
  std::condition_variable buffer_cv_;
  size_t buffer_size_;
  
  // RTMP connection
  void* stream_handle_; // RTMP/stream handle
  bool is_connected_;
  
  // Reception thread
  std::unique_ptr<std::thread> reception_thread_;
  
  // Statistics
  mutable std::mutex stats_mutex_;
  uint64_t frames_received_;
  uint64_t frames_dropped_;
  float packet_loss_;
};

} // namespace video_stream
} // namespace edu_camera

#endif // EDU_CAMERA_VIDEO_STREAM_VIDEO_STREAM_CLIENT_HPP
