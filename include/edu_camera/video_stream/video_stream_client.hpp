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
   */
  VideoStreamClient();
  
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

private:
  bool _is_initialized = false;
};

} // namespace video_stream
} // namespace edu_camera

#endif // EDU_CAMERA_VIDEO_STREAM_VIDEO_STREAM_CLIENT_HPP
