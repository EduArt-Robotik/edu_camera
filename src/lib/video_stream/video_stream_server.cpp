#include "edu_camera/video_stream/video_stream_server.hpp"

#include <iostream>
#include <chrono>
#include <cmath>

namespace eduart {
namespace camera {
namespace video_stream {

VideoStreamServer::VideoStreamServer(std::unique_ptr<VideoStreamOutput> output)
  : _stream_output(std::move(output))
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
    std::cerr << "[VideoStreamServer] Warning: Empty frame, skipping" << std::endl;
    return false;
  }
  
  // Send frame to output
  _stream_output->encodeAndSendFrame(frame, codec);
  
  return true;
}

void VideoStreamServer::updateNetworkMetrics(const NetworkMetrics& metrics)
{
  // Calculate new quality settings based on network metrics
  // \todo implement actual adaptation logic
  (void)metrics;

  _stream_output->setQualitySettings({});
}

void VideoStreamServer::setQualityManual(int bitrate, int width, int height, int fps)
{
  QualitySettings settings;

  settings.bitrate = bitrate;
  settings.width = width;
  settings.height = height;
  settings.fps = fps;

  _stream_output->setQualitySettings(settings);  
}

bool VideoStreamServer::isConnected() const
{
  return true; // \todo implement actual connection check
}

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



// void VideoStreamServer::encoding_worker_()
// {
//   std::cout << "[VideoStreamServer] Encoding thread started" << std::endl;
  
//   while (!should_shutdown_) {
//     cv::Mat frame;
    
//     {
//       std::unique_lock<std::mutex> lock(queue_mutex_);
//       queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
//         return !frame_queue_.empty() || should_shutdown_;
//       });
      
//       if (frame_queue_.empty()) {
//         continue;
//       }
      
//       frame = frame_queue_.front();
//       frame_queue_.pop();
//     }
    
//     // Get current quality settings
//     QualitySettings quality;
//     {
//       std::lock_guard<std::mutex> lock(quality_mutex_);
//       quality = current_quality_;
//     }
    
//     // Resize frame if needed
//     cv::Mat resized_frame = frame;
//     if (frame.cols != quality.width || frame.rows != quality.height) {
//       cv::resize(frame, resized_frame, cv::Size(quality.width, quality.height));
//     }
    
//     // Encode frame
//     std::vector<uint8_t> encoded_data;
//     if (encode_frame_(resized_frame, encoded_data)) {
//       // TODO: Send encoded data via RTMP
//       // RTMP_WriteFrame(rtmp_handle_, encoded_data.data(), encoded_data.size());
      
//       frames_sent_++;
//     }
//   }
  
//   std::cout << "[VideoStreamServer] Encoding thread stopped" << std::endl;
// }

// bool VideoStreamServer::encode_frame_(const cv::Mat& frame, std::vector<uint8_t>& encoded_data)
// {
//   // TODO: Implement H.264 encoding using FFmpeg
//   // For now, simulate encoding
  
//   if (frame.empty()) {
//     return false;
//   }
  
//   // Simulate encoding by creating dummy data
//   encoded_data.clear();
//   encoded_data.resize(frame.total() / 4); // Simplified
  
//   return true;
// }

// void VideoStreamServer::network_monitor_worker_()
// {
//   std::cout << "[VideoStreamServer] Network monitor thread started" << std::endl;
  
//   auto last_check = std::chrono::steady_clock::now();
  
//   while (!should_shutdown_) {
//     auto now = std::chrono::steady_clock::now();
//     auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_check);
    
//     // Monitor network metrics every 2 seconds
//     if (elapsed.count() >= 2) {
//       // TODO: Measure actual network metrics
//       // - Latency (RTT)
//       // - Packet loss
//       // - Available bandwidth
      
//       last_check = now;
//     }
    
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
//   }
  
//   std::cout << "[VideoStreamServer] Network monitor thread stopped" << std::endl;
// }

} // namespace video_stream
} // namespace camera
} // namespace eduart
