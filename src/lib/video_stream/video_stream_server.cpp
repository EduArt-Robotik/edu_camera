#include "edu_camera/video_stream/video_stream_server.hpp"
#include <iostream>
#include <chrono>
#include <cmath>

namespace edu_camera {
namespace video_stream {

VideoStreamServer::VideoStreamServer(const std::string& rtmp_url, int initial_bitrate)
  : rtmp_url_(rtmp_url)
  , is_initialized_(false)
  , should_shutdown_(false)
  , rtmp_handle_(nullptr)
  , is_connected_(false)
  , frames_sent_(0)
{
  current_quality_.bitrate = initial_bitrate;
  current_quality_.width = 640;
  current_quality_.height = 480;
  current_quality_.fps = 30;
  
  last_metrics_.latency_ms = 30;
  last_metrics_.packet_loss = 0.0f;
  last_metrics_.available_bandwidth = initial_bitrate * 2;
}

VideoStreamServer::~VideoStreamServer()
{
  shutdown();
}

bool VideoStreamServer::initialize()
{
  if (is_initialized_) {
    return true;
  }
  
  // TODO: Initialize RTMP connection using librtmp
  // RTMP_Alloc();
  // RTMP_SetupURL();
  // RTMP_Connect();
  
  std::cout << "[VideoStreamServer] Connecting to RTMP: " << rtmp_url_ << std::endl;
  
  // For now, simulate connection
  is_connected_ = true;
  is_initialized_ = true;
  
  // Start worker threads
  encoding_thread_ = std::make_unique<std::thread>(&VideoStreamServer::encoding_worker_, this);
  network_monitor_thread_ = std::make_unique<std::thread>(&VideoStreamServer::network_monitor_worker_, this);
  
  return true;
}

void VideoStreamServer::shutdown()
{
  if (!is_initialized_) {
    return;
  }
  
  should_shutdown_ = true;
  queue_cv_.notify_all();
  
  if (encoding_thread_ && encoding_thread_->joinable()) {
    encoding_thread_->join();
  }
  
  if (network_monitor_thread_ && network_monitor_thread_->joinable()) {
    network_monitor_thread_->join();
  }
  
  // TODO: Close RTMP connection
  // RTMP_Close(rtmp_handle_);
  // RTMP_Free(rtmp_handle_);
  
  is_connected_ = false;
  is_initialized_ = false;
}

bool VideoStreamServer::send_frame(const cv::Mat& frame)
{
  if (!is_initialized_ || !is_connected_) {
    return false;
  }
  
  std::lock_guard<std::mutex> lock(queue_mutex_);
  
  // Drop frame if queue is full (avoid blocking)
  if (frame_queue_.size() >= max_queue_size_) {
    return false;
  }
  
  frame_queue_.push(frame.clone());
  queue_cv_.notify_one();
  
  return true;
}

void VideoStreamServer::update_network_metrics(const NetworkMetrics& metrics)
{
  std::lock_guard<std::mutex> lock(quality_mutex_);
  last_metrics_ = metrics;
  
  // Recalculate quality based on new metrics
  QualitySettings new_quality = calculate_quality_(metrics);
  
  if (new_quality.bitrate != current_quality_.bitrate) {
    std::cout << "[VideoStreamServer] Adapting bitrate: " 
              << current_quality_.bitrate << " kbps -> " 
              << new_quality.bitrate << " kbps (latency: " 
              << metrics.latency_ms << "ms, loss: " 
              << (metrics.packet_loss * 100.0f) << "%)" << std::endl;
    
    current_quality_ = new_quality;
    update_encoder_bitrate_(new_quality.bitrate);
  }
}

QualitySettings VideoStreamServer::get_quality_settings() const
{
  std::lock_guard<std::mutex> lock(quality_mutex_);
  return current_quality_;
}

void VideoStreamServer::set_quality_manual(int bitrate, int width, int height, int fps)
{
  std::lock_guard<std::mutex> lock(quality_mutex_);
  current_quality_.bitrate = bitrate;
  current_quality_.width = width;
  current_quality_.height = height;
  current_quality_.fps = fps;
  
  update_encoder_bitrate_(bitrate);
}

bool VideoStreamServer::is_connected() const
{
  return is_connected_;
}

QualitySettings VideoStreamServer::calculate_quality_(const NetworkMetrics& metrics)
{
  QualitySettings qs = current_quality_;
  
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

void VideoStreamServer::update_encoder_bitrate_(int bitrate)
{
  // TODO: Update FFmpeg x264enc encoder bitrate
  // For RTMP with FFmpeg:
  // ffmpeg -f v4l2 -i /dev/video0 -c:v libx264 -b:v [bitrate]k -f flv rtmp://...
  
  std::cout << "[VideoStreamServer] Encoder bitrate updated to " << bitrate << " kbps" << std::endl;
}

void VideoStreamServer::encoding_worker_()
{
  std::cout << "[VideoStreamServer] Encoding thread started" << std::endl;
  
  while (!should_shutdown_) {
    cv::Mat frame;
    
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
        return !frame_queue_.empty() || should_shutdown_;
      });
      
      if (frame_queue_.empty()) {
        continue;
      }
      
      frame = frame_queue_.front();
      frame_queue_.pop();
    }
    
    // Get current quality settings
    QualitySettings quality;
    {
      std::lock_guard<std::mutex> lock(quality_mutex_);
      quality = current_quality_;
    }
    
    // Resize frame if needed
    cv::Mat resized_frame = frame;
    if (frame.cols != quality.width || frame.rows != quality.height) {
      cv::resize(frame, resized_frame, cv::Size(quality.width, quality.height));
    }
    
    // Encode frame
    std::vector<uint8_t> encoded_data;
    if (encode_frame_(resized_frame, encoded_data)) {
      // TODO: Send encoded data via RTMP
      // RTMP_WriteFrame(rtmp_handle_, encoded_data.data(), encoded_data.size());
      
      frames_sent_++;
    }
  }
  
  std::cout << "[VideoStreamServer] Encoding thread stopped" << std::endl;
}

bool VideoStreamServer::encode_frame_(const cv::Mat& frame, std::vector<uint8_t>& encoded_data)
{
  // TODO: Implement H.264 encoding using FFmpeg
  // For now, simulate encoding
  
  if (frame.empty()) {
    return false;
  }
  
  // Simulate encoding by creating dummy data
  encoded_data.clear();
  encoded_data.resize(frame.total() / 4); // Simplified
  
  return true;
}

void VideoStreamServer::network_monitor_worker_()
{
  std::cout << "[VideoStreamServer] Network monitor thread started" << std::endl;
  
  auto last_check = std::chrono::steady_clock::now();
  
  while (!should_shutdown_) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_check);
    
    // Monitor network metrics every 2 seconds
    if (elapsed.count() >= 2) {
      // TODO: Measure actual network metrics
      // - Latency (RTT)
      // - Packet loss
      // - Available bandwidth
      
      last_check = now;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  
  std::cout << "[VideoStreamServer] Network monitor thread stopped" << std::endl;
}

} // namespace video_stream
} // namespace edu_camera
