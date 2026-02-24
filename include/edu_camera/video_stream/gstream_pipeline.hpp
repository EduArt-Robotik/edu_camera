/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

struct _GstElement;
typedef struct _GstElement GstElement;

struct _GstBuffer;
typedef struct _GstBuffer GstBuffer;


#include "edu_camera/video_stream/codec.hpp"

#include "edu_camera/camera/video_camera.hpp"

#include <unordered_map>
#include <string>
#include <vector>

namespace eduart {
namespace camera {
namespace video_stream {

class GstreamPipeline
{
friend class GstreamPipelineBuilder;
public:
  constexpr static std::size_t BUFFER_SIZE = 10;

  GstreamPipeline();

  // forbid copy and move semantics
  GstreamPipeline(const GstreamPipeline&) = delete;
  GstreamPipeline(GstreamPipeline&&) = delete;
  GstreamPipeline& operator=(const GstreamPipeline&) = delete;
  GstreamPipeline& operator=(GstreamPipeline&&) = delete;

  ~GstreamPipeline();

  void set(const std::string& element_name, const char* property_name, const int value);
  void sendFrame(const cv::Mat& frame, const Codec codec);

private:
  std::unordered_map<std::string, GstElement*> _elements; // allow access to elements by name
  std::vector<GstElement*> _element_order; // to keep track of element order for linking
  GstElement* _pipeline = nullptr;
  cv::Size _frame_size{0, 0};
  std::array<cv::Mat, BUFFER_SIZE> _frame_buffer; // buffering frames until they have been send
  std::array<std::atomic<bool>, BUFFER_SIZE> _frame_sent; // flags to indicate if frame has been sent
  std::size_t _buffer_index = 0; // index to keep track of current frame in buffer
};

class GstreamPipelineBuilder
{
public:
  GstreamPipelineBuilder(const camera::VideoCamera::Parameter& camera_parameter);
  
  GstreamPipelineBuilder& addVideoConvert(const std::string& name);
  GstreamPipelineBuilder& addVideoScale(const std::string& name);
  GstreamPipelineBuilder& addEncoderH264(
    const std::string& name, const int bitrate_kbps);
  GstreamPipelineBuilder& addRtpPayloader(const std::string& name);
  GstreamPipelineBuilder& addUdpSink(
    const std::string& name, const std::string& host, const int port);

  std::unique_ptr<GstreamPipeline> build();

private:
  std::unique_ptr<GstreamPipeline> _pipeline;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
