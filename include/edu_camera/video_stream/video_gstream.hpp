/**
 * Copyright EduArt Robotik GmbH 2026
 *
 * Author: Christian Wendt (christian.wendt@eduart-robotik.com)
 */
#pragma once

#include "edu_camera/video_stream/video_stream.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <string>

namespace eduart {
namespace camera {
namespace video_stream {

class VideoGstreamOutput : public VideoStreamOutput
{
public:
  VideoGstreamOutput(
    const std::string& destination = "127.0.0.1", int port = 5000, const camera::VideoCamera::Parameter& camera_parameter = {});
  ~VideoGstreamOutput() override;

  void encodeAndSendFrame(const cv::Mat& frame) override;

private:
  void initializePipeline();
  void cleanupPipeline();

  GstElement* _pipeline     = nullptr;
  GstElement* _appsrc       = nullptr;
  GstElement* _videoconvert = nullptr;
  GstElement* _videoscale   = nullptr;
  GstElement* _encoder      = nullptr;
  GstElement* _rtph264pay   = nullptr;
  GstElement* _udpsink      = nullptr;
  
  std::string _destination = "127.0.0.1";
  int _port = 5000;
  bool _is_initialized = false;
  cv::Size _frame_size{0, 0};
  GstBuffer* _gst_buffer = nullptr;
};

class VideoGstreamInput : public VideoStreamInput
{
public:
  VideoGstreamInput(int port = 5000);
  ~VideoGstreamInput() override;

  void receiveFrameAndDecode(cv::Mat& frame) override;

private:
  void initializePipeline();
  void cleanupPipeline();

  GstElement* _pipeline     = nullptr;
  GstElement* _udpsrc       = nullptr;
  GstElement* _rtph264depay = nullptr;
  GstElement* _decoder      = nullptr;
  GstElement* _videoconvert = nullptr;
  GstElement* _appsink      = nullptr;
  
  int _port = 5000;
  bool _is_initialized = false;
};

} // namespace video_stream
} // namespace camera
} // namespace eduart
