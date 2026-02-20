#include "edu_camera/video_stream/video_gstream.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

#include <opencv2/imgproc.hpp>

#include <string>
#include <cstring>

#include <rclcpp/logging.hpp>

namespace eduart {
namespace camera {
namespace video_stream {

VideoGstreamOutput::VideoGstreamOutput(const ::std::string& host, int port)
  : _host(host)
  , _port(port)
{
  // Initialize GStreamer
  gst_init(nullptr, nullptr);
  initializePipeline();
}

VideoGstreamOutput::~VideoGstreamOutput()
{
  cleanupPipeline();
}

void VideoGstreamOutput::initializePipeline()
{
  if (_is_initialized) {
    return;
  }

  const auto& settings = getQualitySettings();
  
  // Create pipeline elements
  _pipeline     = gst_pipeline_new("video-output-pipeline");

  _appsrc       = gst_element_factory_make("appsrc"      , "source");
  _videoconvert = gst_element_factory_make("videoconvert", "convert");
  _videoscale   = gst_element_factory_make("videoscale"  , "scale");
  _encoder      = gst_element_factory_make("x264enc"     , "encoder");
  _rtph264pay   = gst_element_factory_make("rtph264pay"  , "payloader");
  _udpsink      = gst_element_factory_make("udpsink"     , "sink");

  if (!_pipeline || !_appsrc || !_videoconvert || !_videoscale || !_encoder || !_rtph264pay || !_udpsink) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoGstreamOutput"), "failed to create GStreamer elements");
    cleanupPipeline();
    return;
  }

  // Configure appsrc
  g_object_set(G_OBJECT(_appsrc),
    "caps", gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, "BGR",
      "width", G_TYPE_INT, settings.width,
      "height", G_TYPE_INT, settings.height,
      "framerate", GST_TYPE_FRACTION, settings.fps, 1,
      nullptr),
    "stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
    "format", GST_FORMAT_TIME,
    "is-live", TRUE,
    nullptr);

  // Configure encoder with bitrate from quality settings
  g_object_set(G_OBJECT(_encoder),
    "bitrate", settings.bitrate,  // bitrate in kbps
    "tune", 0x00000004,  // zerolatency
    "speed-preset", 1,   // ultrafast
    nullptr);

  // Configure RTP payloader
  g_object_set(G_OBJECT(_rtph264pay),
    "config-interval", 1,
    "pt", 96,
    nullptr);

  // Configure UDP sink
  g_object_set(G_OBJECT(_udpsink),
    "host", _host.c_str(),
    "port", _port,
    nullptr);

  // Add elements to pipeline
  gst_bin_add_many(GST_BIN(_pipeline), _appsrc, _videoconvert, _videoscale, _encoder, _rtph264pay, _udpsink, nullptr);

  // Link elements
  if (!gst_element_link_many(_appsrc, _videoconvert, _videoscale, _encoder, _rtph264pay, _udpsink, nullptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoGstreamOutput"), "failed to link GStreamer elements.");
    cleanupPipeline();
    return;
  }

  // Start pipeline
  GstStateChangeReturn ret = gst_element_set_state(_pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoGstreamOutput"), "failed to start GStreamer pipeline.");
    cleanupPipeline();
    return;
  }

  _gst_buffer = gst_buffer_new_allocate(nullptr, 1, nullptr);

  _is_initialized = true;
  RCLCPP_INFO(
    rclcpp::get_logger("VideoGstreamOutput"),
    "GStreamer output pipeline initialized (UDP to %s:%d, bitrate: %d kbps)", _host.c_str(), _port, settings.bitrate
  );
}

void VideoGstreamOutput::cleanupPipeline()
{
  if (_pipeline) {
    gst_element_set_state(_pipeline, GST_STATE_NULL);
    gst_object_unref(_pipeline);
    _pipeline = nullptr;
  }
  
  // Child elements are automatically unreferenced when pipeline is unreferenced
  _appsrc       = nullptr;
  _videoconvert = nullptr;
  _videoscale   = nullptr;
  _encoder      = nullptr;
  _rtph264pay   = nullptr;
  _udpsink      = nullptr;

  // Buffer
  if (_gst_buffer) {
    gst_buffer_unref(_gst_buffer);
    _gst_buffer = nullptr;
  }

  // Is now uninitialized
  _is_initialized = false;
}

void VideoGstreamOutput::encodeAndSendFrame(const cv::Mat& frame)
{
  if (frame.empty()) {
    return;
  }

  // Initialize pipeline on first frame (lazy initialization)
  if (!_is_initialized) {
    throw std::runtime_error("GStreamer pipeline is not initialized.");
  }
  // Ensure BGR format
  if (frame.type() != CV_8UC3) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoGstreamOutput"), "unsupported frame format, expected CV_8UC3 (BGR).");
    throw std::runtime_error("unsupported frame format, expected CV_8UC3 (BGR).");
  }


  // Putting frame data into GStreamer buffer
  const size_t buffer_size = frame.total() * frame.elemSize();

  // Adapt frame size if it differs to previous frames (and update appsrc caps)
  if (frame.size() != _frame_size) {
    RCLCPP_INFO(
      rclcpp::get_logger("VideoGstreamOutput"),
      "Frame size changed to %dx%d, updating GStreamer caps.", frame.cols, frame.rows
    );
    _frame_size = frame.size();

    GstCaps* caps = gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, "BGR",
      "width", G_TYPE_INT, _frame_size.width,
      "height", G_TYPE_INT, _frame_size.height,
      "framerate", GST_TYPE_FRACTION, getQualitySettings().fps, 1,
      nullptr);

    g_object_set(G_OBJECT(_appsrc), "caps", caps, nullptr);
    gst_caps_unref(caps);

    // Create GStreamer buffer from OpenCV Mat
    gst_buffer_unref(_gst_buffer);
    _gst_buffer = gst_buffer_new_allocate(nullptr, buffer_size, nullptr);
  }


  GstMapInfo map;
  gst_buffer_map(_gst_buffer, &map, GST_MAP_WRITE);
  std::memcpy(map.data, frame.data, buffer_size);
  gst_buffer_unmap(_gst_buffer, &map);

  // Push buffer to appsrc
  GstFlowReturn ret;
  g_signal_emit_by_name(_appsrc, "push-buffer", _gst_buffer, &ret);

  if (ret != GST_FLOW_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoGstreamOutput"), "failed to push buffer to GStreamer pipeline");
  }
}


VideoGstreamInput::VideoGstreamInput(int port)
  : _pipeline(nullptr)
  , _udpsrc(nullptr)
  , _rtph264depay(nullptr)
  , _decoder(nullptr)
  , _videoconvert(nullptr)
  , _appsink(nullptr)
  , _port(port)
  , _is_initialized(false)
{
  // Initialize GStreamer
  gst_init(nullptr, nullptr);
}

VideoGstreamInput::~VideoGstreamInput()
{
  cleanupPipeline();
}

void VideoGstreamInput::initializePipeline()
{
  if (_is_initialized) {
    return;
  }

  // Create pipeline elements
  _pipeline = gst_pipeline_new("video-input-pipeline");
  _udpsrc = gst_element_factory_make("udpsrc", "source");
  _rtph264depay = gst_element_factory_make("rtph264depay", "depayloader");
  _decoder = gst_element_factory_make("avdec_h264", "decoder");
  _videoconvert = gst_element_factory_make("videoconvert", "convert");
  _appsink = gst_element_factory_make("appsink", "sink");

  if (!_pipeline || !_udpsrc || !_rtph264depay || !_decoder || 
      !_videoconvert || !_appsink) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoGstreamInput"), "failed to create GStreamer input elements");
    cleanupPipeline();
    return;
  }

  // Configure udpsrc
  GstCaps* caps = gst_caps_new_simple("application/x-rtp",
    "media", G_TYPE_STRING, "video",
    "clock-rate", G_TYPE_INT, 90000,
    "encoding-name", G_TYPE_STRING, "H264",
    nullptr);
  
  g_object_set(G_OBJECT(_udpsrc),
    "port", _port,
    "caps", caps,
    nullptr);
  gst_caps_unref(caps);

  // Configure appsink
  g_object_set(G_OBJECT(_appsink),
    "emit-signals", TRUE,
    "sync", FALSE,
    "max-buffers", 1,
    "drop", TRUE,
    nullptr);

  // Add elements to pipeline
  gst_bin_add_many(GST_BIN(_pipeline), _udpsrc, _rtph264depay, _decoder,
                   _videoconvert, _appsink, nullptr);

  // Link elements
  if (!gst_element_link_many(_udpsrc, _rtph264depay, _decoder, 
                             _videoconvert, _appsink, nullptr)) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoGstreamInput"), "failed to link GStreamer input elements");
    cleanupPipeline();
    return;
  }

  // Start pipeline
  GstStateChangeReturn ret = gst_element_set_state(_pipeline, GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoGstreamInput"), "failed to start GStreamer input pipeline");
    cleanupPipeline();
    return;
  }

  _is_initialized = true;
  RCLCPP_INFO(rclcpp::get_logger("VideoGstreamInput"), "GStreamer input pipeline initialized (UDP port %d)", _port);
}

void VideoGstreamInput::cleanupPipeline()
{
  if (_pipeline) {
    gst_element_set_state(_pipeline, GST_STATE_NULL);
    gst_object_unref(_pipeline);
    _pipeline = nullptr;
  }
  
  // Child elements are automatically unreferenced when pipeline is unreferenced
  _udpsrc = nullptr;
  _rtph264depay = nullptr;
  _decoder = nullptr;
  _videoconvert = nullptr;
  _appsink = nullptr;
  _is_initialized = false;
}

void VideoGstreamInput::receiveFrameAndDecode(cv::Mat& frame)
{
  // Initialize pipeline on first call
  if (!_is_initialized) {
    initializePipeline();
    if (!_is_initialized) {
      return;
    }
  }

  // Pull sample from appsink
  GstSample* sample = nullptr;
  g_signal_emit_by_name(_appsink, "pull-sample", &sample);
  
  if (!sample) {
    return;
  }

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstCaps* caps = gst_sample_get_caps(sample);
  
  if (!buffer || !caps) {
    gst_sample_unref(sample);
    return;
  }

  // Get video info from caps
  GstStructure* structure = gst_caps_get_structure(caps, 0);
  int width, height;
  gst_structure_get_int(structure, "width", &width);
  gst_structure_get_int(structure, "height", &height);

  // Map buffer and copy to cv::Mat
  GstMapInfo map;
  if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    // Assuming BGR format from videoconvert
    frame = cv::Mat(height, width, CV_8UC3, map.data).clone();
    gst_buffer_unmap(buffer, &map);
  }

  gst_sample_unref(sample);
}

} // namespace video_stream
} // namespace camera
} // namespace eduart
