#include "edu_camera/video_stream/gstream_pipeline.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

namespace eduart {
namespace camera {
namespace video_stream {

GstreamPipeline::GstreamPipeline()

{
  // Initialize GStreamer
  gst_init(nullptr, nullptr);

  std::fill(_frame_sent.begin(), _frame_sent.end(), true);
}

GstreamPipeline::~GstreamPipeline()
{
  if (_pipeline) {
    gst_element_set_state(_pipeline, GST_STATE_NULL);
    gst_object_unref(_pipeline);
  }
}

void GstreamPipeline::set(const std::string& element_name, const char* property_name, const int value)
{
  auto it = _elements.find(element_name);

  if (it == _elements.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("GstreamPipeline"), "element '%s' not found in pipeline", element_name.c_str());
    return;
  }

  // found element, set property
  g_object_set(G_OBJECT(it->second), property_name, value, nullptr);
}

void GstreamPipeline::sendFrame(const cv::Mat& frame, const Codec codec)
{
  if (!_pipeline) {
    RCLCPP_ERROR(rclcpp::get_logger("GstreamPipeline"), "cannot send frame, pipeline is not initialized");
    return;
  }
  if (frame.empty()) {
    RCLCPP_WARN(rclcpp::get_logger("GstreamPipeline"), "empty frame, skipping");
    return;
  }
  if (frame.size() != _frame_size) {
    RCLCPP_ERROR(rclcpp::get_logger("GstreamPipeline"), "frame size %dx%d does not match pipeline frame size %dx%d",
      frame.cols, frame.rows, _frame_size.width, _frame_size.height);
    return;
  }

  // getting next buffer index in ring buffer
  if (++_buffer_index >= BUFFER_SIZE) {
    _buffer_index = 0;
  }
  if (_frame_sent[_buffer_index] == false) {
    RCLCPP_WARN(rclcpp::get_logger("GstreamPipeline"), "frame buffer overflow, dropping frame");
    return;
  }

  // getting data pointer for current buffer index
  _frame_buffer[_buffer_index] = frame; // data shared here!

  // Hole appsrc-Element
  auto it = _elements.find("source");
  if (it == _elements.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("GstreamPipeline"), "appsrc element not found");
    return;
  }

  // wrapping up data for gstream pipeline
  GstElement* appsrc = it->second;

  GstBuffer* buffer = gst_buffer_new_wrapped_full(
    GST_MEMORY_FLAG_READONLY,
    _frame_buffer[_buffer_index].data,
    _frame_buffer[_buffer_index].total() * _frame_buffer[_buffer_index].elemSize(),
    0,
    _frame_buffer[_buffer_index].total() * _frame_buffer[_buffer_index].elemSize(),
    &_frame_sent[_buffer_index],
    [](gpointer user_data) {
      auto flag = static_cast<std::atomic<bool>*>(user_data);
      *flag = true;
    }
  );

  // Buffer an appsrc pushen
  GstFlowReturn ret = gst_app_src_push_buffer(GST_APP_SRC(appsrc), buffer);
  if (ret != GST_FLOW_OK) {
    RCLCPP_ERROR(rclcpp::get_logger("GstreamPipeline"), "failed to push buffer to appsrc (GstFlowReturn=%d)", ret);
  }
}

GstreamPipelineBuilder::GstreamPipelineBuilder(const camera::VideoCamera::Parameter& camera_parameter)
{
  _pipeline = std::make_unique<GstreamPipeline>();
  _pipeline->_pipeline = gst_pipeline_new("video-output-pipeline");

  // App source (entry point of pipeline)
  auto appsrc = gst_element_factory_make("appsrc", "source");

  g_object_set(G_OBJECT(appsrc),
    "caps", gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, camera_parameter.codec.to_string().c_str(),
      "width", G_TYPE_INT, camera_parameter.resolution.width,
      "height", G_TYPE_INT, camera_parameter.resolution.height,
      "framerate", GST_TYPE_FRACTION, camera_parameter.fps, 1,
      nullptr),
    "stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
    "format", GST_FORMAT_TIME,
    "is-live", TRUE,
    nullptr
  );

  _pipeline->_elements["source"] = appsrc;
  _pipeline->_element_order.push_back(appsrc);
  _pipeline->_frame_size = camera_parameter.resolution;
}

GstreamPipelineBuilder& GstreamPipelineBuilder::addVideoConvert(const std::string& name)
{
  auto videoconvert = gst_element_factory_make("videoconvert", name.c_str());
  _pipeline->_elements[name] = videoconvert;
  _pipeline->_element_order.push_back(videoconvert);

  return *this;
}

GstreamPipelineBuilder& GstreamPipelineBuilder::addVideoScale(const std::string& name)
{
  auto videoscale = gst_element_factory_make("videoscale", name.c_str());
  _pipeline->_elements[name] = videoscale;
  _pipeline->_element_order.push_back(videoscale);

  return *this;
}

GstreamPipelineBuilder& GstreamPipelineBuilder::addEncoderH264(const std::string& name, const int bitrate_kbps)
{
  auto encoder = gst_element_factory_make("x264enc", name.c_str());
  _pipeline->_elements[name] = encoder;
  _pipeline->_element_order.push_back(encoder);

  // Configure encoder with bitrate from quality settings
  g_object_set(G_OBJECT(encoder),
    "bitrate", bitrate_kbps,  // bitrate in kbps
    "tune", 0x00000004,  // zerolatency
    "speed-preset", 1,   // ultrafast
    nullptr
  );

  return *this;
}

GstreamPipelineBuilder& GstreamPipelineBuilder::addRtpPayloader(const std::string& name)
{
  auto payloader = gst_element_factory_make("rtph264pay", name.c_str());
  _pipeline->_elements[name] = payloader;
  _pipeline->_element_order.push_back(payloader);

  // Configure RTP payloader
  g_object_set(G_OBJECT(payloader),
    "config-interval", 1,
    "pt", 96,
    nullptr
  );

  return *this;
}

GstreamPipelineBuilder& GstreamPipelineBuilder::addUdpSink(
  const std::string& name, const std::string& host, const int port)
{
  auto udpsink = gst_element_factory_make("udpsink", name.c_str());
  _pipeline->_elements[name] = udpsink;
  _pipeline->_element_order.push_back(udpsink);

  // Configure UDP sink
  g_object_set(G_OBJECT(udpsink),
    "host", host.c_str(),
    "port", port,
    nullptr
  );

  return *this;
}

std::unique_ptr<GstreamPipeline> GstreamPipelineBuilder::build()
{
  // Adding elements to pipeline
  for (const auto& elem : _pipeline->_element_order) {
    gst_bin_add(GST_BIN(_pipeline->_pipeline), elem);
  }

  // Link elements in the order they were added
  for (size_t i = 0; i < _pipeline->_element_order.size() - 1; ++i) {
    if (!gst_element_link(_pipeline->_element_order[i], _pipeline->_element_order[i + 1])) {
      RCLCPP_ERROR(rclcpp::get_logger("GstreamPipelineBuilder"), "failed to link GStreamer elements: %s -> %s",
        gst_element_get_name(_pipeline->_element_order[i]),
        gst_element_get_name(_pipeline->_element_order[i + 1])
      );
      return nullptr;
    }
  }

  return std::move(_pipeline);
}

} // namespace video_stream
} // namespace camera
} // namespace eduart
