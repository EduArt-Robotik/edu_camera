#include "edu_camera/video_stream/gstream_pipeline.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#include <opencv2/highgui.hpp>

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
  // std::cout << "buffer index: " << _buffer_index << std::endl;
  // std::cout << "frame size: " << frame.cols << "x" << frame.rows << std::endl;
  // std::cout << "frame elem size: " << frame.elemSize() << " bytes" << std::endl;
  // std::cout << "frame total bytes: " << frame.total() * frame.elemSize() << std::endl;

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

bool GstreamPipeline::receiveFrame(cv::Mat& frame, Codec& codec)
{
  auto it = _elements.find("sink");
  if (it == _elements.end()) {
    RCLCPP_ERROR(rclcpp::get_logger("GstreamPipeline"), "appsink element not found");
    return false;
  }

  GstElement* appsink = it->second;
  GstSample* sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));
  
  if (!sample) {
    return false;  // No frame available yet
  }

  GstCaps* sample_caps = gst_sample_get_caps(sample);
  if (sample_caps) {
    GstStructure* structure = gst_caps_get_structure(sample_caps, 0);
    int width = 0;
    int height = 0;
    if (gst_structure_get_int(structure, "width", &width) &&
        gst_structure_get_int(structure, "height", &height)) {
      _frame_size = cv::Size(width, height);
    }
  }

  GstBuffer* buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;
  gst_buffer_map(buffer, &map, GST_MAP_READ);

  const GstVideoMeta* video_meta = gst_buffer_get_video_meta(buffer);
  const int stride = video_meta ? static_cast<int>(video_meta->stride[0]) :
    static_cast<int>(map.size / static_cast<size_t>(_frame_size.height));

  // Map to cv::Mat (BGR format after videoconvert)
  frame = cv::Mat(
    _frame_size.height, 
    _frame_size.width, 
    CV_8UC3,
    map.data,
    stride
  ).clone(); // clone() is important, as map.data becomes invalid after unmap
  codec = Codec(Codec::Type::BGR); // assuming videoconvert is used to convert to BGR format, so yes it smells a bit, but it works for now

  gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);
  
  return true;  // Frame successfully received
}




GstreamPipelineBuilder::GstreamPipelineBuilder(const camera::VideoCamera::Parameter& camera_parameter, const Codec input_codec)
{
  _pipeline = std::make_unique<GstreamPipeline>();
  _pipeline->_pipeline = gst_pipeline_new("video-output-pipeline");

  // App source (entry point of pipeline)
  auto appsrc = gst_element_factory_make("appsrc", "source");
  const auto format_str = input_codec.to_string();
  GstCaps* caps = nullptr;

  // if (camera_parameter.codec.type() == Codec::Type::MJPEG) {
  //   // mjpeg compressed video
  //   caps = gst_caps_new_simple("video/x-jpeg",
  //     "width", G_TYPE_INT, camera_parameter.resolution.width,
  //     "height", G_TYPE_INT, camera_parameter.resolution.height,
  //     "framerate", GST_TYPE_FRACTION, static_cast<int>(camera_parameter.fps), 1,
  //     nullptr
  //   );
  // }
  // else {
    // raw video + mjpeg because opencv captures mjpeg as raw frames
    caps = gst_caps_new_simple("video/x-raw",
      "format", G_TYPE_STRING, format_str.c_str(),
      "width", G_TYPE_INT, camera_parameter.resolution.width,
      "height", G_TYPE_INT, camera_parameter.resolution.height,
      "framerate", GST_TYPE_FRACTION, static_cast<int>(camera_parameter.fps), 1,
      nullptr
    );
  // }

  g_object_set(G_OBJECT(appsrc),
    "caps", caps,
    "stream-type", 0, // GST_APP_STREAM_TYPE_STREAM
    "format", GST_FORMAT_TIME,
    "is-live", TRUE,
    nullptr
  );
  gst_caps_unref(caps);

  _pipeline->_elements["source"] = appsrc;
  _pipeline->_element_order.push_back(appsrc);
  _pipeline->_frame_size = camera_parameter.resolution;
}

GstreamPipelineBuilder::GstreamPipelineBuilder(const int udp_port)
{
  _pipeline = std::make_unique<GstreamPipeline>();
  _pipeline->_pipeline = gst_pipeline_new("video-input-pipeline");

  // UDP source (entry point of pipeline for receiver)
  auto udpsrc = gst_element_factory_make("udpsrc", "source");

  // Configure UDP source with RTP caps
  GstCaps* caps = gst_caps_new_simple("application/x-rtp",
    "media", G_TYPE_STRING, "video",
    "clock-rate", G_TYPE_INT, 90000,
    "payload", G_TYPE_INT, 96,
    "encoding-name", G_TYPE_STRING, "H264",
    nullptr
  );

  g_object_set(G_OBJECT(udpsrc),
    "port", udp_port,
    "caps", caps,
    nullptr
  );
  gst_caps_unref(caps);

  _pipeline->_elements["source"] = udpsrc;
  _pipeline->_element_order.push_back(udpsrc);
}

GstreamPipelineBuilder& GstreamPipelineBuilder::addDecoderMJpeg(const std::string& name)
{
  auto decoder = gst_element_factory_make("jpegdec", name.c_str());
  _pipeline->_elements[name] = decoder;
  _pipeline->_element_order.push_back(decoder);

  return *this;
}

GstreamPipelineBuilder& GstreamPipelineBuilder::addCapFilter(const std::string& name, const int width, const int height)
{
  auto capfilter = gst_element_factory_make("capsfilter", name.c_str());
  _pipeline->_elements[name] = capfilter;
  _pipeline->_element_order.push_back(capfilter);

  // Configure caps filter with specified width and height
  auto caps = gst_caps_new_simple("video/x-raw",
    "width", G_TYPE_INT, width,
    "height", G_TYPE_INT, height,
    nullptr
  );
  g_object_set(G_OBJECT(capfilter), "caps", caps, nullptr);
  gst_caps_unref(caps);

  return *this;
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

GstreamPipelineBuilder& GstreamPipelineBuilder::addRtpDepayloader(const std::string& name)
{
  auto depayloader = gst_element_factory_make("rtph264depay", name.c_str());
  _pipeline->_elements[name] = depayloader;
  _pipeline->_element_order.push_back(depayloader);

  return *this;
}

GstreamPipelineBuilder& GstreamPipelineBuilder::addH264Parser(const std::string& name)
{
  auto parser = gst_element_factory_make("h264parse", name.c_str());
  _pipeline->_elements[name] = parser;
  _pipeline->_element_order.push_back(parser);

  return *this;
}

GstreamPipelineBuilder& GstreamPipelineBuilder::addDecoderH264(const std::string& name)
{
  auto decoder = gst_element_factory_make("avdec_h264", name.c_str());
  _pipeline->_elements[name] = decoder;
  _pipeline->_element_order.push_back(decoder);

  return *this;
}

GstreamPipelineBuilder& GstreamPipelineBuilder::addAppSink(const std::string& name)
{
  auto appsink = gst_element_factory_make("appsink", name.c_str());
  _pipeline->_elements[name] = appsink;
  _pipeline->_element_order.push_back(appsink);

  GstCaps* caps = gst_caps_new_simple("video/x-raw",
    "format", G_TYPE_STRING, "BGR",
    nullptr
  );

  g_object_set(G_OBJECT(appsink),
    "emit-signals", TRUE,
    "sync", FALSE,
    "caps", caps,
    nullptr
  );

  gst_caps_unref(caps);

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

  // Set pipeline to playing state
  GstStateChangeReturn ret = gst_element_set_state(_pipeline->_pipeline, GST_STATE_PLAYING);

  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(rclcpp::get_logger("GstreamPipelineBuilder"), "failed to set GStreamer pipeline to PLAYING state");
    return nullptr;
  }
  else {
    RCLCPP_INFO(rclcpp::get_logger("GstreamPipelineBuilder"), "GStreamer pipeline set to PLAYING state");
  }

  return std::move(_pipeline);
}

} // namespace video_stream
} // namespace camera
} // namespace eduart
