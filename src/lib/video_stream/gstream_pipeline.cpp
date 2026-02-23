#include "edu_camera/video_stream/gstream_pipeline.hpp"

#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>

namespace eduart {
namespace camera {
namespace video_stream {

GstreamPipeline::GstreamPipeline(const camera::VideoCamera::Parameter& camera_parameter)
{
  _pipeline = gst_pipeline_new("video-output-pipeline");

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

  _elements["source"] = appsrc;
}

GstreamPipeline::~GstreamPipeline()
{
  if (_pipeline) {
    gst_element_set_state(_pipeline, GST_STATE_NULL);
    gst_object_unref(_pipeline);
  }
}

GstreamPipeline& GstreamPipeline::addVideoConvert(const std::string& name)
{

}

GstreamPipeline& GstreamPipeline::addVideoScale(const std::string& name)
{

}

GstreamPipeline& GstreamPipeline::addEncoderH264(const std::string& name)
{

}

GstreamPipeline& GstreamPipeline::addRtpPayloader(const std::string& name)
{

}

GstreamPipeline& GstreamPipeline::addUdpSink(const std::string& name)
{

}

GstreamPipeline& GstreamPipeline::build()
{

}

} // namespace video_stream
} // namespace camera
} // namespace eduart
