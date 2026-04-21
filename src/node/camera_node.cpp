#include <edu_camera/camera/video_camera_opencv.hpp>
#include <edu_camera/video_stream/video_stream_server.hpp>

#include <rclcpp/rclcpp.hpp>

using eduart::camera::camera::VideoCameraOpenCV;
using eduart::camera::video_stream::QualitySettings;
using eduart::camera::video_stream::VideoStreamServer;
using eduart::camera::video_stream::VideoStreamBuilder;
using eduart::camera::video_stream::Codec;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  const QualitySettings settings{10000, 1920, 1080, 30};
  const VideoCameraOpenCV::Parameter default_camera_parameter = {
    {
      cv::Size2i(1920, 1080),
      30.0f,
      Codec(Codec::Type::MJPEG)
    },
    4
  };
  auto node = rclcpp::Node::make_shared("camera_node");

  // camera
  VideoCameraOpenCV::Parameter camera_parameter = VideoCameraOpenCV::get_parameter(default_camera_parameter, *node);
  VideoCameraOpenCV camera(camera_parameter);

  if (!camera.open()) {
    RCLCPP_FATAL(node->get_logger(), "failed to open camera --> shutting down node...");
    return -1;
  }

  // stream server
  const auto camera_parameter_applied = camera.getAppliedParameter();
  auto builder = std::make_unique<VideoStreamBuilder>();
  VideoStreamServer stream_server(camera_parameter_applied, std::move(builder), *node);

  if (!stream_server.initialize()) {
    RCLCPP_FATAL(node->get_logger(), "failed to initialize stream server --> shutting down node...");
    return -1;
  }

  // camera and stream server are ready --> start streaming
  RCLCPP_INFO(node->get_logger(), "camera and stream server are ready --> starting streaming...");

  while (rclcpp::ok()) {
    // first spin node to process callbacks
    rclcpp::spin_some(node);

    // second capture frame from camera and send it to stream server
    const cv::Mat frame = camera.captureFrame();
    stream_server.sendFrame(frame, camera_parameter_applied.codec);
  }

  camera.close();
  return 0;
}
