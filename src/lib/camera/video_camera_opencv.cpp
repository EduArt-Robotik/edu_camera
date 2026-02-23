#include "edu_camera/camera/video_camera_opencv.hpp"

#include <rclcpp/logging.hpp>

namespace eduart {
namespace camera {
namespace camera {

static int get_codec_prop(const video_stream::Codec& codec)
{
  switch (codec.type()) {
    case video_stream::Codec::Type::H264:
      return cv::VideoWriter::fourcc('H', '2', '6', '4');
    case video_stream::Codec::Type::H265:
      return cv::VideoWriter::fourcc('H', '2', '6', '5');
    case video_stream::Codec::Type::MJPEG:
      return cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    case video_stream::Codec::Type::NV12:
      return cv::VideoWriter::fourcc('N', 'V', '1', '2');
    case video_stream::Codec::Type::YUYV:
      return cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V');
    case video_stream::Codec::Type::BGR:
      return cv::VideoWriter::fourcc('B', 'G', 'R', ' ');
    default:
      throw std::invalid_argument("Unsupported codec type");
  }
}

VideoCameraOpenCV::VideoCameraOpenCV(const Parameter& parameter)
  : _parameter(parameter)
{

}

VideoCameraOpenCV::~VideoCameraOpenCV() { close(); }

bool VideoCameraOpenCV::open() {
  if (_camera_device.isOpened()) {
    return true; // Already opened
  }
  if (_camera_device.open(_parameter.device_id) == false) {
    // \todo maybe just throw an exception here instead of returning false, since this is a critical error
    return false; // Failed to open camera
  }

  _camera_device.set(cv::CAP_PROP_FRAME_WIDTH, _parameter.resolution.width);
  _camera_device.set(cv::CAP_PROP_FRAME_HEIGHT, _parameter.resolution.height);
  _camera_device.set(cv::CAP_PROP_FPS, _parameter.fps);
  _camera_device.set(cv::CAP_PROP_BUFFERSIZE, 1); // Reduce latency by using a smaller buffer size
  _camera_device.set(cv::CAP_PROP_FOURCC, get_codec_prop(_parameter.codec));

  RCLCPP_INFO(rclcpp::get_logger("VideoCameraOpenCV"), "camera opened successfully");
  RCLCPP_INFO(rclcpp::get_logger("VideoCameraOpenCV"), "resolution: %dx%d", _parameter.resolution.width, _parameter.resolution.height);
  RCLCPP_INFO(rclcpp::get_logger("VideoCameraOpenCV"), "fps: %.2f", _parameter.fps);
  RCLCPP_INFO(rclcpp::get_logger("VideoCameraOpenCV"), "codec: %s", _parameter.codec.to_string().c_str());

  return true;
}

void VideoCameraOpenCV::close() {
  if (_camera_device.isOpened()) {
    _camera_device.release();
  }
}

cv::Mat VideoCameraOpenCV::captureFrame() {
  cv::Mat frame;

  if (!_camera_device.isOpened()) {
    throw std::runtime_error("Camera is not opened.");
  }
  if (!_camera_device.read(frame)) {
    throw std::runtime_error("Failed to capture frame from camera.");
  }

  return frame;
}

VideoCameraOpenCV::Parameter VideoCameraOpenCV::get_parameter(const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  Parameter parameter = default_parameter;
  static_cast<VideoCamera::Parameter&>(parameter) = VideoCamera::get_parameter(parameter, ros_node);

  ros_node.get_parameter("device_id", parameter.device_id);

  parameter.device_id = ros_node.get_parameter("device_id").as_int();

  return parameter;
}

} // namespace camera
} // namespace camera
} // namespace eduart
