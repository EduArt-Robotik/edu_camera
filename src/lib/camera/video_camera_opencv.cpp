#include "edu_camera/camera/video_camera_opencv.hpp"

namespace eduart {
namespace camera {
namespace camera {

VideoCameraOpenCV::VideoCameraOpenCV(int device_id) : _device_id(device_id) {}

VideoCameraOpenCV::~VideoCameraOpenCV() { close(); }

bool VideoCameraOpenCV::open(const video_stream::QualitySettings &settings) {
  if (_camera_device.isOpened()) {
    return true; // Already opened
  }
  if (_camera_device.open(_device_id) == false) {
    return false; // Failed to open camera
  }

  _camera_device.set(cv::CAP_PROP_FRAME_WIDTH, settings.width);
  _camera_device.set(cv::CAP_PROP_FRAME_HEIGHT, settings.height);
  _camera_device.set(cv::CAP_PROP_FPS, settings.fps);
  _camera_device.set(cv::CAP_PROP_BUFFERSIZE, 1);

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

} // namespace camera
} // namespace camera
} // namespace eduart
