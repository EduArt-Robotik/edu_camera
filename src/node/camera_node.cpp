#include <edu_camera/camera/video_camera_opencv.hpp>
#include <edu_camera/video_stream/video_stream_server.hpp>
#include <edu_camera/video_stream/video_gstream.hpp>

#include <rclcpp/rclcpp.hpp>

#include <iostream>

using eduart::camera::camera::VideoCameraOpenCV;
using eduart::camera::video_stream::QualitySettings;
using eduart::camera::video_stream::VideoGstreamOutput;
using eduart::camera::video_stream::VideoStreamServer;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  const QualitySettings settings{0, 1920, 1080, 30};
  VideoCameraOpenCV camera(0);
  VideoStreamServer stream_server(std::make_unique<VideoGstreamOutput>("127.0.0.1", 5000));
  
  if (!camera.open(settings)) {
    std::cerr << "Failed to open camera." << std::endl;
    return -1;
  }

  while (rclcpp::ok()) {
    const cv::Mat frame = camera.captureFrame();
    stream_server.sendFrame(frame);
  }

  camera.close();
  return 0;
}
