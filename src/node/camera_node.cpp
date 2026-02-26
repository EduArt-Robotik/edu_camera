#include <edu_camera/camera/video_camera_opencv.hpp>
#include <edu_camera/video_stream/video_stream_server.hpp>
#include <edu_camera/video_stream/video_gstream.hpp>

#include <rclcpp/rclcpp.hpp>

#include <iostream>

using eduart::camera::camera::VideoCameraOpenCV;
using eduart::camera::video_stream::QualitySettings;
using eduart::camera::video_stream::VideoGstreamOutput;
using eduart::camera::video_stream::VideoStreamServer;
using eduart::camera::video_stream::Codec;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  const QualitySettings settings{10000, 1920, 1080, 30};
  const VideoCameraOpenCV::Parameter camera_parameter = {
    {
      cv::Size2i(1920, 1080),
      30.0f,
      Codec(Codec::Type::MJPEG)
      // Codec(Codec::Type::YUYV)
    },
    0
  };
  const VideoGstreamOutput::Parameter stream_parameter = {
    "127.0.0.1",
    5000,
    Codec(Codec::Type::BGR),
    { 
      {"videoconvert", "videoconvert"},
      {"videoscale", "videoscale"},
      {"capfilter", "cap_filter"},
      {"encoder", "encoder_h264"},
      {"payloader", "rtp_payloader"},
      {"sink", "udp_sink"}
    }
  };

  VideoCameraOpenCV camera(camera_parameter);
  VideoStreamServer stream_server(
    std::make_unique<VideoGstreamOutput>(stream_parameter, camera_parameter, settings)
  );

  if (!camera.open()) {
    std::cerr << "Failed to open camera." << std::endl;
    return -1;
  }

  double bitrate_kbps = 1000; // Example bitrate in kbps
  std::chrono::steady_clock::time_point last_update = std::chrono::steady_clock::now();

  while (rclcpp::ok()) {
    const cv::Mat frame = camera.captureFrame();
    stream_server.sendFrame(frame, camera_parameter.codec);

    // simulate bade network
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_update);
    if (elapsed.count() < 1) {
      continue;
    }
    last_update = now;
    stream_server.setQualityManual(bitrate_kbps, 1920, 1080, 30);
    
    bitrate_kbps -= 100; // Decrease bitrate to simulate worsening network conditions
    std::cout << "Current bitrate: " << bitrate_kbps << " kbps" << std::endl;
    if (bitrate_kbps < 101) {
      bitrate_kbps = 1000; // Reset to maximum bitrate
    }
  }

  camera.close();
  return 0;
}
