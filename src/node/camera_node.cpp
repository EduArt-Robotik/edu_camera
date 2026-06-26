#include <edu_camera/camera/video_camera_opencv.hpp>
#include <edu_camera/video_stream/video_stream_server.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>

using eduart::camera::camera::VideoCameraOpenCV;
using eduart::camera::video_stream::QualitySettings;
using eduart::camera::video_stream::VideoStreamServer;
using eduart::camera::video_stream::VideoStreamBuilder;
using eduart::camera::video_stream::Codec;

using namespace std::chrono_literals;

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
  auto pub_image = node->create_publisher<sensor_msgs::msg::Image>(
    "image", rclcpp::QoS(10).reliable()
  );
  auto stamp_last_published = node->now();

  // camera
  VideoCameraOpenCV::Parameter camera_parameter = VideoCameraOpenCV::get_parameter(
    default_camera_parameter, *node
  );
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

    // third publish frame to ROS2 topic if there are subscribers
    if (pub_image->get_subscription_count() > 0) {
      if (node->now() - stamp_last_published < rclcpp::Duration(1s / camera_parameter.fps)) {
        continue;
      }
      stamp_last_published = node->now();
      
      std::vector<std::uint8_t> cv_image_encoded;
      cv::imencode(".jpg", frame, cv_image_encoded);

      sensor_msgs::msg::Image msg;
      msg.header.stamp = node->now();
      msg.header.frame_id = "camera";
      msg.height = frame.rows;
      msg.width = frame.cols;
      msg.encoding = "jpg";
      msg.is_bigendian = std::endian::native == std::endian::big;
      msg.step = frame.cols * frame.elemSize();
      msg.data = std::move(cv_image_encoded);
      pub_image->publish(msg);
    }
  }

  camera.close();
  return 0;
}
