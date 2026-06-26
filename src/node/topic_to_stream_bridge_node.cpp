/**
 * \brief Bridges an image topic to an h264 video stream.
 */
#include <rclcpp/rclcpp.hpp>

#include <edu_camera/video_stream/video_stream_server.hpp>
#include <edu_camera/video_stream/video_gstream.hpp>
#include <edu_camera/camera/video_camera.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>

using eduart::camera::video_stream::QualitySettings;
using eduart::camera::video_stream::VideoStreamServer;
using eduart::camera::video_stream::VideoStreamBuilder;
using eduart::camera::video_stream::Codec;
using eduart::camera::camera::VideoCamera;

struct Parameter {
  VideoCamera::Parameter camera_parameter;
};

Parameter get_parameter(const Parameter& default_parameter, rclcpp::Node& node)
{
  Parameter parameter;
  parameter.camera_parameter = VideoCamera::get_parameter(default_parameter.camera_parameter, node);
  parameter.camera_parameter.codec = Codec(Codec::Type::BGR);
  return parameter;
}

cv::Mat receiveImageFromTopic(std::shared_ptr<rclcpp::Node> ros_node, std::shared_ptr<const sensor_msgs::msg::Image> msg)
{
  (void)ros_node;

  if (msg->encoding == "bgr8") {
    const cv::Mat image(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(msg->data.data()), msg->step);
    return image.clone();
  }
  else if (msg->encoding == "rgb8") {
    const cv::Mat image(msg->height, msg->width, CV_8UC3, const_cast<uchar*>(msg->data.data()), msg->step);
    cv::Mat bgr_image;
    cv::cvtColor(image, bgr_image, cv::COLOR_RGB2BGR);
    return bgr_image;
  }
  else if (msg->encoding == "mono8") {
    const cv::Mat image(msg->height, msg->width, CV_8UC1, const_cast<uchar*>(msg->data.data()), msg->step);
    return image.clone();
  }
  else if (msg->encoding == "jpg") {
    const cv::Mat image(1, msg->data.size(), CV_8UC1, const_cast<uchar*>(msg->data.data()));
    return cv::imdecode(image, cv::IMREAD_COLOR);
  }
  else {
    throw std::runtime_error("Unsupported image encoding: " + msg->encoding);
  }  
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto ros_node = rclcpp::Node::make_shared("topic_to_stream_bridge");
  const Parameter parameter = get_parameter({}, *ros_node);
  auto builder = std::make_unique<VideoStreamBuilder>();
  VideoStreamServer stream_server(parameter.camera_parameter, std::move(builder), *ros_node);

  if (!stream_server.initialize()) {
    RCLCPP_FATAL(ros_node->get_logger(), "failed to initialize stream server --> shutting down node...");
    rclcpp::shutdown();
    return -1;
  }
  // stream server is ready --> start bridging

  auto sub_image = ros_node->create_subscription<sensor_msgs::msg::Image>(
    "topic_bridge/image_in", rclcpp::QoS(2).best_effort(),
    [ros_node, &stream_server](std::shared_ptr<const sensor_msgs::msg::Image> msg) {
      const cv::Mat image = receiveImageFromTopic(ros_node, msg);
      stream_server.sendFrame(image, Codec(Codec::Type::BGR));
    }
  );

  rclcpp::spin(ros_node);
  rclcpp::shutdown();

  return 0;
}