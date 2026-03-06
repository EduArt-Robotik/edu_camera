// inspect_image_recorder_node.cpp
// ROS2 C++ port of inspect_image_recorder_node.py
// Publishes images from two cameras when odometry threshold is reached

#include <edu_camera/video_stream/video_stream_server.hpp>
#include <edu_camera/video_stream/video_gstream.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <string>
#include <memory>

namespace fs = std::filesystem;
using namespace std::chrono_literals;
using eduart::camera::video_stream::GstreamPipeline;
using eduart::camera::video_stream::GstreamPipelineBuilder;
using eduart::camera::video_stream::Codec;

class InspectImageRecorderNode : public rclcpp::Node
{
public:
  InspectImageRecorderNode()
  : Node("inspect_image_recorder_node")
  , _camera_left_index(42)
  , _camera_right_index(43)
  , _output_dir("/home/user/Documents/inspect_images/")
  , _recording(true)
  , _interval_send_image(2.0)
  , _distance(0.0)
  {
    _output_dir_left = _output_dir + "/left";
    _output_dir_right = _output_dir + "/right";
    fs::create_directories(_output_dir_left);
    fs::create_directories(_output_dir_right);

    // GstreamPipelineBuilder builder_left(5001); // UDP port for receiving stream

    // _pipeline_left = builder_left.addRtpDepayloader("rtp_depay_loader")
    //                              .addH264Parser("h264_parser")
    //                              .addDecoderH264("h264_decoder")
    //                              .addVideoConvert("videoconvert")
    //                              .addAppSink("sink")
    //                              .build();

    // GstreamPipelineBuilder builder_right(5002); // UDP port for receiving stream

    // _pipeline_right = builder_right.addRtpDepayloader("rtp_depay_loader")
    //                                .addH264Parser("h264_parser")
    //                                .addDecoderH264("h264_decoder")
    //                                .addVideoConvert("videoconvert")
    //                                .addAppSink("sink")
    //                                .build();

    _cap_left.open(_camera_left_index);
    if (!_cap_left.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "error opening left camera.");
      rclcpp::shutdown();
      return;
    }
    _cap_right.open(_camera_right_index);
    if (!_cap_right.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "error opening right camera.");
      rclcpp::shutdown();
      return;
    }

    _pub_image_left = this->create_publisher<sensor_msgs::msg::Image>("inspect/image_left", 10);
    _pub_image_right = this->create_publisher<sensor_msgs::msg::Image>("inspect/image_right", 10);
    _srv_start_stop = this->create_service<std_srvs::srv::Trigger>(
      "start_stop_recording", std::bind(&InspectImageRecorderNode::start_stop_recording, this, std::placeholders::_1, std::placeholders::_2));
    _sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 2, std::bind(&InspectImageRecorderNode::callback_odometry, this, std::placeholders::_1));
    _last_publish_time = this->now();
    _last_odometry = _last_publish_time;
  }

  ~InspectImageRecorderNode()
  {
    if (_cap_left.isOpened()) _cap_left.release();
    if (_cap_right.isOpened()) _cap_right.release();
  }

private:
  void capture_images()
  {
    cv::Mat frame_left, frame_right;
    if (!_cap_left.read(frame_left) || !_cap_right.read(frame_right)) {
      RCLCPP_ERROR(this->get_logger(), "error reading camera image.");
      return;
    }
    // Codec codec_left, codec_right;

    // if (!_pipeline_left->receiveFrame(frame_left, codec_left) || !_pipeline_right->receiveFrame(frame_right, codec_right)) {
    //   RCLCPP_ERROR(this->get_logger(), "error receiving frame from pipeline.");
    //   return;
    // }

    // Save images
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y.%m.%d_%H:%M:%S", &tm);
    std::string filename_left = _output_dir_left + "/" + timestamp + "_inspect_left.jpg";
    std::string filename_right = _output_dir_right + "/" + timestamp + "_inspect_right.jpg";
    cv::imwrite(filename_left, frame_left);
    cv::imwrite(filename_right, frame_right);

    RCLCPP_INFO(this->get_logger(), "image saved: %s", filename_left.c_str());
    RCLCPP_INFO(this->get_logger(), "image saved: %s", filename_right.c_str());

    // Publish images if interval has passed
    auto now = this->now();
    if ((now - _last_publish_time).seconds() > _interval_send_image) {
      std_msgs::msg::Header header;
      header.stamp = now;
      header.frame_id = "camera";
      sensor_msgs::msg::Image::SharedPtr img_msg_left = cv_bridge::CvImage(header, "bgr8", frame_left).toImageMsg();
      sensor_msgs::msg::Image::SharedPtr img_msg_right = cv_bridge::CvImage(header, "bgr8", frame_right).toImageMsg();
      _pub_image_left->publish(*img_msg_left);
      _pub_image_right->publish(*img_msg_right);
      _last_publish_time = now;
    }
  }

  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    const double dt = std::min((rclcpp::Time(msg->header.stamp) - _last_odometry).seconds(), 0.1);
    const double distance = msg->twist.twist.linear.x * dt;

    _distance += distance;

    if (_distance < 0.2) {
      return;
    }

    // distance threshold reached, capture images
    _distance = 0.0;
    RCLCPP_INFO(this->get_logger(), "distance threshold reached --> triggering image capture");
    capture_images();
  }

  void start_stop_recording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
 {
    (void)request;
    _recording = !_recording;
    response->success = true;
    response->message = _recording ? "recording started" : "recording stopped";
  }

  int _camera_left_index;
  int _camera_right_index;
  std::string _output_dir;
  std::string _output_dir_left;
  std::string _output_dir_right;
  bool _recording;
  double _interval_send_image;
  double _distance = 0.0;
  rclcpp::Time _last_publish_time;
  rclcpp::Time _last_odometry;

  cv::VideoCapture _cap_left;
  cv::VideoCapture _cap_right;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_image_left;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _pub_image_right;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srv_start_stop;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub_odometry;
  std::unique_ptr<GstreamPipeline> _pipeline_left;
  std::unique_ptr<GstreamPipeline> _pipeline_right;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InspectImageRecorderNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
