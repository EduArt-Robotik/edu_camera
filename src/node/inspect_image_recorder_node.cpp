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
  , camera_left_index_(42)
  , camera_right_index_(43)
  , output_dir_("/home/user/Documents/inspect_images/")
  , recording_(true)
  , interval_send_image_(2.0)
  , distance_threshold_(0.0)
  {
    output_dir_left_ = output_dir_ + "/left";
    output_dir_right_ = output_dir_ + "/right";
    fs::create_directories(output_dir_left_);
    fs::create_directories(output_dir_right_);

  
    GstreamPipelineBuilder builder_left(5001); // UDP port for receiving stream

    _pipeline_left = builder_left.addRtpDepayloader("rtp_depay_loader")
                                 .addH264Parser("h264_parser")
                                 .addDecoderH264("h264_decoder")
                                 .addVideoConvert("videoconvert")
                                 .addAppSink("sink")
                                 .build();

    GstreamPipelineBuilder builder_right(5002); // UDP port for receiving stream

    _pipeline_right = builder_right.addRtpDepayloader("rtp_depay_loader")
                                   .addH264Parser("h264_parser")
                                   .addDecoderH264("h264_decoder")
                                   .addVideoConvert("videoconvert")
                                   .addAppSink("sink")
                                   .build();

    // cap_left_.open(camera_left_index_);
    // if (!cap_left_.isOpened()) {
    //   RCLCPP_ERROR(this->get_logger(), "error opening left camera.");
    //   rclcpp::shutdown();
    //   return;
    // }
    // cap_right_.open(camera_right_index_);
    // if (!cap_right_.isOpened()) {
    //   RCLCPP_ERROR(this->get_logger(), "error opening right camera.");
    //   rclcpp::shutdown();
    //   return;
    // }

    pub_image_left_ = this->create_publisher<sensor_msgs::msg::Image>("inspect/image_left", 2);
    pub_image_right_ = this->create_publisher<sensor_msgs::msg::Image>("inspect/image_right", 2);
    srv_start_stop_ = this->create_service<std_srvs::srv::Trigger>(
      "start_stop_recording", std::bind(&InspectImageRecorderNode::start_stop_recording, this, std::placeholders::_1, std::placeholders::_2));
    sub_odometry_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odometry", 2, std::bind(&InspectImageRecorderNode::callback_odometry, this, std::placeholders::_1));
    last_publish_time_ = this->now();
  }

  ~InspectImageRecorderNode()
  {
    if (cap_left_.isOpened()) cap_left_.release();
    if (cap_right_.isOpened()) cap_right_.release();
  }

private:
  void capture_images()
  {
    cv::Mat frame_left, frame_right;
    // if (!cap_left_.read(frame_left) || !cap_right_.read(frame_right)) {
    //   RCLCPP_ERROR(this->get_logger(), "error reading camera image.");
    //   return;
    // }
    Codec codec_left, codec_right;

    if (!_pipeline_left->receiveFrame(frame_left, codec_left) || !_pipeline_right->receiveFrame(frame_right, codec_right)) {
      RCLCPP_ERROR(this->get_logger(), "error receiving frame from pipeline.");
      return;
    }

    // Save images
    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    char timestamp[32];
    std::strftime(timestamp, sizeof(timestamp), "%Y.%m.%d_%H:%M", &tm);
    std::string filename_left = output_dir_left_ + "/" + timestamp + "_inspect_left.jpg";
    std::string filename_right = output_dir_right_ + "/" + timestamp + "_inspect_right.jpg";
    cv::imwrite(filename_left, frame_left);
    cv::imwrite(filename_right, frame_right);

    RCLCPP_INFO(this->get_logger(), "image saved: %s", filename_left.c_str());
    RCLCPP_INFO(this->get_logger(), "image saved: %s", filename_right.c_str());

    // Publish images if interval has passed
    auto now = this->now();
    if ((now - last_publish_time_).seconds() > interval_send_image_) {
      std_msgs::msg::Header header;
      header.stamp = now;
      header.frame_id = "camera";
      sensor_msgs::msg::Image::SharedPtr img_msg_left = cv_bridge::CvImage(header, "bgr8", frame_left).toImageMsg();
      sensor_msgs::msg::Image::SharedPtr img_msg_right = cv_bridge::CvImage(header, "bgr8", frame_right).toImageMsg();
      pub_image_left_->publish(*img_msg_left);
      pub_image_right_->publish(*img_msg_right);
      last_publish_time_ = now;
    }
  }

  void callback_odometry(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->pose.pose.position.x < distance_threshold_)
      return;
    if (!recording_)
      return;
    RCLCPP_INFO(this->get_logger(), "distance threshold reached --> triggering image capture");
    distance_threshold_ += 0.2;
    capture_images();
  }

  void start_stop_recording(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response)
 {
    (void)request;
    recording_ = !recording_;
    response->success = true;
    response->message = recording_ ? "recording started" : "recording stopped";
  }

  int camera_left_index_;
  int camera_right_index_;
  std::string output_dir_;
  std::string output_dir_left_;
  std::string output_dir_right_;
  bool recording_;
  double interval_send_image_;
  double distance_threshold_;
  rclcpp::Time last_publish_time_;
  cv::VideoCapture cap_left_;
  cv::VideoCapture cap_right_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_left_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_right_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_start_stop_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry_;
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
