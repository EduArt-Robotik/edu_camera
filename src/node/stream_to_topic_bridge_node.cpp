/**
 * \brief Bridges an image topic to an h264 video stream.
 */
#include <edu_camera/video_stream/video_stream_client.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <rclcpp/rclcpp.hpp>

#include <ifaddrs.h>
#include <arpa/inet.h>
#include <cstring>

using eduart::camera::video_stream::VideoStreamClient;
using eduart::camera::video_stream::VideoStreamBuilder;
using eduart::camera::video_stream::Codec;

struct Parameter {
  std::string ethernet_interface = "eth0";
};

Parameter get_parameter(const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  Parameter parameter;

  ros_node.declare_parameter<std::string>("ethernet_interface", default_parameter.ethernet_interface);

  parameter.ethernet_interface = ros_node.get_parameter("ethernet_interface").as_string();

  return parameter;
}

std::string determine_video_stream_ip_address(rclcpp::Node& ros_node, const std::string& interface_name)
{
  struct ifaddrs *ifaddr, *ifa;
  std::string ip_address;

  if (getifaddrs(&ifaddr) == -1) {
    RCLCPP_ERROR(ros_node.get_logger(), "Failed to get interface addresses");
    return "";
  }

  for (ifa = ifaddr; ifa != NULL; ifa = ifa->ifa_next) {
    if (ifa->ifa_addr == NULL) continue;

    // Check if this is the interface we're looking for
    if (std::string(ifa->ifa_name) == interface_name) {
      // Get IPv4 address
      if (ifa->ifa_addr->sa_family == AF_INET) {
        char ip_str[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &((struct sockaddr_in *)ifa->ifa_addr)->sin_addr, ip_str, INET_ADDRSTRLEN);
        ip_address = ip_str;
        RCLCPP_INFO(ros_node.get_logger(), "Found IP address for interface '%s': %s", interface_name.c_str(), ip_str);
        break;
      }
    }
  }

  freeifaddrs(ifaddr);

  if (ip_address.empty()) {
    RCLCPP_ERROR(ros_node.get_logger(), "No IPv4 address found for interface '%s'", interface_name.c_str());
  }

  return ip_address;
}

sensor_msgs::msg::Image to_msg(const cv::Mat& image)
{
  sensor_msgs::msg::Image msg;
  msg.height = image.rows;
  msg.width = image.cols;
  msg.encoding = "bgr8"; // Assuming the input image is in BGR format
  msg.is_bigendian = false;
  msg.step = image.cols * image.elemSize();
  // assign data to msg. image must be kept alive as long it is used!
  // idea: publish method call serializes the image and then the image can be released
  msg.data.assign(image.data, image.data + (image.rows * image.cols * image.elemSize()));
  return msg;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto ros_node = rclcpp::Node::make_shared("stream_to_topic_bridge");
  auto pub_image = ros_node->create_publisher<sensor_msgs::msg::Image>("image_out", 10);
  const Parameter parameter = get_parameter({}, *ros_node);
  const std::string ip_address = determine_video_stream_ip_address(*ros_node, parameter.ethernet_interface);
  auto builder = std::make_unique<VideoStreamBuilder>();
  VideoStreamClient client(std::move(builder), *ros_node);

  if (!client.initialize()) {
    RCLCPP_ERROR(ros_node->get_logger(), "failed to initialize video stream client");
    return 1;
  }

  // ready to connect to stream
  RCLCPP_INFO(ros_node->get_logger(), "connecting to stream at IP address %s", ip_address.c_str());
  while (!client.isConnected() && rclcpp::ok()) {
    RCLCPP_INFO(ros_node->get_logger(), "trying to connect to stream...");
    client.connect(ip_address);
    rclcpp::spin_some(ros_node);
    std::this_thread::sleep_for(std::chrono::seconds(3));
  }
  RCLCPP_INFO(ros_node->get_logger(), "connected.");
  
  // connected to stream, now receive frames and publish them to topic until ros is shutdown
  while (rclcpp::ok()) {
    // first spin node to process callbacks
    rclcpp::spin_some(ros_node);

    // second receive frame from stream
    cv::Mat frame;
    Codec codec;

    if (client.receiveFrame(frame, codec, std::chrono::seconds(1))) {   
      if (frame.empty()) {
        RCLCPP_ERROR(ros_node->get_logger() ,"received empty frame --> skipping publishing frame.");
        continue;
      }

      // convert frame to ROS message and publish
      auto msg = to_msg(frame);
      pub_image->publish(msg);
    }
  }

  rclcpp::spin(ros_node);
  rclcpp::shutdown();

  return 0;
}