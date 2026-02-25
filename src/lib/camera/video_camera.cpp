#include "edu_camera/camera/video_camera.hpp"

namespace eduart {
namespace camera {
namespace camera {

VideoCamera::Parameter VideoCamera::get_parameter(const Parameter& default_parameter, rclcpp::Node& ros_node)
{
  Parameter parameter = default_parameter;

  ros_node.declare_parameter<int>("resolution.width", parameter.resolution.width);
  ros_node.declare_parameter<int>("resolution.height", parameter.resolution.height);
  ros_node.declare_parameter<double>("fps", parameter.fps);
  ros_node.declare_parameter<std::string>("codec", parameter.codec.to_string());

  parameter.resolution.width = ros_node.get_parameter("resolution.width").as_int();
  parameter.resolution.height = ros_node.get_parameter("resolution.height").as_int();
  parameter.fps = ros_node.get_parameter("fps").as_double();
  parameter.codec = video_stream::Codec(ros_node.get_parameter("codec").as_string());

  return parameter;
}

} // end namespace camera
} // end namespace camera
} // end namespace eduart
