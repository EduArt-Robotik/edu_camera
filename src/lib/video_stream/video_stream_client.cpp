#include "edu_camera/video_stream/video_stream_client.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>

namespace eduart {
namespace camera {
namespace video_stream {

VideoStreamClient::VideoStreamClient(std::unique_ptr<VideoStreamBuilder> builder, rclcpp::Node& node)
  : _node(node)
  , _is_initialized(false)
  , _stream_builder(std::move(builder))
{
  _client_subscribe_to_stream = node.create_client<edu_camera::srv::SubscribeToStream>(
    "subscribe_to_stream"
  );
  _client_unsubscribe_from_stream = node.create_client<edu_camera::srv::UnsubscribeFromStream>(
    "unsubscribe_from_stream"
  );
}

VideoStreamClient::~VideoStreamClient()
{
  shutdown();
  auto future = disconnect();

  rclcpp::spin_until_future_complete(_node.get_node_base_interface(), future);
}

bool VideoStreamClient::initialize()
{
  if (_is_initialized) {
    return true;
  }
  
  _is_initialized = true;
  return true;
}

void VideoStreamClient::connect(const std::string& ip_address)
{
  if (!_is_initialized) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoStreamClient"), "client not initialized");
    return;
  }
  if (!_client_subscribe_to_stream->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoStreamClient"), "service 'subscribe_to_stream' not available");
    return;
  }

  auto request = std::make_shared<edu_camera::srv::SubscribeToStream::Request>();
  request->ip = ip_address;
  request->port = 5000;
  request->pipeline = "gstream_udp";

  _client_subscribe_to_stream->async_send_request(request,
    [this, request](rclcpp::Client<edu_camera::srv::SubscribeToStream>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(
          rclcpp::get_logger("VideoStreamClient"),
          "successfully subscribed to stream with output_id: %zd", response->output_id
        );
        std::scoped_lock lock{_mutex_creating_input};
        _stream_input = _stream_builder->buildInput(request->pipeline, request->port);
        _output_id = response->output_id;
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("VideoStreamClient"), "Failed to subscribe to stream: %s", response->message.c_str());
      }
    });
}

rclcpp::Client<edu_camera::srv::UnsubscribeFromStream>::SharedFutureAndRequestId VideoStreamClient::disconnect()
{
  std::scoped_lock lock{_mutex_creating_input};

  // if no input stream exists --> do nothing
  if (_stream_input == nullptr) {
    return {{}, 0};
  }

  auto request = std::make_shared<edu_camera::srv::UnsubscribeFromStream::Request>();
  request->output_id = _output_id; 

  auto future = _client_unsubscribe_from_stream->async_send_request(request,
    [](rclcpp::Client<edu_camera::srv::UnsubscribeFromStream>::SharedFuture future) {
      auto response = future.get();
      if (response->success) {
        RCLCPP_INFO(rclcpp::get_logger("VideoStreamClient"), "successfully unsubscribed from stream");
      } else {
        RCLCPP_ERROR(
          rclcpp::get_logger("VideoStreamClient"), "failed to unsubscribe from stream: %s",
          response->message.c_str()
        );
      }
    });

  _stream_input = nullptr;
  _output_id = 0;

  return future;
}

bool VideoStreamClient::receiveFrame(cv::Mat& frame, Codec& codec, const std::chrono::nanoseconds timeout)
{
  if (!_is_initialized) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoStreamClient"), "client not initialized");
    return false;
  }  
  if (!_stream_input) {
    RCLCPP_ERROR(rclcpp::get_logger("VideoStreamClient"), "stream input not initialized");
    return false;
  }

  std::scoped_lock lock{_mutex_creating_input};
  _stream_input->receiveFrameAndDecode(frame, codec, timeout);

  return true;
}

void VideoStreamClient::shutdown()
{
  if (!_is_initialized) {
    return;
  }
  _is_initialized = false;
}

} // namespace video_stream
} // namespace camera
} // namespace eduart
