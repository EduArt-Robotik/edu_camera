#include <edu_camera/video_stream/video_stream_builder.hpp>
#include <edu_camera/video_stream/video_stream_client.hpp>

#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::camera::video_stream::VideoStreamBuilder;
using eduart::camera::video_stream::VideoStreamClient;
using eduart::camera::video_stream::Codec;

using namespace std::chrono_literals;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("video_stream_client");
  auto builder = std::make_unique<VideoStreamBuilder>();
  VideoStreamClient client(std::move(builder), *node);

  if (!client.initialize()) {
    RCLCPP_ERROR(node->get_logger(), "failed to initialize video stream client");
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "connecting to stream");

  // connect to stream
  while (!client.isConnected() && rclcpp::ok()) {
    std::cout << "Trying to connect to stream..." << std::endl;
    client.connect();
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(3s);
  }

  RCLCPP_INFO(node->get_logger(), "connected to stream, now receiving frames");

  // connected to stream, now receive frames and display them
  while (rclcpp::ok()) {
    // first spin node to process callbacks
    rclcpp::spin_some(node);

    // second receive frame from stream
    cv::Mat frame;
    Codec codec;

    if (client.receiveFrame(frame, codec, 1s)) {   
      std::cout << "got frame from stream, codec: " << codec.to_string() << std::endl; 
      if (frame.empty()) {
        RCLCPP_ERROR(node->get_logger() ,"received empty frame --> skipping displaying frame.");
        continue;
      }

      // \todo use codec information
      cv::imshow("Received Frame", frame);
      if (cv::waitKey(1) == 27) { // Exit on 'ESC' key
        break;
      }
    }

  }

  rclcpp::shutdown();
}
