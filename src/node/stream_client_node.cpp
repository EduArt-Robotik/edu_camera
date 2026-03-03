#include <edu_camera/video_stream/video_gstream.hpp>

#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>

using eduart::camera::video_stream::GstreamPipeline;
using eduart::camera::video_stream::GstreamPipelineBuilder;
using eduart::camera::video_stream::Codec;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  GstreamPipelineBuilder builder(5000); // UDP port for receiving stream

  auto pipeline = builder.addRtpDepayloader("rtp_depay_loader")
                         .addH264Parser("h264_parser")
                         .addDecoderH264("h264_decoder")
                         .addVideoConvert("videoconvert")
                         .addAppSink("sink")
                         .build();

  cv::Mat frame;
  Codec codec;
  rclcpp::Rate loop_rate(30); // 30 Hz to cover typical video frame rates

  while (rclcpp::ok()) {
    if (pipeline->receiveFrame(frame, codec)) {
      std::cout << "frame received, size: " << frame.cols << "x" << frame.rows << ", codec: " << static_cast<int>(codec.type()) << std::endl;

      if (frame.empty()) {
        std::cerr << "Received empty frame, skipping display." << std::endl;
        continue;
      }
      // Process received frame (e.g., display or analyze)
      cv::imshow("Received Frame", frame);
      if (cv::waitKey(1) == 27) { // Exit on 'ESC' key
        break;
      }
    }

    loop_rate.sleep();
  }

  rclcpp::shutdown();
}