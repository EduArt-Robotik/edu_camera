#include <edu_camera/analyzer/metric_latency.hpp>
#include <edu_camera/analyzer/metric_packet_loss.hpp>
#include <edu_camera/analyzer/metric_rssi.hpp>
#include <edu_camera/analyzer/metric_snr.hpp>
#include <edu_camera/analyzer/metric_link_quality.hpp>
#include <edu_camera/analyzer/wifi_quality_monitor.hpp>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

using eduart::camera::analyzer::MetricLatency;
using eduart::camera::analyzer::MetricPacketLoss;
using eduart::camera::analyzer::MetricRssi;
using eduart::camera::analyzer::MetricSnr; 
using eduart::camera::analyzer::MetricLinkQuality;
using eduart::camera::analyzer::WifiQualityMonitor;

class WifiNetworkAnalyzerNode : public rclcpp::Node
{
public:
  WifiNetworkAnalyzerNode()
    : Node("wifi_network_analyzer_node")
  {
    RCLCPP_INFO(get_logger(), "starting WiFi network analyzer node...");
    
    declare_parameter<std::string>("target_host", "8.8.8.8");
    declare_parameter<std::string>("interface", "wlan0");

    const std::string target_host = get_parameter("target_host").as_string();
    const std::string interface = get_parameter("interface").as_string();

    RCLCPP_INFO(get_logger(), "adding metrics for interface '%s'.", interface.c_str());
    _monitor.addMetric(std::make_shared<MetricRssi>(1.0f, interface, std::chrono::milliseconds(1000)));
    _monitor.addMetric(std::make_shared<MetricSnr>(1.0f, interface, std::chrono::milliseconds(1000)));
    _monitor.addMetric(std::make_shared<MetricLinkQuality>(1.0f, interface, std::chrono::milliseconds(1000)));

    RCLCPP_INFO(get_logger(), "adding metric for target host '%s'.", target_host.c_str());
    _monitor.addMetric(std::make_shared<MetricLatency>(1.0f, target_host, std::chrono::milliseconds(1000)));
    _monitor.addMetric(std::make_shared<MetricPacketLoss>(1.0f, target_host, std::chrono::milliseconds(1000)));

    _pub_score = create_publisher<std_msgs::msg::Float32>("wifi_quality_score", rclcpp::QoS(10).reliable());

    _timer = create_wall_timer(
      std::chrono::seconds(1), std::bind(&WifiNetworkAnalyzerNode::process, this)
    );

    RCLCPP_INFO(get_logger(), "WiFi network analyzer node started.");
  }

private:
  void process()
  {
    const float overall_score = _monitor.overallScore();
    RCLCPP_INFO(get_logger(), "overall WiFi quality score: %.2f", overall_score);

    std_msgs::msg::Float32 msg;
    msg.data = overall_score;

    _pub_score->publish(msg);
  }

  WifiQualityMonitor _monitor;
  rclcpp::TimerBase::SharedPtr _timer;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> _pub_score;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<WifiNetworkAnalyzerNode>());

  rclcpp::shutdown();
  
  return 0;
}
