// Records one line per frame: stamp, publish time and receive time on the ROS clock.
// usage: latency_probe <dds|mirror|flux> <topic> <frames> <output.csv>
// mirror subscribes best effort because the flux mirror publishes best effort.

#include "../src/capture_clock.hpp"

#include <flux/executor.hpp>
#include <flux/subscription.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/flux/image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <string>
#include <utility>

using generic_cam::realtime_minus_monotonic_ns;

namespace
{

class Recorder
{
public:
  Recorder(const char * path, int64_t frames, std::function<void()> stop)
  : file_(std::fopen(path, "w")), left_(frames), stop_(std::move(stop))
  {
    std::fprintf(file_, "stamp_ns,publish_ns,receive_ns\n");
  }

  ~Recorder() { std::fclose(file_); }

  void add(int64_t stamp_ns, int64_t publish_ns, int64_t receive_ns)
  {
    std::fprintf(
      file_, "%ld,%ld,%ld\n", static_cast<int64_t>(stamp_ns), static_cast<int64_t>(publish_ns),
      static_cast<int64_t>(receive_ns));
    if (--left_ == 0) {
      stop_();
    }
  }

private:
  std::FILE * file_;
  int64_t left_;
  std::function<void()> stop_;
};

int run_dds(
  const rclcpp::Node::SharedPtr & node, const std::string & topic, const rclcpp::QoS & qos,
  Recorder & recorder)
{
  const auto subscription = node->create_subscription<sensor_msgs::msg::Image>(
    topic, qos,
    [&recorder, &node](
      const sensor_msgs::msg::Image::ConstSharedPtr & image, const rclcpp::MessageInfo & info) {
      const int64_t receive_ns = node->now().nanoseconds();
      const int64_t stamp_ns = rclcpp::Time(image->header.stamp).nanoseconds();
      recorder.add(stamp_ns, info.get_rmw_message_info().source_timestamp, receive_ns);
    });
  rclcpp::spin(node);
  return 0;
}

int run_flux(const rclcpp::Node::SharedPtr & node, const std::string & topic, Recorder & recorder)
{
  using Image = sensor_msgs::flux_msg::Image;
  flux::Subscription subscription(
    *node, topic, Image::kFingerprint, flux::QoS(1),
    [&recorder, &node](const flux::FrameView & frame) {
      const int64_t receive_ns = node->now().nanoseconds();
      const Image::View image(frame);
      const int64_t stamp_ns =
        static_cast<int64_t>(image.header__sec()) * 1000000000 + image.header__nanosec();
      const int64_t publish_ns =
        static_cast<int64_t>(frame.publish_ns()) + realtime_minus_monotonic_ns();
      recorder.add(stamp_ns, publish_ns, receive_ns);
    });
  flux::Executor executor;
  executor.add(subscription);
  executor.add_node(node);
  executor.spin();
  return 0;
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  if (arguments.size() != 5) {
    std::fprintf(stderr, "usage: latency_probe <dds|mirror|flux> <topic> <frames> <output.csv>\n");
    return 2;
  }
  const auto node = std::make_shared<rclcpp::Node>("latency_probe");
  Recorder recorder(arguments[4].c_str(), std::stol(arguments[3]), [] { rclcpp::shutdown(); });
  int code = 0;
  if (arguments[1] == "flux") {
    code = run_flux(node, arguments[2], recorder);
  } else if (arguments[1] == "mirror") {
    code = run_dds(node, arguments[2], rclcpp::QoS(1).best_effort(), recorder);
  } else {
    code = run_dds(node, arguments[2], rclcpp::QoS(10).reliable(), recorder);
  }
  rclcpp::shutdown();
  return code;
}
