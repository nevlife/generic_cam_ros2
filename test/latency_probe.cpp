// Records one line per frame: stamp and receive time on the ROS clock.
// usage: latency_probe <topic> <frames> <output.csv>

#include <flux_wrapper/flux_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>

namespace
{

class Recorder
{
public:
  Recorder(const char * path, int64_t frames) : file_(std::fopen(path, "w")), left_(frames)
  {
    std::fprintf(file_, "stamp_ns,receive_ns\n");
  }

  ~Recorder() { std::fclose(file_); }

  void add(int64_t stamp_ns, int64_t receive_ns)
  {
    std::fprintf(file_, "%ld,%ld\n", stamp_ns, receive_ns);
    if (--left_ == 0) {
      rclcpp::shutdown();
    }
  }

private:
  std::FILE * file_;
  int64_t left_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto arguments = rclcpp::remove_ros_arguments(argc, argv);
  if (arguments.size() != 4) {
    std::fprintf(stderr, "usage: latency_probe <topic> <frames> <output.csv>\n");
    return 2;
  }
  const auto node = std::make_shared<rclcpp::Node>("latency_probe");
  Recorder recorder(arguments[3].c_str(), std::stol(arguments[2]));
  flux_wrapper::Subscription<sensor_msgs::msg::Image> subscription(
    *node, arguments[1], rclcpp::QoS(10).reliable(),
    [&recorder, &node](const flux_wrapper::View<sensor_msgs::msg::Image> & image) {
      const int64_t receive_ns = node->now().nanoseconds();
      recorder.add(rclcpp::Time(image.stamp()).nanoseconds(), receive_ns);
    });
  flux_wrapper::Executor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
