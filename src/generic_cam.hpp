#pragma once

#include "frame_decoder.hpp"
#include "rate_status.hpp"
#include "stream_monitor.hpp"
#include "v4l2_device.hpp"

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <flux/publisher.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace generic_cam
{

class GenericCam : public rclcpp::Node
{
public:
  explicit GenericCam(const rclcpp::NodeOptions & options);
  ~GenericCam() override;

private:
  struct RequestedControl
  {
    uint32_t id;
    std::string name;
    int32_t value;
  };

  struct Camera
  {
    std::string name;
    std::string frame_id;
    uint32_t rotation{0};
    uint32_t out_width{0};
    uint32_t out_height{0};
    std::size_t out_bytes{0};
    int64_t stamp_offset_ns{0};
    std::string device_path;
    CaptureFormat format;
    // Guards device, which the capture thread replaces after the camera drops off the bus.
    std::mutex device_mutex;
    std::atomic<bool> connected{true};
    std::atomic<uint64_t> reconnects{0};
    std::unique_ptr<V4l2Device> device;
    std::unique_ptr<FrameDecoder> decoder;
    std::unique_ptr<StreamMonitor> monitor;
    std::unique_ptr<RateStatus> rate_status;
    std::vector<RequestedControl> requested_controls;
    std::vector<uint8_t> rotation_scratch;
    sensor_msgs::msg::CameraInfo info;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub;
    std::unique_ptr<flux::Publisher> flux_pub;
    std::thread thread;
  };

  struct ControlBinding
  {
    Camera * camera;
    uint32_t id;
  };

  template <typename T>
  T declare_camera_parameter(const std::string & camera, const std::string & key);
  std::unique_ptr<Camera> make_camera(const std::string & name);
  void configure_monitoring(Camera & camera);
  void declare_controls(Camera & camera);
  void warn_unknown_controls(const Camera & camera);
  void make_publishers(Camera & camera);
  void apply_controls(const Camera & camera);
  void start_streaming(Camera & camera);

  void capture_loop(Camera & camera);
  void capture_frames(Camera & camera);
  bool reopen(Camera & camera);
  void handle_frame(Camera & camera, const RawFrame & frame);
  bool write_bgr(Camera & camera, const RawFrame & frame, uint8_t * bgr);
  bool publish_dds(Camera & camera, const RawFrame & frame, const rclcpp::Time & stamp);
  bool publish_flux(Camera & camera, const RawFrame & frame, const rclcpp::Time & stamp);
  void diagnose(Camera & camera, diagnostic_updater::DiagnosticStatusWrapper & status);
  rcl_interfaces::msg::SetParametersResult on_parameters(
    const std::vector<rclcpp::Parameter> & parameters);

  bool use_flux_;
  bool mirror_to_ros_;
  int64_t flux_slot_count_;
  int64_t reconnect_ms_;
  std::atomic<bool> running_{true};
  std::vector<std::unique_ptr<Camera>> cameras_;
  std::unordered_map<std::string, ControlBinding> control_bindings_;
  diagnostic_updater::Updater updater_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
};

}  // namespace generic_cam
