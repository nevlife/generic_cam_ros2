#include "generic_cam.hpp"

#include "capture_clock.hpp"

#include <camera_calibration_parsers/parse.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <sensor_msgs/flux/image.hpp>

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace generic_cam
{

namespace
{

using FluxImage = sensor_msgs::flux_msg::Image;

constexpr int kFrameWaitMs = 100;
constexpr const char * kEncoding = "bgr8";
constexpr std::size_t kFluxStringBytes = 256;

PixelFormat parse_pixel_format(const std::string & name)
{
  if (name == "mjpeg") {
    return PixelFormat::Mjpeg;
  }
  if (name == "yuyv") {
    return PixelFormat::Yuyv;
  }
  throw std::invalid_argument("pixel_format must be mjpeg or yuyv, got " + name);
}

sensor_msgs::msg::CameraInfo load_camera_info(
  const std::string & path, uint32_t width, uint32_t height)
{
  sensor_msgs::msg::CameraInfo info;
  info.width = width;
  info.height = height;
  if (path.empty()) {
    return info;
  }
  std::string calibrated_name;
  if (!camera_calibration_parsers::readCalibration(path, calibrated_name, info)) {
    throw std::runtime_error("cannot read calibration " + path);
  }
  if (info.width != width || info.height != height) {
    throw std::runtime_error(
      path + " is for " + std::to_string(info.width) + "x" + std::to_string(info.height) +
      ", the published image is " + std::to_string(width) + "x" + std::to_string(height));
  }
  return info;
}

uint8_t status_byte(Level level)
{
  return static_cast<uint8_t>(level);
}

const char * rate_message(Level level)
{
  switch (level) {
    case Level::Ok:
      return "frame rate ok";
    case Level::Warn:
      return "frame rate in warning range";
    case Level::Error:
      return "frame rate out of range";
    case Level::Stale:
      break;
  }
  return "no frames yet";
}

}  // namespace

template <typename T>
T GenericCam::declare_camera_parameter(const std::string & camera, const std::string & key)
{
  const std::string common = "common." + key;
  if (!has_parameter(common)) {
    declare_parameter<T>(common);
  }
  return declare_parameter<T>(camera + "." + key, get_parameter(common).get_value<T>());
}

GenericCam::GenericCam(const rclcpp::NodeOptions & options)
: Node("generic_cam", options), updater_(this)
{
  const auto transport = declare_parameter<std::string>("transport");
  if (transport != "dds" && transport != "flux") {
    throw std::invalid_argument("transport must be dds or flux, got " + transport);
  }
  use_flux_ = transport == "flux";
  mirror_to_ros_ = declare_parameter<bool>("flux_mirror_to_ros");
  flux_slot_count_ = declare_parameter<int64_t>("flux_slot_count");
  reconnect_ms_ = declare_parameter<int64_t>("reconnect_ms");

  for (const auto & name : declare_parameter<std::vector<std::string>>("cameras")) {
    cameras_.push_back(make_camera(name));
  }
  parameter_callback_ =
    add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
      return on_parameters(parameters);
    });

  updater_.setHardwareID(get_name());
  for (const auto & camera : cameras_) {
    updater_.add(
      camera->name, [this, c = camera.get()](diagnostic_updater::DiagnosticStatusWrapper & status) {
        diagnose(*c, status);
      });
    start_streaming(*camera);
  }
  // Threads start last so that a failing camera leaves none running behind a thrown constructor.
  for (const auto & camera : cameras_) {
    Camera & c = *camera;
    c.thread = std::thread([this, &c] { capture_loop(c); });
  }
}

GenericCam::~GenericCam()
{
  running_ = false;
  for (const auto & camera : cameras_) {
    camera->thread.join();
  }
}

std::unique_ptr<GenericCam::Camera> GenericCam::make_camera(const std::string & name)
{
  auto camera = std::make_unique<Camera>();
  camera->name = name;
  camera->frame_id = declare_parameter<std::string>(name + ".frame_id");
  camera->device_path = declare_parameter<std::string>(name + ".device");
  const CaptureFormat format{
    parse_pixel_format(declare_camera_parameter<std::string>(name, "pixel_format")),
    static_cast<uint32_t>(declare_camera_parameter<int64_t>(name, "width")),
    static_cast<uint32_t>(declare_camera_parameter<int64_t>(name, "height")),
    static_cast<uint32_t>(declare_camera_parameter<int64_t>(name, "fps"))};

  camera->rotation = static_cast<uint32_t>(declare_camera_parameter<int64_t>(name, "rotation"));
  if (camera->rotation % 90 != 0 || camera->rotation >= 360) {
    throw std::invalid_argument(name + ".rotation must be 0, 90, 180 or 270");
  }
  const bool swapped = camera->rotation == 90 || camera->rotation == 270;
  camera->out_width = swapped ? format.height : format.width;
  camera->out_height = swapped ? format.width : format.height;
  camera->out_bytes = std::size_t{camera->out_width} * camera->out_height * 3;
  if (camera->rotation != 0) {
    camera->rotation_scratch.resize(camera->out_bytes);
  }
  camera->stamp_offset_ns = declare_camera_parameter<int64_t>(name, "stamp_offset_ns");
  camera->info = load_camera_info(
    declare_camera_parameter<std::string>(name, "camera_info_path"), camera->out_width,
    camera->out_height);
  camera->info.header.frame_id = camera->frame_id;

  configure_monitoring(*camera);

  camera->decoder = make_decoder(format, declare_camera_parameter<std::string>(name, "decoder"));
  camera->format = format;
  camera->device = std::make_unique<V4l2Device>(camera->device_path, format);
  declare_controls(*camera);
  make_publishers(*camera);
  RCLCPP_INFO(
    get_logger(), "[%s] %s %ux%u@%u, publishing %ux%u over %s", name.c_str(),
    camera->device_path.c_str(), format.width, format.height, format.fps, camera->out_width,
    camera->out_height, use_flux_ ? "flux" : "dds");
  return camera;
}

void GenericCam::configure_monitoring(Camera & camera)
{
  const std::string & name = camera.name;
  const auto frozen_limit = declare_camera_parameter<int64_t>(name, "frozen_frame_limit");
  camera.monitor = std::make_unique<StreamMonitor>(static_cast<uint32_t>(frozen_limit));
  const RateLimits limits{
    declare_camera_parameter<double>(name, "rate.min_ok"),
    declare_camera_parameter<double>(name, "rate.max_ok"),
    declare_camera_parameter<double>(name, "rate.min_warn"),
    declare_camera_parameter<double>(name, "rate.max_warn")};
  const auto hysteresis = declare_camera_parameter<int64_t>(name, "rate.hysteresis_updates");
  const bool immediate_error = declare_camera_parameter<bool>(name, "rate.immediate_error_report");
  const bool immediate_relax = declare_camera_parameter<bool>(name, "rate.immediate_relax_state");
  camera.rate_status = std::make_unique<RateStatus>(
    limits, static_cast<std::size_t>(hysteresis), immediate_error, immediate_relax);
}

void GenericCam::declare_controls(Camera & camera)
{
  const auto & overrides = get_node_parameters_interface()->get_parameter_overrides();
  std::vector<Control> ordered = camera.device->controls();
  // An auto mode is a bool or menu control and it must be off before its manual value is writable.
  std::stable_partition(
    ordered.begin(), ordered.end(), [](const Control & c) { return !c.is_integer; });

  for (const Control & control : ordered) {
    const std::string key = camera.name + ".controls." + control.name;
    auto found = overrides.find(key);
    if (found == overrides.end()) {
      found = overrides.find("common.controls." + control.name);
    }
    const bool requested = found != overrides.end();
    const int64_t initial =
      requested ? found->second.get<int64_t>() : camera.device->get_control(control.id);

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = control.menu;
    rcl_interfaces::msg::IntegerRange range;
    range.from_value = control.minimum;
    range.to_value = control.maximum;
    range.step = static_cast<uint64_t>(control.step);
    descriptor.integer_range.push_back(range);
    const int64_t value = declare_parameter<int64_t>(key, initial, descriptor);

    control_bindings_[key] = ControlBinding{&camera, control.id};
    if (requested) {
      camera.requested_controls.push_back({control.id, control.name, static_cast<int32_t>(value)});
    }
  }
  warn_unknown_controls(camera);
}

void GenericCam::warn_unknown_controls(const Camera & camera)
{
  const std::vector<std::string> prefixes{camera.name + ".controls.", "common.controls."};
  for (const auto & [name, value] : get_node_parameters_interface()->get_parameter_overrides()) {
    for (const auto & prefix : prefixes) {
      const bool matches = name.rfind(prefix, 0) == 0;
      if (matches && !has_parameter(camera.name + ".controls." + name.substr(prefix.size()))) {
        RCLCPP_WARN(
          get_logger(), "[%s] device has no control named by %s", camera.name.c_str(),
          name.c_str());
      }
    }
  }
}

void GenericCam::make_publishers(Camera & camera)
{
  const std::string base = "~/output/" + camera.name;
  const auto qos = rclcpp::QoS(10).reliable().durability_volatile();
  camera.info_pub = create_publisher<sensor_msgs::msg::CameraInfo>(base + "/camera_info", qos);
  if (!use_flux_) {
    camera.image_pub = create_publisher<sensor_msgs::msg::Image>(base + "/image_raw", qos);
    return;
  }
  const std::size_t slot_size = camera.out_bytes + FluxImage::kScalarBytes + kFluxStringBytes;
  camera.flux_pub = std::make_unique<flux::Publisher>(
    *this, base + "/image_raw", FluxImage::kFingerprint, slot_size,
    static_cast<std::size_t>(flux_slot_count_));
  if (mirror_to_ros_) {
    camera.flux_pub->mirror_to_ros();
  }
  RCLCPP_INFO(
    get_logger(), "[%s] flux channel %s", camera.name.c_str(),
    camera.flux_pub->signpost_name().c_str());
}

void GenericCam::apply_controls(const Camera & camera)
{
  for (const RequestedControl & control : camera.requested_controls) {
    const std::string error = camera.device->set_control(control.id, control.value);
    if (!error.empty()) {
      RCLCPP_WARN(
        get_logger(), "[%s] set %s=%d: %s", camera.name.c_str(), control.name.c_str(),
        control.value, error.c_str());
    }
  }
}

void GenericCam::start_streaming(Camera & camera)
{
  apply_controls(camera);
  camera.device->start();
  // Some ISPs drop values written before STREAMON.
  apply_controls(camera);
}

void GenericCam::capture_loop(Camera & camera)
{
  const auto context = get_node_options().context();
  while (running_ && rclcpp::ok(context)) {
    try {
      capture_frames(camera);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "[%s] capture stopped: %s", camera.name.c_str(), error.what());
      camera.connected = false;
    }
    if (reconnect_ms_ == 0) {
      return;
    }
    {
      std::lock_guard<std::mutex> lock(camera.device_mutex);
      camera.device.reset();
    }
    while (running_ && rclcpp::ok(context) && !reopen(camera)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(reconnect_ms_));
    }
  }
}

void GenericCam::capture_frames(Camera & camera)
{
  const auto context = get_node_options().context();
  while (running_ && rclcpp::ok(context)) {
    if (const auto frame = camera.device->wait_frame(kFrameWaitMs)) {
      handle_frame(camera, *frame);
      camera.device->release(*frame);
    }
  }
}

bool GenericCam::reopen(Camera & camera)
{
  std::lock_guard<std::mutex> lock(camera.device_mutex);
  try {
    camera.device = std::make_unique<V4l2Device>(camera.device_path, camera.format);
    start_streaming(camera);
  } catch (const std::exception & error) {
    camera.device.reset();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "[%s] reconnect failed: %s", camera.name.c_str(),
      error.what());
    return false;
  }
  camera.connected = true;
  ++camera.reconnects;
  RCLCPP_INFO(get_logger(), "[%s] reconnected", camera.name.c_str());
  return true;
}

void GenericCam::handle_frame(Camera & camera, const RawFrame & frame)
{
  const int64_t stamp_ns =
    frame.monotonic_ns + realtime_minus_monotonic_ns() + camera.stamp_offset_ns;
  camera.monitor->on_frame(
    {frame.monotonic_ns, frame.sequence, frame.error_flag, content_hash(frame.data, frame.size)});
  if (frame.error_flag) {
    return;
  }

  const rclcpp::Time stamp(stamp_ns, RCL_ROS_TIME);
  const bool published =
    use_flux_ ? publish_flux(camera, frame, stamp) : publish_dds(camera, frame, stamp);
  if (!published) {
    return;
  }
  auto info = std::make_unique<sensor_msgs::msg::CameraInfo>(camera.info);
  info->header.stamp = stamp;
  camera.info_pub->publish(std::move(info));
  camera.monitor->on_published(read_clock_ns(CLOCK_REALTIME) - stamp_ns);
}

bool GenericCam::write_bgr(Camera & camera, const RawFrame & frame, uint8_t * bgr)
{
  uint8_t * target = camera.rotation == 0 ? bgr : camera.rotation_scratch.data();
  if (!camera.decoder->decode(frame.data, frame.size, target)) {
    camera.monitor->on_invalid_frame();
    return false;
  }
  if (camera.rotation != 0) {
    const bool swapped = camera.rotation != 180;
    rotate_bgr(
      target, swapped ? camera.out_height : camera.out_width,
      swapped ? camera.out_width : camera.out_height, camera.rotation, bgr);
  }
  return true;
}

bool GenericCam::publish_dds(Camera & camera, const RawFrame & frame, const rclcpp::Time & stamp)
{
  auto image = std::make_unique<sensor_msgs::msg::Image>();
  image->data.resize(camera.out_bytes);
  if (!write_bgr(camera, frame, image->data.data())) {
    return false;
  }
  image->header.stamp = stamp;
  image->header.frame_id = camera.frame_id;
  image->height = camera.out_height;
  image->width = camera.out_width;
  image->encoding = kEncoding;
  image->step = camera.out_width * 3;
  camera.image_pub->publish(std::move(image));
  return true;
}

bool GenericCam::publish_flux(Camera & camera, const RawFrame & frame, const rclcpp::Time & stamp)
{
  // The decoder writes straight into the loaned slot, so the frame is never copied.
  FluxImage::Builder image = FluxImage::build__(*camera.flux_pub);
  if (!image) {
    camera.monitor->on_dropped_publish();
    return false;
  }
  const auto pixels = image.alloc__data(camera.out_bytes);
  if (!write_bgr(camera, frame, pixels.data())) {
    return false;
  }
  const builtin_interfaces::msg::Time header_stamp = stamp;
  image.set__header__stamp(header_stamp.sec, header_stamp.nanosec);
  image.set__header__frame_id(camera.frame_id);
  image.set__height(camera.out_height);
  image.set__width(camera.out_width);
  image.set__encoding(kEncoding);
  image.set__is_bigendian(0);
  image.set__step(camera.out_width * 3);
  const flux::Published result = image.commit__();
  if (result != flux::Published::Ok) {
    camera.monitor->on_dropped_publish();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "[%s] flux publish: %s", camera.name.c_str(),
      flux::to_string(result));
    return false;
  }
  return true;
}

void GenericCam::diagnose(Camera & camera, diagnostic_updater::DiagnosticStatusWrapper & status)
{
  using diagnostic_msgs::msg::DiagnosticStatus;
  const StreamReport report = camera.monitor->collect();
  const Level level = camera.rate_status->update(report.rate_hz);
  status.summary(status_byte(level), rate_message(level));
  const bool connected = camera.connected;
  if (!connected) {
    status.mergeSummary(DiagnosticStatus::ERROR, "camera disconnected");
  }
  if (report.order_errors > 0) {
    status.mergeSummary(DiagnosticStatus::ERROR, "sequence or timestamp did not increase");
  }
  if (report.frozen) {
    status.mergeSummary(DiagnosticStatus::ERROR, "image is frozen");
  }
  if (report.sequence_gaps > 0) {
    status.mergeSummary(DiagnosticStatus::WARN, "frames were lost");
  }
  if (report.error_flags > 0) {
    status.mergeSummary(DiagnosticStatus::WARN, "driver flagged corrupted buffers");
  }
  if (report.invalid_frames > 0) {
    status.mergeSummary(DiagnosticStatus::WARN, "frames failed validation");
  }
  if (report.dropped_publishes > 0) {
    status.mergeSummary(DiagnosticStatus::WARN, "publisher dropped frames");
  }
  status.add("connected", connected);
  status.addf("reconnects", "%lu", camera.reconnects.load());
  status.addf("rate_hz", "%.2f", report.rate_hz.value_or(0.0));
  status.addf("frames", "%lu", report.frames);
  status.addf("sequence_gaps", "%lu", report.sequence_gaps);
  status.addf("missed_frames", "%lu", report.missed_frames);
  status.addf("missed_frames_total", "%lu", report.missed_frames_total);
  status.addf("order_errors", "%lu", report.order_errors);
  status.addf("buffer_error_flags", "%lu", report.error_flags);
  status.addf("invalid_frames", "%lu", report.invalid_frames);
  status.addf("dropped_publishes", "%lu", report.dropped_publishes);
  status.addf("identical_frames", "%u", report.identical_frames);
  status.addf("stamp_age_at_publish_mean_ms", "%.3f", report.latency_mean_ms);
  status.addf("stamp_age_at_publish_max_ms", "%.3f", report.latency_max_ms);
}

rcl_interfaces::msg::SetParametersResult GenericCam::on_parameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const rclcpp::Parameter & parameter : parameters) {
    const auto binding = control_bindings_.find(parameter.get_name());
    if (binding == control_bindings_.end()) {
      continue;
    }
    Camera & camera = *binding->second.camera;
    const uint32_t id = binding->second.id;
    const auto value = static_cast<int32_t>(parameter.as_int());
    std::lock_guard<std::mutex> lock(camera.device_mutex);
    // Remembered first so that a reconnect restores it, also while the camera is away.
    const auto remembered = std::find_if(
      camera.requested_controls.begin(), camera.requested_controls.end(),
      [id](const RequestedControl & control) { return control.id == id; });
    if (remembered != camera.requested_controls.end()) {
      remembered->value = value;
    } else {
      camera.requested_controls.push_back({id, parameter.get_name(), value});
    }
    if (!camera.device) {
      continue;
    }
    const std::string error = camera.device->set_control(id, value);
    if (!error.empty()) {
      result.successful = false;
      result.reason = parameter.get_name() + ": " + error;
    }
  }
  return result;
}

}  // namespace generic_cam

RCLCPP_COMPONENTS_REGISTER_NODE(generic_cam::GenericCam)
