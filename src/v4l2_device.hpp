#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace generic_cam
{

enum class PixelFormat { Mjpeg, Yuyv };

struct CaptureFormat
{
  PixelFormat pixel_format;
  uint32_t width;
  uint32_t height;
  uint32_t fps;
};

struct Control
{
  uint32_t id;
  std::string name;
  bool is_integer;
  int32_t minimum;
  int32_t maximum;
  int32_t step;
  std::string menu;
};

struct RawFrame
{
  const uint8_t * data;
  std::size_t size;
  uint32_t index;
  uint32_t sequence;
  int64_t monotonic_ns;
  bool error_flag;
};

// Same lower-case form as v4l2-ctl: "Exposure Time, Absolute" -> "exposure_time_absolute".
std::string control_param_name(const std::string & v4l2_name);

// Reads v4l2_buffer directly for the driver's timestamp, sequence number and error flag.
class V4l2Device
{
public:
  V4l2Device(const std::string & path, const CaptureFormat & format);
  ~V4l2Device();
  V4l2Device(const V4l2Device &) = delete;
  V4l2Device & operator=(const V4l2Device &) = delete;

  const std::vector<Control> & controls() const { return controls_; }
  int32_t get_control(uint32_t id) const;
  // Returns the strerror text on failure, empty on success.
  std::string set_control(uint32_t id, int32_t value) const;

  void start();
  // Waits up to timeout_ms and returns the newest ready frame; older ready frames go back to the
  // driver, which shows up as a sequence gap. Throws when the device is gone.
  std::optional<RawFrame> wait_frame(int timeout_ms);
  void release(const RawFrame & frame);

private:
  struct Mapping
  {
    void * address;
    std::size_t length;
  };

  void set_format(const CaptureFormat & format);
  void set_frame_rate(uint32_t fps);
  void enumerate_controls();
  void map_buffers();
  void queue_buffer(uint32_t index);
  bool dequeue_buffer(
    uint32_t & index, uint32_t & used, uint32_t & sequence, int64_t & stamp_ns, bool & error_flag);

  std::string path_;
  int fd_;
  bool streaming_{false};
  std::vector<Control> controls_;
  std::vector<Mapping> mappings_;
};

}  // namespace generic_cam
