#include "v4l2_device.hpp"

#include <fcntl.h>
#include <linux/videodev2.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>
#include <utility>

namespace generic_cam
{

namespace
{

constexpr uint32_t kBufferCount = 4;

int retry_ioctl(int fd, uint64_t request, void * argument)
{
  int result = 0;
  do {
    result = ioctl(fd, request, argument);
  } while (result == -1 && errno == EINTR);
  return result;
}

void checked_ioctl(int fd, uint64_t request, void * argument, const std::string & what)
{
  if (retry_ioctl(fd, request, argument) == -1) {
    throw std::runtime_error(what + ": " + std::strerror(errno));
  }
}

uint32_t fourcc(PixelFormat format)
{
  return format == PixelFormat::Mjpeg ? V4L2_PIX_FMT_MJPEG : V4L2_PIX_FMT_YUYV;
}

std::string menu_text(int fd, const v4l2_query_ext_ctrl & query)
{
  std::string text;
  for (int64_t i = query.minimum; i <= query.maximum; ++i) {
    v4l2_querymenu item{};
    item.id = query.id;
    item.index = static_cast<uint32_t>(i);
    if (retry_ioctl(fd, VIDIOC_QUERYMENU, &item) == 0) {
      text += std::to_string(i) + "=" + reinterpret_cast<const char *>(item.name) + " ";
    }
  }
  return text;
}

}  // namespace

std::string control_param_name(const std::string & v4l2_name)
{
  std::string name;
  for (const char c : v4l2_name) {
    const auto u = static_cast<unsigned char>(c);
    if (std::isalnum(u)) {
      name += static_cast<char>(std::tolower(u));
    } else if (!name.empty() && name.back() != '_') {
      name += '_';
    }
  }
  while (!name.empty() && name.back() == '_') {
    name.pop_back();
  }
  return name;
}

V4l2Device::V4l2Device(const std::string & path, const CaptureFormat & format)
: path_(path), fd_(open(path.c_str(), O_RDWR | O_NONBLOCK))
{
  if (fd_ < 0) {
    throw std::runtime_error("open " + path_ + ": " + std::strerror(errno));
  }
  set_format(format);
  set_frame_rate(format.fps);
  enumerate_controls();
  map_buffers();
}

V4l2Device::~V4l2Device()
{
  if (streaming_) {
    v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    retry_ioctl(fd_, VIDIOC_STREAMOFF, &type);
  }
  for (const Mapping & mapping : mappings_) {
    munmap(mapping.address, mapping.length);
  }
  close(fd_);
}

void V4l2Device::set_format(const CaptureFormat & format)
{
  v4l2_format request{};
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.fmt.pix.width = format.width;
  request.fmt.pix.height = format.height;
  request.fmt.pix.pixelformat = fourcc(format.pixel_format);
  request.fmt.pix.field = V4L2_FIELD_NONE;
  checked_ioctl(fd_, VIDIOC_S_FMT, &request, path_ + " VIDIOC_S_FMT");
  // S_FMT replaces an unsupported request with the nearest mode instead of failing.
  if (
    request.fmt.pix.pixelformat != fourcc(format.pixel_format) ||
    request.fmt.pix.width != format.width || request.fmt.pix.height != format.height) {
    throw std::runtime_error(
      path_ + ": requested format is not supported, nearest is " +
      std::to_string(request.fmt.pix.width) + "x" + std::to_string(request.fmt.pix.height));
  }
}

void V4l2Device::set_frame_rate(uint32_t fps)
{
  v4l2_streamparm request{};
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.parm.capture.timeperframe.numerator = 1;
  request.parm.capture.timeperframe.denominator = fps;
  checked_ioctl(fd_, VIDIOC_S_PARM, &request, path_ + " VIDIOC_S_PARM");
  const v4l2_fract & actual = request.parm.capture.timeperframe;
  if (actual.denominator != fps * actual.numerator) {
    throw std::runtime_error(
      path_ + ": " + std::to_string(fps) + " fps is not supported in this mode, nearest is " +
      std::to_string(actual.denominator) + "/" + std::to_string(actual.numerator));
  }
}

void V4l2Device::enumerate_controls()
{
  v4l2_query_ext_ctrl query{};
  query.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  while (retry_ioctl(fd_, VIDIOC_QUERY_EXT_CTRL, &query) == 0) {
    const bool is_integer = query.type == V4L2_CTRL_TYPE_INTEGER;
    const bool is_choice =
      query.type == V4L2_CTRL_TYPE_BOOLEAN || query.type == V4L2_CTRL_TYPE_MENU;
    if ((is_integer || is_choice) && !(query.flags & V4L2_CTRL_FLAG_READ_ONLY)) {
      Control control{};
      control.id = query.id;
      control.name = control_param_name(query.name);
      control.is_integer = is_integer;
      control.minimum = static_cast<int32_t>(query.minimum);
      control.maximum = static_cast<int32_t>(query.maximum);
      control.step = static_cast<int32_t>(query.step);
      if (query.type == V4L2_CTRL_TYPE_MENU) {
        control.menu = menu_text(fd_, query);
      }
      controls_.push_back(std::move(control));
    }
    query.id |= V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_FLAG_NEXT_COMPOUND;
  }
}

int32_t V4l2Device::get_control(uint32_t id) const
{
  v4l2_control control{};
  control.id = id;
  checked_ioctl(fd_, VIDIOC_G_CTRL, &control, path_ + " VIDIOC_G_CTRL");
  return control.value;
}

std::string V4l2Device::set_control(uint32_t id, int32_t value) const
{
  v4l2_control control{};
  control.id = id;
  control.value = value;
  return retry_ioctl(fd_, VIDIOC_S_CTRL, &control) == 0 ? "" : std::strerror(errno);
}

void V4l2Device::map_buffers()
{
  v4l2_requestbuffers request{};
  request.count = kBufferCount;
  request.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  request.memory = V4L2_MEMORY_MMAP;
  checked_ioctl(fd_, VIDIOC_REQBUFS, &request, path_ + " VIDIOC_REQBUFS");
  for (uint32_t index = 0; index < request.count; ++index) {
    v4l2_buffer buffer{};
    buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buffer.memory = V4L2_MEMORY_MMAP;
    buffer.index = index;
    checked_ioctl(fd_, VIDIOC_QUERYBUF, &buffer, path_ + " VIDIOC_QUERYBUF");
    void * address = mmap(nullptr, buffer.length, PROT_READ, MAP_SHARED, fd_, buffer.m.offset);
    if (address == MAP_FAILED) {
      throw std::runtime_error(path_ + " mmap: " + std::strerror(errno));
    }
    mappings_.push_back({address, buffer.length});
  }
}

void V4l2Device::queue_buffer(uint32_t index)
{
  v4l2_buffer buffer{};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;
  buffer.index = index;
  checked_ioctl(fd_, VIDIOC_QBUF, &buffer, path_ + " VIDIOC_QBUF");
}

bool V4l2Device::dequeue_buffer(
  uint32_t & index, uint32_t & used, uint32_t & sequence, int64_t & stamp_ns, bool & error_flag)
{
  v4l2_buffer buffer{};
  buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buffer.memory = V4L2_MEMORY_MMAP;
  if (retry_ioctl(fd_, VIDIOC_DQBUF, &buffer) == -1) {
    if (errno == EAGAIN) {
      return false;
    }
    throw std::runtime_error(path_ + " VIDIOC_DQBUF: " + std::strerror(errno));
  }
  index = buffer.index;
  used = buffer.bytesused;
  sequence = buffer.sequence;
  stamp_ns = static_cast<int64_t>(buffer.timestamp.tv_sec) * 1000000000 +
             static_cast<int64_t>(buffer.timestamp.tv_usec) * 1000;
  error_flag = (buffer.flags & V4L2_BUF_FLAG_ERROR) != 0;
  return true;
}

void V4l2Device::start()
{
  for (uint32_t index = 0; index < mappings_.size(); ++index) {
    queue_buffer(index);
  }
  v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  checked_ioctl(fd_, VIDIOC_STREAMON, &type, path_ + " VIDIOC_STREAMON");
  streaming_ = true;
}

std::optional<RawFrame> V4l2Device::wait_frame(int timeout_ms)
{
  pollfd descriptor{fd_, POLLIN, 0};
  const int ready = poll(&descriptor, 1, timeout_ms);
  if (ready <= 0) {
    if (ready == -1 && errno != EINTR) {
      throw std::runtime_error(path_ + " poll: " + std::strerror(errno));
    }
    return std::nullopt;
  }

  RawFrame frame{};
  uint32_t used = 0;
  if (!dequeue_buffer(frame.index, used, frame.sequence, frame.monotonic_ns, frame.error_flag)) {
    return std::nullopt;
  }
  RawFrame newer = frame;
  while (dequeue_buffer(newer.index, used, newer.sequence, newer.monotonic_ns, newer.error_flag)) {
    queue_buffer(frame.index);
    frame = newer;
  }
  frame.data = static_cast<const uint8_t *>(mappings_[frame.index].address);
  frame.size = used;
  return frame;
}

void V4l2Device::release(const RawFrame & frame)
{
  queue_buffer(frame.index);
}

}  // namespace generic_cam
