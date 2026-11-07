#pragma once

#include "v4l2_device.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace generic_cam
{

class FrameDecoder
{
public:
  virtual ~FrameDecoder() = default;
  // Writes BGR to bgr. Returns false for an invalid frame; bgr may then be partly written.
  virtual bool decode(const uint8_t * src, std::size_t size, uint8_t * bgr) = 0;
};

// jpeg_decoder is "libjpeg" or a GStreamer decoder element such as vajpegdec.
std::unique_ptr<FrameDecoder> make_decoder(
  const CaptureFormat & format, const std::string & jpeg_decoder);

// BT.601 limited range, the V4L2 default for UVC YUYV.
void yuyv_to_bgr(const uint8_t * yuyv, uint32_t width, uint32_t height, uint8_t * bgr);

// degrees is clockwise: 90, 180 or 270.
void rotate_bgr(
  const uint8_t * src, uint32_t width, uint32_t height, uint32_t degrees, uint8_t * dst);

}  // namespace generic_cam
