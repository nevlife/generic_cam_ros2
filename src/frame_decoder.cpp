#include "frame_decoder.hpp"

#include <gst/app/gstappsink.h>
#include <gst/app/gstappsrc.h>
#include <gst/gst.h>
// jpeglib.h needs FILE and size_t declared first.
#include <jpeglib.h>

#include <algorithm>
#include <csetjmp>
#include <cstdio>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace generic_cam
{

namespace
{

constexpr GstClockTime kDecodeTimeout = 500 * GST_MSECOND;

struct JpegError
{
  jpeg_error_mgr manager;
  std::jmp_buf jump;
};

// The default handler calls exit(), and a damaged USB frame must not end the process.
void jpeg_error_exit(j_common_ptr info)
{
  std::longjmp(reinterpret_cast<JpegError *>(info->err)->jump, 1);
}

void jpeg_count_warning(j_common_ptr info, int level)
{
  if (level < 0) {
    ++info->err->num_warnings;
  }
}

class LibjpegDecoder : public FrameDecoder
{
public:
  LibjpegDecoder(uint32_t width, uint32_t height) : width_(width), height_(height), rows_(height)
  {
    info_.err = jpeg_std_error(&error_.manager);
    error_.manager.error_exit = jpeg_error_exit;
    error_.manager.emit_message = jpeg_count_warning;
    jpeg_create_decompress(&info_);
  }

  ~LibjpegDecoder() override { jpeg_destroy_decompress(&info_); }

  bool decode(const uint8_t * src, std::size_t size, uint8_t * bgr) override
  {
    if (size == 0) {
      return false;
    }
    if (setjmp(error_.jump) != 0) {
      jpeg_abort_decompress(&info_);
      return false;
    }
    jpeg_mem_src(&info_, src, size);
    jpeg_read_header(&info_, TRUE);
    if (info_.image_width != width_ || info_.image_height != height_) {
      jpeg_abort_decompress(&info_);
      return false;
    }
    info_.out_color_space = JCS_EXT_BGR;
    jpeg_start_decompress(&info_);
    for (uint32_t row = 0; row < height_; ++row) {
      rows_[row] = bgr + static_cast<std::size_t>(row) * width_ * 3;
    }
    while (info_.output_scanline < height_) {
      jpeg_read_scanlines(&info_, rows_.data() + info_.output_scanline, height_);
    }
    const bool intact = error_.manager.num_warnings == 0;
    jpeg_finish_decompress(&info_);
    return intact;
  }

private:
  uint32_t width_;
  uint32_t height_;
  std::vector<JSAMPROW> rows_;
  jpeg_decompress_struct info_{};
  JpegError error_{};
};

const char * jpeg_sampling(const uint8_t * src, std::size_t size)
{
  // Only the SOF0 component table is read, so a plain marker walk is enough.
  std::size_t at = 2;
  while (at + 12 < size && src[at] == 0xFF) {
    const uint8_t marker = src[at + 1];
    if (marker == 0xC0 || marker == 0xC2) {
      return src[at + 11] == 0x22 ? "YCbCr-4:2:0" : "YCbCr-4:2:2";
    }
    at += 2 + ((static_cast<std::size_t>(src[at + 2]) << 8) | src[at + 3]);
  }
  return "YCbCr-4:2:2";
}

class GstreamerDecoder : public FrameDecoder
{
public:
  GstreamerDecoder(uint32_t width, uint32_t height, uint32_t fps, const std::string & element)
  : width_(width), height_(height), fps_(fps)
  {
    gst_init(nullptr, nullptr);
    const std::string description =
      "appsrc name=src format=time ! " + element +
      " ! videoconvert ! video/x-raw,format=BGR ! appsink name=sink sync=false";
    GError * error = nullptr;
    pipeline_ = gst_parse_launch(description.c_str(), &error);
    if (error != nullptr) {
      const std::string message = error->message;
      g_error_free(error);
      throw std::runtime_error("decoder pipeline: " + message);
    }
    src_ = gst_bin_get_by_name(GST_BIN(pipeline_), "src");
    sink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "sink");
    gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  }

  ~GstreamerDecoder() override
  {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(src_);
    gst_object_unref(sink_);
    gst_object_unref(pipeline_);
  }

  bool decode(const uint8_t * src, std::size_t size, uint8_t * bgr) override
  {
    if (size == 0) {
      return false;
    }
    if (next_pts_ == 0) {
      set_caps(src, size);
    }
    GstBuffer * buffer = gst_buffer_new_memdup(src, size);
    const GstClockTime pts = next_pts_;
    next_pts_ += GST_SECOND / fps_;
    GST_BUFFER_PTS(buffer) = pts;
    gst_app_src_push_buffer(GST_APP_SRC(src_), buffer);

    // A late sample from a previous timeout must not pair with this frame's stamp.
    while (GstSample * sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink_), kDecodeTimeout)) {
      GstBuffer * decoded = gst_sample_get_buffer(sample);
      const bool match = GST_BUFFER_PTS(decoded) == pts;
      if (match) {
        copy_rows(decoded, bgr);
      }
      gst_sample_unref(sample);
      if (match) {
        return true;
      }
    }
    throw_on_pipeline_error();
    return false;
  }

private:
  // Some elements refuse a stream instead of a frame: nvjpegdec in GStreamer 1.24 stops on 4:2:2.
  void throw_on_pipeline_error()
  {
    GstBus * bus = gst_element_get_bus(pipeline_);
    GstMessage * message = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
    gst_object_unref(bus);
    if (message != nullptr) {
      GError * error = nullptr;
      gst_message_parse_error(message, &error, nullptr);
      const std::string text = error->message;
      g_error_free(error);
      gst_message_unref(message);
      throw std::runtime_error("decoder pipeline: " + text);
    }
  }

  void set_caps(const uint8_t * src, std::size_t size)
  {
    GstCaps * caps = gst_caps_new_simple(
      "image/jpeg", "sof-marker", G_TYPE_INT, 0, "colorspace", G_TYPE_STRING, "sYUV", "sampling",
      G_TYPE_STRING, jpeg_sampling(src, size), "width", G_TYPE_INT, static_cast<int>(width_),
      "height", G_TYPE_INT, static_cast<int>(height_), "framerate", GST_TYPE_FRACTION,
      static_cast<int>(fps_), 1, "interlace-mode", G_TYPE_STRING, "progressive", nullptr);
    gst_app_src_set_caps(GST_APP_SRC(src_), caps);
    gst_caps_unref(caps);
  }

  void copy_rows(GstBuffer * decoded, uint8_t * bgr) const
  {
    GstMapInfo map{};
    gst_buffer_map(decoded, &map, GST_MAP_READ);
    const std::size_t row_bytes = static_cast<std::size_t>(width_) * 3;
    const std::size_t stride = GST_ROUND_UP_4(row_bytes);
    for (uint32_t row = 0; row < height_; ++row) {
      std::memcpy(bgr + row * row_bytes, map.data + row * stride, row_bytes);
    }
    gst_buffer_unmap(decoded, &map);
  }

  uint32_t width_;
  uint32_t height_;
  uint32_t fps_;
  GstElement * pipeline_{nullptr};
  GstElement * src_{nullptr};
  GstElement * sink_{nullptr};
  GstClockTime next_pts_{0};
};

class YuyvDecoder : public FrameDecoder
{
public:
  YuyvDecoder(uint32_t width, uint32_t height) : width_(width), height_(height) {}

  bool decode(const uint8_t * src, std::size_t size, uint8_t * bgr) override
  {
    if (size < static_cast<std::size_t>(width_) * height_ * 2) {
      return false;
    }
    yuyv_to_bgr(src, width_, height_, bgr);
    return true;
  }

private:
  uint32_t width_;
  uint32_t height_;
};

uint8_t clamp_byte(int value)
{
  return static_cast<uint8_t>(std::clamp(value, 0, 255));
}

void write_pixel(int y, int cb, int cr, uint8_t * bgr)
{
  const int luma = 298 * (y - 16);
  const int u = cb - 128;
  const int v = cr - 128;
  bgr[0] = clamp_byte((luma + 516 * u + 128) >> 8);
  bgr[1] = clamp_byte((luma - 100 * u - 208 * v + 128) >> 8);
  bgr[2] = clamp_byte((luma + 409 * v + 128) >> 8);
}

}  // namespace

std::unique_ptr<FrameDecoder> make_decoder(
  const CaptureFormat & format, const std::string & jpeg_decoder)
{
  if (format.pixel_format == PixelFormat::Yuyv) {
    return std::make_unique<YuyvDecoder>(format.width, format.height);
  }
  if (jpeg_decoder == "libjpeg") {
    return std::make_unique<LibjpegDecoder>(format.width, format.height);
  }
  return std::make_unique<GstreamerDecoder>(format.width, format.height, format.fps, jpeg_decoder);
}

void yuyv_to_bgr(const uint8_t * yuyv, uint32_t width, uint32_t height, uint8_t * bgr)
{
  const std::size_t macro_pixels = static_cast<std::size_t>(width) * height / 2;
  for (std::size_t i = 0; i < macro_pixels; ++i) {
    write_pixel(yuyv[0], yuyv[1], yuyv[3], bgr);
    write_pixel(yuyv[2], yuyv[1], yuyv[3], bgr + 3);
    yuyv += 4;
    bgr += 6;
  }
}

void rotate_bgr(
  const uint8_t * src, uint32_t width, uint32_t height, uint32_t degrees, uint8_t * dst)
{
  const bool swapped = degrees != 180;
  const std::size_t out_width = swapped ? height : width;
  for (uint32_t y = 0; y < height; ++y) {
    for (uint32_t x = 0; x < width; ++x) {
      std::size_t out_x = width - 1 - x;
      std::size_t out_y = height - 1 - y;
      if (degrees == 90) {
        out_x = height - 1 - y;
        out_y = x;
      } else if (degrees == 270) {
        out_x = y;
        out_y = width - 1 - x;
      }
      std::memcpy(dst + (out_y * out_width + out_x) * 3, src + (y * std::size_t{width} + x) * 3, 3);
    }
  }
}

}  // namespace generic_cam
